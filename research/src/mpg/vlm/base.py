"""Shared backend contract, disk cache, and call ledger.

AGENTS.md rule 14: cache every VLM response on disk (WORK_DIR/cache/,
keyed by a hash of model + prompt + image) and log every REAL call (not
cache hits) to WORK_DIR/logs/vlm_calls.jsonl with timestamp, model, and
latency. Both backends (gemini.py, local.py) share this so caching/logging
behavior cannot drift between them.
"""

from __future__ import annotations

import hashlib
import fcntl
import json
import os
import threading
import time
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

from PIL import Image


class CallBudgetExceeded(RuntimeError):
    """Raised when a backend would exceed its configured real-call cap.

    AGENTS.md rule 14: "Default cap is 200 real Gemini calls per phase;
    ask before exceeding it." Catch this and get explicit confirmation
    before passing allow_over_budget=True to retry.
    """


@dataclass(frozen=True)
class QueryResult:
    raw_text: str
    latency_s: float
    cache_hit: bool
    model: str
    request_id: str | None = None


class VLMBackend(Protocol):
    """Both backends expose this same call shape (AGENTS.md Sec 2 task 3)."""

    def query(
        self, image: Image.Image, prompt: str, *, replicate_id: str = ""
    ) -> QueryResult: ...


def _image_bytes(image: Image.Image) -> bytes:
    import io

    buf = io.BytesIO()
    image.save(buf, format="PNG")
    return buf.getvalue()


def cache_key(*, model: str, prompt: str, image: Image.Image, replicate_id: str = "") -> str:
    """Hash of model + prompt text + actual image bytes (+ optional
    replicate_id). Different replicate_ids deliberately produce different
    keys so a consistency-test caller (docs/data_and_evaluation.md: repeat
    each query 3x with independent samples) can bypass the cache on
    purpose rather than replaying the same cached response three times."""
    digest = hashlib.sha256()
    digest.update(model.encode("utf-8"))
    digest.update(b"\0")
    digest.update(prompt.encode("utf-8"))
    digest.update(b"\0")
    digest.update(_image_bytes(image))
    digest.update(b"\0")
    digest.update(replicate_id.encode("utf-8"))
    return digest.hexdigest()


class DiskCache:
    def __init__(self, cache_dir: Path):
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)

    def _path(self, key: str) -> Path:
        return self.cache_dir / f"{key}.json"

    def get(self, key: str) -> str | None:
        path = self._path(key)
        if not path.is_file():
            return None
        return json.loads(path.read_text())["raw_text"]

    def put(self, key: str, *, raw_text: str, model: str, latency_s: float) -> None:
        entry = {
            "raw_text": raw_text,
            "model": model,
            "latency_s": latency_s,
            "cached_at": datetime.now(timezone.utc).isoformat(),
        }
        path = self._path(key)
        staging = path.with_suffix(f".{uuid.uuid4().hex}.tmp")
        try:
            staging.write_text(json.dumps(entry, ensure_ascii=False, indent=2))
            staging.replace(path)
        finally:
            staging.unlink(missing_ok=True)


class CallLedger:
    """Append-only log of REAL (non-cached) calls, and the source of truth
    for the per-backend call-budget check."""

    def __init__(self, log_path: Path):
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, *, backend: str, model: str, latency_s: float, **fields) -> None:
        entry = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "backend": backend,
            "model": model,
            "latency_s": latency_s,
            **fields,
        }
        with self.log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
            f.flush()
            os.fsync(f.fileno())

    def count(self, *, backend: str, phase_id: str = "default") -> int:
        if not self.log_path.is_file():
            return 0
        n = 0
        with self.log_path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                entry = json.loads(line)
                if (entry.get("backend") == backend
                    and entry.get("event", "attempt") == "attempt"
                    and ("phase_id" not in entry or entry["phase_id"] == phase_id)):
                    n += 1
        return n


class BaseBackend(ABC):
    """Common caching/budget/single-flight plumbing. Subclasses implement
    only the actual transport in _call()."""

    backend_name: str
    model: str

    def __init__(
        self,
        *,
        cache_dir: Path,
        log_path: Path,
        call_cap: int | None = None,
        phase_id: str = "default",
    ):
        self._cache = DiskCache(cache_dir)
        self._ledger = CallLedger(log_path)
        self._call_cap = call_cap
        self.phase_id = phase_id
        # AGENTS.md rule 12: "Run one inference at a time" — serialize
        # calls through this backend instance rather than trusting callers
        # not to fan out concurrently.
        self._lock = threading.Lock()

    @abstractmethod
    def _call(self, image: Image.Image, prompt: str) -> str:
        """Perform the actual network call and return raw response text."""

    def query(
        self,
        image: Image.Image,
        prompt: str,
        *,
        replicate_id: str = "",
        allow_over_budget: bool = False,
    ) -> QueryResult:
        identity = json.dumps({"backend": self.backend_name, "model": self.model,
            "endpoint": getattr(self, "_url", "gemini"),
            "system_prompt": getattr(self, "system_prompt", None),
            "generation": getattr(self, "generation_config", {}), "cache_version": 2}, sort_keys=True)
        key = cache_key(model=identity, prompt=prompt, image=image, replicate_id=replicate_id)
        # Advisory process lock coordinates all clients sharing this ledger.
        # Holding it over transport also prevents concurrent local inference.
        with self._lock, self._ledger.log_path.with_suffix(".lock").open("a") as lock_file:
            fcntl.flock(lock_file, fcntl.LOCK_EX)
            try:
                cached = self._cache.get(key)
                if cached is not None:
                    return QueryResult(raw_text=cached, latency_s=0.0, cache_hit=True, model=self.model)
                used = self._ledger.count(backend=self.backend_name, phase_id=self.phase_id)
                if self._call_cap is not None and not allow_over_budget and used >= self._call_cap:
                    raise CallBudgetExceeded(f"{self.backend_name}: attempts={used}, cap={self._call_cap}")
                request_id = uuid.uuid4().hex
                fields = dict(backend=self.backend_name, model=self.model,
                              phase_id=self.phase_id, request_id=request_id,
                              cache_key=key, replicate_id=replicate_id)
                self._ledger.append(**fields, event="attempt", latency_s=0.0)
                t0 = time.monotonic()
                try:
                    raw_text = self._call(image, prompt)
                    if not isinstance(raw_text, str):
                        raise TypeError("backend response text must be a string")
                except Exception as exc:
                    # Only the class is logged: transport exceptions can contain secrets.
                    self._ledger.append(**fields, event="failed", latency_s=time.monotonic()-t0,
                                        error_class=type(exc).__name__, remote_completion="unknown")
                    raise
                latency_s = time.monotonic() - t0
                self._ledger.append(**fields, event="completed", latency_s=latency_s)
                self._cache.put(key, raw_text=raw_text, model=self.model, latency_s=latency_s)
                return QueryResult(raw_text=raw_text, latency_s=latency_s, cache_hit=False,
                                   model=self.model, request_id=request_id)
            finally:
                fcntl.flock(lock_file, fcntl.LOCK_UN)
