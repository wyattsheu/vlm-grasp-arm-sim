"""Gemini backend: query(image, prompt) -> raw text, via caching/logging.

Deviation from AGENTS.md Sec 2 task 3 ("gemini.py"): the live system
(gemini_client.py) uses the official `google-genai` SDK, which is the
preferred choice per policy (prefer official SDKs). That package is not
installed here, and this environment cannot install it: `python3 -m venv`
fails because `python3.12-venv`/ensurepip is missing, and AGENTS.md rule 5
forbids `apt install` to fix that. This calls the public Gemini REST API
directly with `requests` (already available) instead. If a working venv
becomes available, prefer switching to `google-genai` for consistency with
the live system rather than keeping this REST client as permanent duplicate
code.

Never called with a real key in this environment: GEMINI_API_KEY is unset
(docs/questions.md). Untested against the live API; only exercised via
tests/test_vlm_backends.py with an injected fake HTTP session.
"""

from __future__ import annotations

import base64
import os
from pathlib import Path

import requests
from PIL import Image

from .base import BaseBackend, _image_bytes

DEFAULT_MODEL = "gemini-robotics-er-2-preview"
API_BASE = "https://generativelanguage.googleapis.com/v1beta"
DEFAULT_CALL_CAP = 200  # AGENTS.md rule 14


class GeminiBackend(BaseBackend):
    backend_name = "gemini"

    def __init__(
        self,
        *,
        cache_dir: Path,
        log_path: Path,
        model: str | None = None,
        call_cap: int | None = DEFAULT_CALL_CAP,
        api_key_env: str = "GEMINI_API_KEY",
        session: requests.Session | None = None,
        timeout_s: float = 60.0,
        phase_id: str = "default",
        temperature: float = 0.0,
        max_tokens: int = 1024,
        system_prompt: str | None = None,
    ):
        # Rule 13: read the key from the environment only. Never accept it
        # as a constructor argument, print it, log it, or write it to a
        # file — only ever hold it in this instance attribute.
        api_key = os.environ.get(api_key_env)
        if not api_key:
            raise RuntimeError(
                f"{api_key_env} is not set. Per AGENTS.md rule 13, ask Wyatt "
                f"rather than guessing or hardcoding a key."
            )
        self.system_prompt = system_prompt
        self.model = model or DEFAULT_MODEL
        self._api_key = api_key
        self._session = session or requests.Session()
        self._timeout_s = timeout_s
        self.generation_config = {"temperature": temperature, "max_tokens": max_tokens}
        super().__init__(cache_dir=cache_dir, log_path=log_path, call_cap=call_cap, phase_id=phase_id)

    def _call(self, image: Image.Image, prompt: str) -> str:
        url = f"{API_BASE}/models/{self.model}:generateContent"
        image_b64 = base64.b64encode(_image_bytes(image)).decode("ascii")
        body = {
            "contents": [
                {
                    "parts": [
                        {"text": prompt},
                        {"inline_data": {"mime_type": "image/png", "data": image_b64}},
                    ]
                }
            ],
            "generationConfig": {
                "temperature": self.generation_config["temperature"],
                "responseMimeType": "application/json",
                "maxOutputTokens": self.generation_config["max_tokens"],
            },
        }
        if self.system_prompt is not None:
            body["systemInstruction"] = {"parts": [{"text": self.system_prompt}]}
            body["contents"][0]["parts"].reverse()  # legacy image then user text
        # Key goes in a header, never the URL, so it can never end up in a
        # logged/echoed request URL (rule 13).
        headers = {"x-goog-api-key": self._api_key, "Content-Type": "application/json"}
        response = self._session.post(url, json=body, headers=headers, timeout=self._timeout_s)
        response.raise_for_status()
        data = response.json()
        if data.get("candidates") and data["candidates"][0].get("finishReason") == "MAX_TOKENS":
            raise RuntimeError("Gemini response truncated at token limit")
        try:
            return data["candidates"][0]["content"]["parts"][0]["text"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError(f"unexpected Gemini response shape: keys={list(data)}") from exc
