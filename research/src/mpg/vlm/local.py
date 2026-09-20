"""Local backend: query(image, prompt) -> raw text, via caching/logging.

AGENTS.md Sec 2 task 3 says to "reuse the interface from LOCAL_VLM_CODE"
but AGENTS.md Sec 0.1 separately forbids executing scripts under
LOCAL_VLM_CODE. This module does not import or execute
local_pipeline_client.py; it talks to the same underlying OpenAI-compatible
vLLM server that client already targets (confirmed healthy at
http://127.0.0.1:8000/v1 in Phase 0 recon and in out/smoke_test/), using a
freshly written request here. See docs/prompt_strategy.md Sec 5 for the
local decision tree this sits at rung 2 (L1: fixed prompt through the Qwen
endpoint). Molmo2 (port 8002, rung 3/L2) is not running in this
environment and is out of scope for this module.
"""

from __future__ import annotations

import base64
import os
from pathlib import Path

import requests
from PIL import Image

from .base import BaseBackend, _image_bytes

DEFAULT_MODEL = "Qwen/Qwen3-VL-4B-Instruct"
DEFAULT_URL = "http://127.0.0.1:8000/v1/chat/completions"


class LocalQwenBackend(BaseBackend):
    backend_name = "local_qwen"

    def __init__(
        self,
        *,
        cache_dir: Path,
        log_path: Path,
        model: str | None = None,
        url: str | None = None,
        call_cap: int | None = None,  # no shared-service $ cost; rule 12 caps concurrency, not count
        session: requests.Session | None = None,
        timeout_s: float = 60.0,
        phase_id: str = "default",
        temperature: float = 0.0,
        max_tokens: int = 1024,
        system_prompt: str | None = None,
    ):
        self.system_prompt = system_prompt
        self.model = model or os.getenv("ROBOT129_LOCAL_VLLM_MODEL", DEFAULT_MODEL)
        self._url = url or os.getenv("ROBOT129_LOCAL_VLLM_URL", DEFAULT_URL)
        self._session = session or requests.Session()
        self._timeout_s = timeout_s
        self.generation_config = {"temperature": temperature, "max_tokens": max_tokens}
        super().__init__(cache_dir=cache_dir, log_path=log_path, call_cap=call_cap, phase_id=phase_id)

    def _call(self, image: Image.Image, prompt: str) -> str:
        image_b64 = base64.b64encode(_image_bytes(image)).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{image_b64}"},
                        },
                    ],
                }
            ],
            "max_tokens": self.generation_config["max_tokens"],
            "temperature": self.generation_config["temperature"],
        }
        if self.system_prompt is not None:
            payload["messages"].insert(0, {"role": "system", "content": self.system_prompt})
        response = self._session.post(self._url, json=payload, timeout=self._timeout_s)
        response.raise_for_status()
        data = response.json()
        if data.get("choices") and data["choices"][0].get("finish_reason") == "length":
            raise RuntimeError("local response truncated at token limit")
        try:
            return data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError(f"unexpected local backend response shape: keys={list(data)}") from exc
