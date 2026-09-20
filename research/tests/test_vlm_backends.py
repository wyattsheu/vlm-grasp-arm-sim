from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from PIL import Image  # noqa: E402

from mpg.vlm import CallBudgetExceeded, GeminiBackend, LocalQwenBackend  # noqa: E402


class _FakeResponse:
    def __init__(self, json_body: dict, status: int = 200):
        self._json = json_body
        self.status_code = status

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise RuntimeError(f"HTTP {self.status_code}")

    def json(self) -> dict:
        return self._json


class _FakeSession:
    """Counts POSTs and returns a canned response; no real network I/O."""

    def __init__(self, response_json: dict):
        self.response_json = response_json
        self.post_count = 0
        self.last_url = None
        self.last_headers = None

    def post(self, url, json=None, headers=None, timeout=None):  # noqa: A002
        self.post_count += 1
        self.last_url = url
        self.last_headers = headers
        return _FakeResponse(self.response_json)


def _tmp_image() -> Image.Image:
    return Image.new("RGB", (4, 4), color=(1, 2, 3))


class LocalQwenBackendTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.cache_dir = Path(self.tmp.name) / "cache"
        self.log_path = Path(self.tmp.name) / "logs" / "vlm_calls.jsonl"

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _backend(self, session: _FakeSession) -> LocalQwenBackend:
        return LocalQwenBackend(
            cache_dir=self.cache_dir, log_path=self.log_path, session=session
        )

    def test_real_call_is_cached_and_logged(self) -> None:
        session = _FakeSession({"choices": [{"message": {"content": '{"ok": true}'}}]})
        backend = self._backend(session)
        result = backend.query(_tmp_image(), "prompt A")
        self.assertFalse(result.cache_hit)
        self.assertEqual(result.raw_text, '{"ok": true}')
        self.assertEqual(session.post_count, 1)

        lines = self.log_path.read_text().strip().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertEqual(json.loads(lines[0])["event"], "attempt")
        self.assertEqual(json.loads(lines[1])["event"], "completed")
        entry = json.loads(lines[0])
        self.assertEqual(entry["backend"], "local_qwen")

    def test_second_identical_call_is_a_cache_hit_and_skips_network(self) -> None:
        session = _FakeSession({"choices": [{"message": {"content": '{"ok": true}'}}]})
        backend = self._backend(session)
        backend.query(_tmp_image(), "prompt A")
        result2 = backend.query(_tmp_image(), "prompt A")
        self.assertTrue(result2.cache_hit)
        self.assertEqual(session.post_count, 1)  # not called again

    def test_different_replicate_id_bypasses_cache(self) -> None:
        session = _FakeSession({"choices": [{"message": {"content": '{"ok": true}'}}]})
        backend = self._backend(session)
        backend.query(_tmp_image(), "prompt A", replicate_id="r1")
        result2 = backend.query(_tmp_image(), "prompt A", replicate_id="r2")
        self.assertFalse(result2.cache_hit)
        self.assertEqual(session.post_count, 2)

    def test_different_prompt_is_a_different_cache_entry(self) -> None:
        session = _FakeSession({"choices": [{"message": {"content": '{"ok": true}'}}]})
        backend = self._backend(session)
        backend.query(_tmp_image(), "prompt A")
        result2 = backend.query(_tmp_image(), "prompt B")
        self.assertFalse(result2.cache_hit)
        self.assertEqual(session.post_count, 2)

    def test_malformed_response_shape_raises(self) -> None:
        session = _FakeSession({"unexpected": "shape"})
        backend = self._backend(session)
        with self.assertRaises(RuntimeError):
            backend.query(_tmp_image(), "prompt A")


class GeminiBackendTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.cache_dir = Path(self.tmp.name) / "cache"
        self.log_path = Path(self.tmp.name) / "logs" / "vlm_calls.jsonl"

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_refuses_to_construct_without_api_key(self) -> None:
        import os

        env_backup = os.environ.pop("GEMINI_API_KEY", None)
        try:
            with self.assertRaises(RuntimeError):
                GeminiBackend(cache_dir=self.cache_dir, log_path=self.log_path)
        finally:
            if env_backup is not None:
                os.environ["GEMINI_API_KEY"] = env_backup

    def test_api_key_never_appears_in_request_url(self) -> None:
        import os

        os.environ["GEMINI_API_KEY"] = "test-fake-key-do-not-log"
        try:
            session = _FakeSession(
                {"candidates": [{"content": {"parts": [{"text": '{"ok": true}'}]}}]}
            )
            backend = GeminiBackend(cache_dir=self.cache_dir, log_path=self.log_path, session=session)
            result = backend.query(_tmp_image(), "prompt A")
            self.assertFalse(result.cache_hit)
            self.assertNotIn("test-fake-key-do-not-log", session.last_url)
            self.assertEqual(session.last_headers["x-goog-api-key"], "test-fake-key-do-not-log")
        finally:
            del os.environ["GEMINI_API_KEY"]

    def test_call_budget_is_enforced(self) -> None:
        import os

        os.environ["GEMINI_API_KEY"] = "test-fake-key"
        try:
            session = _FakeSession(
                {"candidates": [{"content": {"parts": [{"text": '{"ok": true}'}]}}]}
            )
            backend = GeminiBackend(
                cache_dir=self.cache_dir, log_path=self.log_path, session=session, call_cap=2
            )
            backend.query(_tmp_image(), "prompt A")
            backend.query(_tmp_image(), "prompt B")
            with self.assertRaises(CallBudgetExceeded):
                backend.query(_tmp_image(), "prompt C")
            self.assertEqual(session.post_count, 2)
        finally:
            del os.environ["GEMINI_API_KEY"]


if __name__ == "__main__":
    unittest.main()
