"""Regressions for review R1-R6; only synthetic geometry and fake transport."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image

from test_grounding import _stage_a_json, _FakeBackend
from mpg.grounding import run_stage_a, GroundingFailure
from mpg.schema import StageAPlan, ParseError
from mpg.lifting import AxisAlignedBox, PlaneFit, LiftingError, refine_point, fit_table_plane_ransac, waypoint_corridor_height
from mpg.vlm.base import BaseBackend, CallBudgetExceeded

ROOT = Path(__file__).resolve().parents[1]


class Fake(BaseBackend):
    backend_name = "regression"
    model = "fake"

    def __init__(self, path, fail=False):
        self.calls = 0
        self.fail = fail
        super().__init__(cache_dir=path / "cache", log_path=path / "calls.jsonl", call_cap=1)

    def _call(self, image, prompt):
        self.calls += 1
        if self.fail:
            raise TimeoutError("fake")
        return json.dumps(_stage_a_json())


class ReviewRegressions(unittest.TestCase):
    def test_reduced_svd(self):
        real = np.linalg.svd
        shapes = []
        def spy(a, **kw):
            out = real(a, **kw)
            shapes.append(out[0].shape)
            return out
        rng = np.random.default_rng(1)
        pts = np.column_stack([rng.random((100, 2)), np.zeros(100)])
        with patch("numpy.linalg.svd", side_effect=spy):
            fit_table_plane_ransac(pts, distance_threshold_m=.001, max_iterations=10, min_inlier_fraction=.5, rng=rng)
        self.assertEqual(shapes, [(100, 3)])

    def test_refine_nearest_up_and_support(self):
        box = AxisAlignedBox(np.full(3, .01))
        plane = PlaneFit(np.array([0., 0., 1.]), 0., np.ones(1, bool))
        point, changed = refine_point(np.zeros(3), box, np.zeros((1, 3)),
                                     radius_m=.04, step_m=.02, support_plane=plane)
        np.testing.assert_allclose(point, [0, 0, .02])
        self.assertTrue(changed)
        with self.assertRaises(LiftingError):
            refine_point(np.zeros(3), box, np.zeros((1, 3)), radius_m=0, step_m=.02)
        with self.assertRaises(LiftingError):
            refine_point(np.zeros(3), box, np.empty((0, 3)), radius_m=.04, step_m=.02)

    def test_elevated_endpoints_project_to_table(self):
        plane = PlaneFit(np.array([0., 0., 1.]), .5, np.ones(1, bool))
        result = waypoint_corridor_height(np.array([[.5, 0, .7]]), plane,
            np.array([0, 0, .8]), np.array([1, 0, .9]), corridor_r_m=.01)
        self.assertAlmostEqual(result, .2)

    def test_semantic_failure_does_not_retry(self):
        invalid = _stage_a_json()
        invalid['destination'] = None
        backend = _FakeBackend([json.dumps(invalid)])
        with self.assertRaises(GroundingFailure):
            run_stage_a(backend, Image.new('RGB', (4, 4)), 'test')
        self.assertEqual(len(backend.calls), 1)

    def test_stage_a_roundtrip(self):
        plan = StageAPlan.from_json_text(json.dumps(_stage_a_json()))
        self.assertEqual(plan, StageAPlan.from_json_text(json.dumps(plan.to_json())))

    def test_failed_request_consumes_budget_after_reconstruction(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            path = Path(tmp)
            backend = Fake(path, fail=True)
            with self.assertRaises(TimeoutError):
                backend.query(Image.new('RGB', (4, 4)), 'test')
            with self.assertRaises(CallBudgetExceeded):
                Fake(path).query(Image.new('RGB', (4, 4)), 'test')
            events = [json.loads(line)['event'] for line in (path/'calls.jsonl').read_text().splitlines()]
            self.assertEqual(events, ['attempt', 'failed'])

    def test_shared_ledger_concurrency_and_cache_metrics(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            path = Path(tmp)
            backends = [Fake(path), Fake(path)]
            def query(b):
                return run_stage_a(b, Image.new('RGB', (4, 4)), 'test')[1]
            with ThreadPoolExecutor(max_workers=2) as pool:
                infos = list(pool.map(query, backends))
            self.assertEqual(sum(b.calls for b in backends), 1)
            self.assertEqual(sum(i.n_calls for i in infos), 1)
            self.assertEqual(sum(i.cache_hits for i in infos), 1)
