import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from PIL import Image

from test_grounding import _stage_a_json, _stage_b_json
from mpg.candidates import propose_grid, parse_candidates
from mpg.schema import StageAPlan, ParseError

ROOT=Path(__file__).resolve().parents[1]


class OfflineRunnerTest(unittest.TestCase):
    def test_success_failure_and_missing_run_denominator(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            root=Path(tmp)
            Image.new('RGB',(100,80)).save(root/'image.png')
            (root/'responses.json').write_text(json.dumps([_stage_a_json(),_stage_b_json()]))
            common=[sys.executable,str(ROOT/'scripts/run_grounding.py'),'--image',str(root/'image.png'),
                    '--instruction','put cup in bowl','--scene-id','synthetic','--replicate-id','r0',
                    '--replay',str(root/'responses.json')]
            result=subprocess.run(common+['--output',str(root/'success')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            metadata=json.loads((root/'success/result.json').read_text())
            self.assertTrue(metadata['replay'])
            self.assertEqual(metadata['actual_calls'],0)
            self.assertTrue((root/'success/overlay.png').exists())
            (root/'responses.json').write_text(json.dumps(['bad json','bad json']))
            result=subprocess.run(common+['--output',str(root/'failed')],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertTrue((root/'failed/response_001.txt').exists())
            labels={'target':{'bbox_xyxy':[0,0,99,79]},'destination':{'bbox_xyxy':[0,0,99,79]}}
            (root/'labels.json').write_text(json.dumps(labels))
            entries=[dict(run_dir=run,labels='labels.json',group='test',scene_id='synthetic')
                     for run in ('success','failed','missing')]
            (root/'manifest.json').write_text(json.dumps(entries))
            result=subprocess.run([sys.executable,str(ROOT/'scripts/run_eval.py'),'--manifest',str(root/'manifest.json'),
                                   '--output',str(root/'eval.json')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            summary=json.loads((root/'eval.json').read_text())['groups']['test']
            self.assertEqual(summary['target_bbox_hit']['annotated_denominator'],3)
            self.assertEqual(summary['target_bbox_hit']['successes'],1)
            self.assertEqual(summary['missing'],1)

    def test_candidates_preserve_coordinates_and_reject_invented_id(self):
        plan=StageAPlan.from_json_text(json.dumps(_stage_a_json()))
        grid=propose_grid(100,80,rows=3,columns=4,seed=129)
        rows=[dict(step_id=s.step_id,candidate_id=grid[i]['id'],point_status='localized',reason_codes=[])
              for i,s in enumerate(plan.steps)]
        steps=parse_candidates(json.dumps(rows),plan,grid)
        self.assertEqual(list(steps[0].point_yx_norm1000),grid[0]['point_yx_norm1000'])
        rows[0]['candidate_id']='C999'
        with self.assertRaises(ParseError):
            parse_candidates(json.dumps(rows),plan,grid)

    def test_candidate_runner_with_fixed_stage_a(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
            root=Path(tmp)
            Image.new('RGB',(100,80)).save(root/'image.png')
            plan=_stage_a_json()
            (root/'stage_a.json').write_text(json.dumps(plan))
            (root/'input.json').write_text(json.dumps(dict(image_sha256=hashlib.sha256((root/'image.png').read_bytes()).hexdigest(),
                                                         instruction='test',scene_id='synthetic')))
            grid=propose_grid(100,80,rows=6,columns=8,seed=129)
            response=[dict(step_id=s['step_id'],candidate_id=grid[i]['id'],point_status='localized',reason_codes=[])
                      for i,s in enumerate(plan['steps'])]
            (root/'responses.json').write_text(json.dumps([response]))
            result=subprocess.run([sys.executable,str(ROOT/'scripts/run_grounding.py'),
                '--image',str(root/'image.png'),'--instruction','test','--scene-id','synthetic',
                '--replicate-id','r0','--mode','candidates','--stage-a',str(root/'stage_a.json'),
                '--replay',str(root/'responses.json'),'--output',str(root/'run')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertTrue((root/'run/candidates.png').exists())
