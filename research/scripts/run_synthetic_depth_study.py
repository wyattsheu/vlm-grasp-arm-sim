#!/usr/bin/env python3
"""Controlled statistic ablation: same windows/validity gate for mean and median.

Synthetic measurement experiment; not camera calibration or grasp evaluation.
The mean here is NOT the full legacy 11x11 pipeline reproduction.
"""
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from mpg.lifting import depth_window_median


def main():
    config=json.loads((ROOT/'configs/synthetic_depth_study.json').read_text())
    rng=np.random.default_rng(config['seed'])
    k=config['window_k']
    rows=[]
    samples=[]
    for bg in config['background_fractions']:
        for missing in config['missing_fractions']:
            errors={name:[] for name in ('mean','median')}
            for repeat in range(config['repeats']):
                window=np.full((k,k),config['foreground_depth_m'])
                window[rng.random((k,k))<bg]=config['background_depth_m']
                # Query ray is on foreground; neighboring pixels may be background.
                window[k//2,k//2]=config['foreground_depth_m']
                window+=rng.normal(0,config['gaussian_sigma_m'],(k,k))
                window[rng.random((k,k))<missing]=0
                sample=depth_window_median(window,k//2,k//2,window_k=k)
                accepted=sample.valid_fraction>=config['valid_fraction_min']
                estimates={}
                if accepted:
                    estimates={'mean':float(window[window>0].mean()),'median':sample.depth_m}
                    for name,estimate in estimates.items():
                        errors[name].append(abs(estimate-config['foreground_depth_m']))
                samples.append(dict(background_fraction=bg,missing_fraction=missing,repeat=repeat,
                    valid_fraction=sample.valid_fraction,accepted=accepted,estimates_m=estimates))
            row=dict(background_fraction=bg,missing_fraction=missing,total=config['repeats'],accepted=len(errors['mean']))
            for name,values in errors.items():
                row[name+'_mae_mm']=float(np.mean(values)*1000) if values else None
                row[name+'_p90_abs_error_mm']=float(np.percentile(values,90)*1000) if values else None
            rows.append(row)
    result=dict(scope='SYNTHETIC_DEPTH_ONLY',config=config,windows=len(samples),
        independent_camera_observations=0,rows=rows,samples=samples)
    (ROOT/'logs/synthetic_depth_study_20260912.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# 合成深度視窗實驗 — 2026-09-12','',
        '此實驗只比較相同 7×7 視窗、相同有效比例 gate 的 mean/median。不是實機測試，也不是完整 legacy 11×11 mean 管線重現。',
        '',f"固定 seed={config['seed']}；共 {len(samples)} 個合成視窗。真前景 1.0m、背景 1.3m、獨立高斯 noise σ=3mm；每條件100次。中心 ray 設為前景，鄰域按配置機率混入背景；缺值獨立抽樣。",'',
        '| 背景混入機率 | 缺值機率 | 接受／總數 | mean MAE mm | median MAE mm |',
        '|---|---|---|---|---|']
    for row in rows:
        def number(x): return 'N/A' if x is None else f'{x:.3f}'
        lines.append(f"| {row['background_fraction']} | {row['missing_fraction']} | {row['accepted']}/{row['total']} | {number(row['mean_mae_mm'])} | {number(row['median_mae_mm'])} |")
    lines+=['','## 可支持的結論','',
        '在此人工深度分布下，少量背景混入時 median 比 mean 更能保持前景深度；背景占多數時 median 可能直接選到背景層。高有效比例不能保證選到正確物體。這支持下一步測 object-mask / depth-cluster sampling，而非繼續只調大 median 視窗。',
        '', '缺值 gate 會拒絕樣本；表中 MAE 僅以接受者為分母，接受率同時保留。全部拒絕時誤差為 N/A，不能寫0。',
        '', '## 限制與重現','',
        '背景／缺值是獨立抽樣，未模擬 RealSense 的空間相關誤差、反光、邊界飛點或畸變。沒有外參、物體幾何或機器人成功率。結論不能外推成真實 D435 的毫米精度。',
        '', '`PYTHONDONTWRITEBYTECODE=1 python3 scripts/run_synthetic_depth_study.py`',
        '', '參數：`configs/synthetic_depth_study.json`；逐樣本估計與彙總：`logs/synthetic_depth_study_20260912.json`。',
        '', '方法關聯：ZeroDex 的 RGB-D 限制是背景動機；此噪聲分布、mean/median 消融與數值結果由本專案自行設計，未冒充原論文實驗。[ZeroDex](https://arxiv.org/abs/2606.19340)']
    (ROOT/'docs/synthetic_depth_results_20260912.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(rows,indent=2))


if __name__=='__main__':
    main()
