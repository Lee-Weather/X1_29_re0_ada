# -*- coding: utf-8 -*-
"""exp2.3 两臂 vs 已知基线：训练曲线同期对比（reward / jac_proxy / noise_std）
用法: python czy/analysis/diag_e23_curves.py
注意：旧任务（exp2.0 / exp2.1p / exp2_2a / exp2_2b）属其它 GM 账号，当前账号 4385 可能读不到，
      读不到时该行显示「无数据」，此时请对照 exp2.md 中已记录的历史数值。
"""
import json
import subprocess

TASKS = [('exp2.0 (LCP1e-5,KP35)', 'TASK_20260922_044'),
         ('exp2_2b (KP28)', 'TASK_20260928_141'),
         ('exp2_3a (复跑 exp2.0)', 'TASK_20261008_148'),
         ('exp2_3b (smooth-0.05)', 'TASK_20261008_149')]
KEYS = ['Train/mean_reward', 'Policy/jac_proxy']
MARKS = [300, 614, 1000, 1398, 1715, 2100, 2903, 3569, 4473, 5500, 5999]


def fetch(task, key):
    for mode in ('precise', 'accelerate'):
        r = subprocess.run(['flux.cmd', 'task', 'data', 'get', '--task-id', task,
                            '--data-key', key, '--sampling-mode', mode,
                            '--max-data-points', '10000'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace')
        try:
            d = json.loads(r.stdout)
            seg = d['data'][key]
            if len(seg['steps']) > 1:
                return seg['steps'], seg['values']
        except Exception:
            pass
    return [], []


for key in KEYS:
    print(f'\n===== {key} =====')
    print(f'{"任务":>24} {"pts":>5} ' + ''.join(f'{m:>9}' for m in MARKS))
    print('-' * (30 + 9 * len(MARKS)))
    for tag, task in TASKS:
        steps, vals = fetch(task, key)
        if not steps:
            print(f'{tag:>24} {"无数据":>5}')
            continue
        cells = []
        for mk in MARKS:
            if steps[0] <= mk <= steps[-1]:
                i = min(range(len(steps)), key=lambda x: abs(steps[x] - mk))
                cells.append(f'{vals[i]:>9.2f}')
            else:
                cells.append(f'{"-":>9}')
        print(f'{tag:>24} {len(steps):>5} ' + ''.join(cells))
