# -*- coding: utf-8 -*-
"""exp2.2 两臂 vs 已知基线：训练曲线同期对比（reward / jac_proxy / noise_std）
用法: python czy/analysis/diag_e22_curves.py
"""
import json
import subprocess

TASKS = [('exp2.0 (LCP1e-5,KP35)', 'TASK_20260922_044'),
         ('exp2.1p (LCP3e-5,KP28)', 'TASK_20260924_011'),
         ('exp2_2a (LCP3e-5)', 'TASK_20260928_140'),
         ('exp2_2b (KP28)', 'TASK_20260928_141')]
KEYS = ['Train/mean_reward', 'Policy/jac_proxy', 'Policy/mean_noise_std']
MARKS = [300, 614, 1000, 1398, 1715, 2100, 2903, 3569]


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


prefix = {'Train/mean_reward': 'reward', 'Policy/jac_proxy': 'jac_proxy',
          'Policy/mean_noise_std': 'noise_std'}

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
