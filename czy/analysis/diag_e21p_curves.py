# -*- coding: utf-8 -*-
"""exp2.1p (TASK_20260924_011) 训练曲线健康检查：reward / jac_proxy / noise_std 对照 exp2.0 同期"""
import json
import urllib.request

BASE = 'https://internal.limxdynamics.com/dev-api/api'
TASK = 'TASK_20260924_011'


def get_key(k):
    url = f'{BASE}/task/data/info'
    payload = json.dumps({'task_id': TASK, 'data_key': k,
                          'sampling_mode': 'precise'}).encode()
    req = urllib.request.Request(url, method='POST', data=payload,
                                 headers={'Content-Type': 'application/json'})
    # 复用 flux 已保存的 keychain？CLI 更稳，改用 subprocess
    raise RuntimeError('unused')


if __name__ == '__main__':
    import subprocess
    for key in ['Train/mean_reward', 'Policy/jac_proxy', 'Policy/mean_noise_std']:
        r = subprocess.run(['flux.cmd', 'task', 'data', 'get', '--task-id', TASK,
                            '--data-key', key, '--sampling-mode', 'precise',
                            '--max-data-points', '10000'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace')
        d = json.loads(r.stdout)
        seg = d['data'][key]
        steps, vals = seg['steps'], seg['values']
        print(f'\n[{key}] {len(steps)} pts, iter {steps[0]}~{steps[-1]}')
        marks = [1, 614, 1398, 1500, 2100, 2903, 3569]
        for mk in marks:
            if mk < steps[0] or mk > steps[-1]:
                continue
            idx = min(range(len(steps)), key=lambda i: abs(steps[i] - mk))
            print(f'  iter {steps[idx]:>5}: {vals[idx]:.3f}')
