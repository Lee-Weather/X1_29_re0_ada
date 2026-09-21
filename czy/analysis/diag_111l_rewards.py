# -*- coding: utf-8 -*-
"""对比 1.11 vs 1.11l 的奖励分解：LCP 让哪些目标崩塌、哪些被"占便宜" """
import json
import subprocess

KEYS = ['rew_tracking_lin_vel', 'rew_tracking_ang_vel', 'rew_ref_joint_pos', 'rew_base_height',
        'rew_orientation', 'rew_gait_symmetry', 'rew_feet_air_time', 'rew_feet_clearance',
        'rew_stand_still', 'rew_lat_vel', 'rew_feet_distance', 'rew_knee_distance',
        'rew_stance_hip_roll', 'rew_torques', 'rew_dof_acc', 'rew_action_smoothness']

TASKS = [('1.11', 'TASK_20260920_130'), ('1.11l', 'TASK_20260921_024')]
res = {}
for tag, tid in TASKS:
    res[tag] = {}
    for k in KEYS:
        key = f'Episode/{k}'
        r = subprocess.run(f'flux task data get --task-id {tid} --data-key {key} '
                           f'--sampling-mode accelerate --max-data-points 200',
                           capture_output=True, text=True, shell=True)
        try:
            d = json.loads(r.stdout)
            kv = d['data'][key]
            v = kv['values']
            tail = v[-10:]
            res[tag][k] = sum(tail) / len(tail)
        except Exception:
            res[tag][k] = None

print(f'{"奖励项":>26} {"1.11":>12} {"1.11l":>12} {"差值":>12} {"判定":>8}')
print('-' * 78)
for k in KEYS:
    a, b = res['1.11'][k], res['1.11l'][k]
    if a is None or b is None:
        print(f'{k:>26} {"n/a":>12} {"n/a":>12}')
        continue
    d = b - a
    flag = '❌崩' if d < -0.02 else ('✅升' if d > 0.02 else '—')
    print(f'{k:>26} {a:>12.4f} {b:>12.4f} {d:>+12.4f} {flag:>8}')
