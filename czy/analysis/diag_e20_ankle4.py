# -*- coding: utf-8 -*-
"""踝关节：总抖动 vs 高频抖动（颤振）—— 区分步态固有变化与真正的高频颤振"""
import numpy as np
import pandas as pd

CAND = [('1.6', 'czy/data/exp_ada_1.6/isaac_diag.csv'),
        ('1.7', 'czy/data/exp_ada_1.7/isaac_diag.csv'),
        ('1.8', 'czy/data/exp_ada_1.8/isaac_diag.csv'),
        ('1.9', 'czy/data/exp_ada_1.9/isaac_diag.csv'),
        ('1.10', 'czy/data/exp_ada_1.10/isaac_diag.csv'),
        ('1.11', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('1.11l', 'czy/data/exp_ada_1.11l/isaac_diag.csv'),
        ('exp2.0', 'czy/data/exp2.0/isaac_diag.csv'),
        ('exp2.1', 'czy/data/exp2.1/isaac_diag.csv'),
        ('exp2.1p', 'czy/data/exp2.1p/isaac_diag.csv')]
FS = 100.0


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return np.std(x - np.convolve(x, k, mode='same'))


print('踝关节力矩：总抖 vs 高频抖（颤振分量）')
print(f'{"版本":>8} {"roll|T|Nm":>10} {"roll总抖":>9} {"roll高频抖":>11} {"颤振/均值":>9} '
      f'{"pitch|T|":>9} {"pitch高频抖":>12} {"颤振/均值":>9}')
print('-' * 94)
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    r = []
    for j in ['ankle_roll', 'ankle_pitch']:
        t = np.mean([np.abs(df[f'effort_{s}_{j}_joint'].values[m]).mean() for s in ['left', 'right']])
        tot = np.mean([np.std(np.diff(df[f'effort_{s}_{j}_joint'].values[m])) * FS for s in ['left', 'right']])
        hf = np.mean([hp(df[f'effort_{s}_{j}_joint'].values[m]) for s in ['left', 'right']])
        r.append((t, tot, hf, hf / max(t, 1e-9) * 100))
    print(f'{tag:>8} {r[0][0]:>10.2f} {r[0][1]:>9.0f} {r[0][2]:>11.2f} {r[0][3]:>8.1f}% '
          f'{r[1][0]:>9.2f} {r[1][2]:>12.2f} {r[1][3]:>8.1f}%')

print()
print('踝关节速度：总抖 vs 高频抖')
print(f'{"版本":>8} {"roll总抖":>9} {"roll高频抖":>11} {"高频/总":>8} '
      f'{"pitch总抖":>10} {"pitch高频抖":>12} {"高频/总":>8}')
print('-' * 94)
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    o = []
    for j in ['ankle_roll', 'ankle_pitch']:
        tot = np.mean([np.std(np.diff(df[f'vel_{s}_{j}_joint'].values[m])) * FS for s in ['left', 'right']])
        hf = np.mean([hp(df[f'vel_{s}_{j}_joint'].values[m]) for s in ['left', 'right']])
        o.append((tot, hf, hf / max(tot, 1e-9) * 100))
    print(f'{tag:>8} {o[0][0]:>9.0f} {o[0][1]:>11.2f} {o[0][2]:>7.1f}% '
          f'{o[1][0]:>10.0f} {o[1][1]:>12.2f} {o[1][2]:>7.1f}%')
