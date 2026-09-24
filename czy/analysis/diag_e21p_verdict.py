# -*- coding: utf-8 -*-
"""exp2.1p 裁决：与 exp2.0 逐项对照（同回放口径）"""
import numpy as np
import pandas as pd

FS = 100.0
KP = 28.0
KD = {'ankle_roll': 1.5, 'ankle_pitch': 1.2}


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return x - np.convolve(x, k, mode='same')


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)))


df = pd.read_csv('czy/data/exp2.1p/isaac_diag.csv', encoding='utf-8-sig')
m = np.abs(df['cmd_linear_x'].values) > 0.05

print('=== P/D 项拆解（KP=28 实际值）===')
for j in ['ankle_roll', 'ankle_pitch']:
    kd = KD[j]
    P, D, T = [], [], []
    for s in ['left', 'right']:
        pos = df[f'pos_{s}_{j}_joint'].values[m]
        des = df[f'pos_des_{s}_{j}_joint'].values[m]
        vel = df[f'vel_{s}_{j}_joint'].values[m]
        tau = df[f'effort_{s}_{j}_joint'].values[m]
        P.append((np.abs(KP * (des - pos)).mean(), rms(hp(KP * (des - pos)))))
        D.append((np.abs(-kd * vel).mean(), rms(hp(-kd * vel))))
        T.append((np.abs(tau).mean(), rms(hp(tau))))
    f = lambda x: (np.mean([v[0] for v in x]), np.mean([v[1] for v in x]))
    pt, pf = f(P); dt, dfq = f(D); tt, tf = f(T)
    print(f'{j:>12}: tau高频={tf:5.2f}  P高频={pf:5.2f}  D高频={dfq:4.2f}  '
          f'|tau|={tt:5.2f}  P|.|={pt:5.2f}  D|.|={dt:4.2f}')

print()
print('=== 与 exp2.0 对照（同一 CSV 口径）===')
d0 = pd.read_csv('czy/data/exp2.0/isaac_diag.csv', encoding='utf-8-sig')
m0 = np.abs(d0['cmd_linear_x'].values) > 0.05
for j in ['ankle_roll', 'ankle_pitch']:
    for tag, d_, mm in [('exp2.0', d0, m0), ('exp2.1p', df, m)]:
        hf = np.mean([rms(hp(d_[f'effort_{s}_{j}_joint'].values[mm])) for s in ['left', 'right']])
        av = np.mean([np.std(np.diff(d_[f'vel_{s}_{j}_joint'].values[mm])) * FS for s in ['left', 'right']])
        print(f'{j:>12} {tag:>8}: tau高频={hf:5.2f} Nm   vel抖={av:6.1f}')
