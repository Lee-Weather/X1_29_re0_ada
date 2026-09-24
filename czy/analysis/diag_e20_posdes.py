# -*- coding: utf-8 -*-
"""踝关节：指令抖(pos_des) vs 实际抖(pos) 的高频成分
判断 P 项颤振来自"策略指令跳变"还是"机械欠阻尼振荡"。
同时估算 PD 闭环的自然频率与阻尼比量级。
"""
import numpy as np
import pandas as pd

CAND = [('1.8', 'czy/data/exp_ada_1.8/isaac_diag.csv'),
        ('1.11', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('exp2.0', 'czy/data/exp2.0/isaac_diag.csv'),
        ('exp2.1', 'czy/data/exp2.1/isaac_diag.csv')]
FS = 100.0
KP = 35.0
KD = 1.5


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return x - np.convolve(x, k, mode='same')


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)))


print('踝关节位置：指令(pos_des) vs 实际(pos) 的高频抖')
print(f'{"版本":>8} {"关节":>12} {"des总抖":>9} {"pos总抖":>9} {"des高频":>9} {"pos高频":>9} '
      f'{"des占cmd%":>10} {"pos占cmd%":>10}')
print('-' * 100)
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for j in ['ankle_roll', 'ankle_pitch']:
        rows = []
        for s in ['left', 'right']:
            pos = df[f'pos_{s}_{j}_joint'].values[m]
            des = df[f'pos_des_{s}_{j}_joint'].values[m]
            act = df[f'action_{s}_{j}_joint'].values[m]
            rows.append((np.std(des), np.std(pos), rms(hp(des)), rms(hp(pos)),
                         np.std(act), (rms(hp(des)) / max(np.std(act), 1e-9)) * 100))
        a, b, c, d, e, f = [np.mean([r[i] for r in rows]) for i in range(6)]
        print(f'{tag:>8} {j:>12} {a:>9.4f} {b:>9.4f} {c:>9.4f} {d:>9.4f} {e:>10.3f} {f:>10.1f}')

print()
print('踝关节 action 高频抖（策略输出，未经 PD）')
print(f'{"版本":>8} {"关节":>12} {"act总抖":>9} {"act高频":>9}')
print('-' * 50)
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for j in ['ankle_roll', 'ankle_pitch']:
        rows = []
        for s in ['left', 'right']:
            act = df[f'action_{s}_{j}_joint'].values[m]
            rows.append((np.std(act), rms(hp(act))))
        a, b = [np.mean([r[i] for r in rows]) for i in range(2)]
        print(f'{tag:>8} {j:>12} {a:>9.4f} {b:>9.4f}')
