# -*- coding: utf-8 -*-
"""踝关节力矩颤振的来源拆解：P 项 (KP·指令误差) vs D 项 (-KD·速度)
tau = KP*(pos_des - pos) - KD*vel  (legged_robot._compute_torques)
输出各分量的 >5Hz 高频 RMS，判断该调 KP 还是 KD。
"""
import numpy as np
import pandas as pd

CAND = [('1.11', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('exp2.0', 'czy/data/exp2.0/isaac_diag.csv')]
FS = 100.0
KP = 35.0   # ankle_pitch / ankle_roll
KD = 1.5


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return x - np.convolve(x, k, mode='same')


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)))


print(f'踝关节力矩拆解  KP={KP} KD={KD}  (高通用 {5.0}Hz 滑动均值)')
print(f'{"版本":>8} {"关节":>12} {"|tau|":>7} {"tau高频":>8} '
      f'{"P项|.|":>8} {"P项高频":>9} {"D项|.|":>8} {"D项高频":>9} {"主因":>6}')
print('-' * 92)
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for j in ['ankle_roll', 'ankle_pitch']:
        res = []
        for s in ['left', 'right']:
            pos = df[f'pos_{s}_{j}_joint'].values[m]
            des = df[f'pos_des_{s}_{j}_joint'].values[m]
            vel = df[f'vel_{s}_{j}_joint'].values[m]
            tau = df[f'effort_{s}_{j}_joint'].values[m]
            P = KP * (des - pos)
            D = -KD * vel
            res.append((np.abs(tau).mean(), rms(hp(tau)),
                        np.abs(P).mean(), rms(hp(P)),
                        np.abs(D).mean(), rms(hp(D))))
        a = np.mean([r[0] for r in res]); b = np.mean([r[1] for r in res])
        c = np.mean([r[2] for r in res]); d = np.mean([r[3] for r in res])
        e = np.mean([r[4] for r in res]); f = np.mean([r[5] for r in res])
        who = 'P(指令)' if d > f else 'D(速度)'
        print(f'{tag:>8} {j:>12} {a:>7.2f} {b:>8.2f} {c:>8.2f} {d:>9.2f} {e:>8.2f} {f:>9.2f} {who:>6}')

print()
print('校验：P+D 与实测 tau 的相关系数 / 残差（确认公式与列语义）')
for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for s in ['left', 'right']:
        pos = df[f'pos_{s}_ankle_roll_joint'].values[m]
        des = df[f'pos_des_{s}_ankle_roll_joint'].values[m]
        vel = df[f'vel_{s}_ankle_roll_joint'].values[m]
        tau = df[f'effort_{s}_ankle_roll_joint'].values[m]
        pred = KP * (des - pos) - KD * vel
        cc = np.corrcoef(pred, tau)[0, 1]
        print(f'{tag:>8} {s:>6} corr={cc:.4f}  resid_rms={rms(pred - tau):.3f}  tau_rms={rms(tau):.3f}')
