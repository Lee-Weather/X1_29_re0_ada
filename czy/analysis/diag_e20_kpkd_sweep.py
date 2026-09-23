# -*- coding: utf-8 -*-
"""KP/KD 细扫（可用区间）+ 峰值力矩余量检查
tau(KP,KD) = KP*(pos_des - pos) - KD*vel   一阶估计
重点看：颤振 RMS 是否下降、以及 |tau| 的 P99 是否仍在力矩限幅内。
"""
import numpy as np
import pandas as pd

FS = 100.0
P = 'czy/data/exp2.0/isaac_diag.csv'
KP_GRID = [35, 32, 30, 28, 25]
KD_GRID = [0.8, 1.0, 1.2, 1.5]


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return x - np.convolve(x, k, mode='same')


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)))


df = pd.read_csv(P, encoding='utf-8-sig')
m = np.abs(df['cmd_linear_x'].values) > 0.05
print('数据源 exp2.0；tau 限幅：ankle effort limit = 80 Nm（URDF），'
      '但 cfg.safety.torque_limit=0.85 作用于整体\n')

for j in ['ankle_roll', 'ankle_pitch']:
    seg = {s: (df[f'pos_des_{s}_{j}_joint'].values[m],
               df[f'pos_{s}_{j}_joint'].values[m],
               df[f'vel_{s}_{j}_joint'].values[m]) for s in ['left', 'right']}
    # 基线统计
    base = {}
    for s in ['left', 'right']:
        des, pos, vel = seg[s]
        tau = 35 * (des - pos) - 1.5 * vel
        base[s] = tau
    err = np.mean([np.mean(np.abs(seg[s][0] - seg[s][1])) for s in ['left', 'right']])
    pk = np.mean([np.percentile(np.abs(base[s]), 99) for s in ['left', 'right']])
    print(f'[{j}] 基线(KP35/KD1.5): 平均|误差|={err:.3f} rad ({np.degrees(err):.1f}°), '
          f'|tau|P99={pk:.1f} Nm, 高频颤振={np.mean([rms(hp(base[s])) for s in ["left","right"]]):.2f} Nm')
    print(f'   KP\\KD ' + ''.join(f'{k:>14.1f}' for k in KD_GRID))
    for kp in KP_GRID:
        cells = []
        for kd in KD_GRID:
            hs, ps, es = [], [], []
            for s in ['left', 'right']:
                des, pos, vel = seg[s]
                tau = kp * (des - pos) - kd * vel
                hs.append(rms(hp(tau)))
                ps.append(np.percentile(np.abs(tau), 99))
                es.append(np.mean(np.abs(tau)))
            cells.append(f'{np.mean(hs):>5.2f}/{np.mean(es):>4.1f}')
        print(f'   {kp:>5} ' + ''.join(f'{c:>14}' for c in cells))
    print('   格内 = 颤振RMS / |tau|均值 (Nm)\n')
