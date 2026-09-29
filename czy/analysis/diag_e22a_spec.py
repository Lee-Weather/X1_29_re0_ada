# -*- coding: utf-8 -*-
"""exp2.2a 抖动溯源：9.1Hz 振荡落在哪个信号通道（策略输出 or 机械响应）"""
import numpy as np
import pandas as pd

FS = 100.0


def spec(x):
    x = x - x.mean()
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / FS)
    P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    lo = P[(f > 0) & (f <= 5)].sum()
    hi = P[(f > 5) & (f <= 50)].sum()
    sel = (f >= 1) & (f <= 50)
    top = np.argsort(P[sel])[-4:][::-1]
    return f[sel][top], hi / (lo + hi) * 100


CH = [('action', 'action_left_ankle_pitch_joint'), ('pos_des', 'pos_des_left_ankle_pitch_joint'),
      ('pos', 'pos_left_ankle_pitch_joint'), ('vel', 'vel_left_ankle_pitch_joint'),
      ('torque', 'effort_left_ankle_pitch_joint')]

print('踝 pitch 各信号通道频谱（左腿，运动段 cmd!=0）')
print(f'{"版本":>8} {"通道":>9} {"主峰 Hz":>26} {">5Hz能量":>9}')
print('-' * 58)
for tag in ['exp2.0', 'exp2.2a']:
    df = pd.read_csv(f'czy/data/{tag}/isaac_diag.csv', encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for ch, col in CH:
        fpk, hi = spec(df[col].values[m])
        pk = ', '.join(f'{v:.1f}' for v in fpk)
        print(f'{tag:>8} {ch:>9} {pk:>26} {hi:>8.1f}%')
    print()

print('对照：踝 roll 力矩 + 膝关节力矩的 >5Hz 能量占比')
print('-' * 58)
for tag in ['exp2.0', 'exp2.2a']:
    df = pd.read_csv(f'czy/data/{tag}/isaac_diag.csv', encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    for j in ['ankle_roll', 'knee_pitch', 'hip_pitch']:
        _, hi = spec(df[f'effort_left_{j}_joint'].values[m])
        print(f'{tag:>8} {j:>12}: >5Hz 能量 {hi:5.1f}%')
    print()
