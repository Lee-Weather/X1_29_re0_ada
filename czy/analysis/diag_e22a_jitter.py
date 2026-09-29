# -*- coding: utf-8 -*-
"""exp2.2a 视频抖动诊断：全关节抖动强度 + 频谱定位
对比 exp2.0（基线，画面干净）/ exp2.2a（用户反馈视频抖动严重）
"""
import numpy as np
import pandas as pd

FS = 100.0
JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]
SHORT = ['hip_p', 'hip_r', 'hip_y', 'knee', 'ank_p', 'ank_r'] * 2


def load(tag):
    p = f'czy/data/{tag}/isaac_diag.csv'
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    return df, m


print('=' * 108)
print('一、全关节抖动（运动段, vel 一阶差分 std × FS, 单位 rad/s²）')
print('=' * 108)
print(f'{"关节":>14} {"exp2.0":>9} {"exp2.2a":>9} {"倍数":>7}   {"加速度扭矩抖(Nm)":>15} {"exp2.0":>9} {"exp2.2a":>9} {"倍数":>7}')
print('-' * 108)
d0, m0 = load('exp2.0')
d2, m2 = load('exp2.2a')
rows = []
for j, js in zip(JOINTS, SHORT):
    v0 = np.mean([np.std(np.diff(d0[f'vel_{j}_joint'].values[m0])) * FS for _ in [0]])
    v2 = np.mean([np.std(np.diff(d2[f'vel_{j}_joint'].values[m2])) * FS for _ in [0]])
    t0 = np.sqrt(np.mean((d0[f'effort_{j}_joint'].values[m0]
                          - np.convolve(d0[f'effort_{j}_joint'].values[m0],
                                        np.ones(20) / 20, mode='same')) ** 2))
    t2 = np.sqrt(np.mean((d2[f'effort_{j}_joint'].values[m2]
                          - np.convolve(d2[f'effort_{j}_joint'].values[m2],
                                        np.ones(20) / 20, mode='same')) ** 2))
    rows.append((js, v0, v2, v2 / v0, t0, t2, t2 / max(t0, 1e-9)))
for js, v0, v2, r1, t0, t2, r2 in rows:
    flag = ' <<<' if r1 > 1.5 or r2 > 1.5 else ''
    print(f'{js:>14} {v0:>9.1f} {v2:>9.1f} {r1:>6.2f}x   {t0:>15.2f} {t2:>9.2f} {r2:>6.2f}x{flag}')

print()
print('=' * 108)
print('二、频谱定位：踝 pitch 力矩的高频成分（运动段，功率谱峰值频率）')
print('=' * 108)
for tag in ['exp2.0', 'exp2.2a', 'exp2.2b']:
    df, m = load(tag)
    sig = df['effort_left_ankle_pitch_joint'].values[m]
    sig = sig - sig.mean()
    n = len(sig)
    f = np.fft.rfftfreq(n, 1 / FS)
    P = np.abs(np.fft.rfft(sig * np.hanning(n))) ** 2
    # 只看 1~50 Hz（排除直流与极低频步态）
    sel = (f >= 1) & (f <= 50)
    top = np.argsort(P[sel])[-5:][::-1]
    peaks = ', '.join(f'{f[sel][i]:.1f}Hz' for i in top)
    # 高频能量占比（>5Hz）
    e_lo = P[(f > 0) & (f <= 5)].sum()
    e_hi = P[(f > 5) & (f <= 50)].sum()
    print(f'  {tag:>8}: 主峰 {peaks}   | >5Hz 能量占比 = {e_hi / (e_lo + e_hi) * 100:.1f}%')

print()
print('=' * 108)
print('三、时间域：踝 pitch 力矩的短时抖（0.5s 滑窗 RMS 的 P90）')
print('=' * 108)
for tag in ['exp2.0', 'exp2.2a']:
    df, m = load(tag)
    for j in ['ankle_pitch', 'ankle_roll']:
        s = df[f'effort_left_{j}_joint'].values[m]
        w = int(0.5 * FS)
        rms = np.array([np.std(s[i:i + w]) for i in range(0, len(s) - w, 5)])
        print(f'  {tag:>8} {j:>12}: 滑窗RMS 中位 {np.median(rms):5.2f}  P90 {np.percentile(rms,90):5.2f}  '
              f'最大 {rms.max():5.2f} Nm')
