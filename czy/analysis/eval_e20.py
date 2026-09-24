# -*- coding: utf-8 -*-
"""exp2.0 验收对比（LCP 三态）：
   1.11  = 无 LCP（对照）
   1.11l = LCP 无 warmup，w=1e-4（致瘫）
   exp2.0= LCP warmup=1500，w=1e-5（本次）
"""
import numpy as np
import pandas as pd

FS = 50.0
VERS = [('1.11 (无LCP)', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('1.11l (LCP无warmup)', 'czy/data/exp_ada_1.11l/isaac_diag.csv'),
        ('exp2.0 (LCP+warmup)', 'czy/data/exp2.0/isaac_diag.csv'),
        ('exp2.1 (LPF fc=10Hz)', 'czy/data/exp2.1/isaac_diag.csv')]


def swing_peaks(z, mask, min_len=5):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if j - i >= min_len:
                out.append(z[i:j].max())
            i = j
        else:
            i += 1
    return np.array(out)


def segs_of(df):
    cmd = df['cmd_linear_x'].values
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    return [(a, b, cmd[a]) for a, b in zip(edges[:-1], edges[1:]) if b - a > 100]


JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]

print('=' * 104)
print('一、核心验收指标')
print(f'{"版本":>22} {"track0.2/0.4/0.6":>18} {"yaw0.2/0.4/0.6°":>20} {"|yaw_rate|中位":>13} '
      f'{"抬脚R/L":>9} {"双脚离地":>9} {"Δaction":>9} {"jac_frob":>9}')
print('-' * 104)
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    cmd = df['cmd_linear_x'].values
    wz = df['base_ang_vel_z'].values
    m = np.abs(cmd) > 0.05
    trs, yaws = [], []
    for a, b, c in segs_of(df):
        if abs(c) <= 0.05:
            continue
        vx = df['base_vel_x'].values[a:b]
        trs.append(np.median(vx[int((b - a) * 0.4):]) / c * 100)
        y = df['base_yaw'].values[a:b]
        yaws.append((y[-1] - y[0]) * 180 / np.pi)
    tr3 = '/'.join(f'{x:.0f}' for x in trs[:3])
    yw3 = '/'.join(f'{x:+.1f}' for x in yaws[:3])
    yr = np.median(np.abs(wz[m])) * 180 / np.pi
    ph = df['phase_sin'].values
    lp = swing_peaks(df['foot_z_l'].values * 1000, (ph < -0.3) & m)
    rp = swing_peaks(df['foot_z_r'].values * 1000, (ph > 0.3) & m)
    lift = np.median(rp) / np.median(lp) * 100 if len(lp) and len(rp) else np.nan
    both_air = ((df['foot_force_l'].values[m] < 5) & (df['foot_force_r'].values[m] < 5)).mean() * 100
    da = np.mean([np.diff(df[f'action_{j}_joint'].values).std() for j in JOINTS])
    jf = np.median(df['lcp_jac_frob'].values)
    print(f'{tag:>22} {tr3:>18} {yw3:>20} {yr:>13.2f} {lift:>8.0f}% {both_air:>8.0f}% {da:>9.4f} {jf:>9.3f}')

print()
print('=' * 104)
print('二、步态健康度（是否行走）')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    print(f'  {tag:>22}: |vx|={np.abs(df["base_vel_x"].values[m]).mean():.3f} m/s  '
          f'height={df["base_height"].mean():.3f}m  '
          f'knee={df["pos_left_knee_pitch_joint"].values.mean()*180/np.pi:.1f}°  '
          f'hip={df["pos_left_hip_pitch_joint"].values.mean()*180/np.pi:.1f}°')

print()
print('=' * 104)
print('三、LCP 指标（平滑度，越小越平滑）')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    f = df['lcp_jac_frob'].values
    sw = df['lcp_sigma_w'].values
    print(f'  {tag:>22}: jac_frob 中位={np.median(f):.3f} (p10={np.percentile(f,10):.3f} '
          f'p90={np.percentile(f,90):.3f})  σ加权={np.median(sw):.1f}  短历史占比={np.median(df["lcp_frac_short"].values):.3f}')

print()
print('=' * 104)
print('四、关节抖动（运动段 vel 差分 std×FS）')
print(f'{"版本":>22} {"踝roll L/R":>16} {"膝 L/R":>16} {"髋pitch L/R":>16}')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    row = []
    for j in ['ankle_roll', 'knee_pitch', 'hip_pitch']:
        jl = np.std(np.diff(df[f'vel_left_{j}_joint'].values[m])) * FS
        jr = np.std(np.diff(df[f'vel_right_{j}_joint'].values[m])) * FS
        row.append(f'{jl:.1f}/{jr:.1f}')
    print(f'{tag:>22} {row[0]:>16} {row[1]:>16} {row[2]:>16}')
