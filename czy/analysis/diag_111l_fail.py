# -*- coding: utf-8 -*-
"""1.11l 训练崩塌的机理分析：策略到底变成了什么？为什么？
对比 1.11（正常）vs 1.11l（LCP 致瘫）
"""
import numpy as np
import pandas as pd

FS = 50.0
DT = 0.02
A = pd.read_csv('czy/data/exp_ada_1.11/isaac_diag.csv', encoding='utf-8-sig')
B = pd.read_csv('czy/data/exp_ada_1.11l/isaac_diag.csv', encoding='utf-8-sig')

JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]

print('=' * 92)
print('一、动作输出特性（actor 输出 μ 的"是否随观测变化"）')
print(f'{"":>22} {"action std(全体)":>17} {"action 时间差分std":>18} {"pos_des 摆幅中位":>16}')
for tag, df in [('1.11 (正常)', A), ('1.11l (LCP)', B)]:
    acts = np.stack([df[f'action_{j}_joint'].values for j in JOINTS], axis=1)
    diffs = np.stack([np.diff(df[f'action_{j}_joint'].values) for j in JOINTS], axis=1)
    des = np.stack([df[f'pos_des_{j}_joint'].values for j in JOINTS], axis=1)
    print(f'{tag:>22} {acts.std():>17.4f} {diffs.std():>18.4f} {np.median(des.max(0) - des.min(0)):>16.4f}')

print()
print('=' * 92)
print('二、关节实际运动幅度（dof_pos 峰峰值，判断"是否在动"）')
print(f'{"关节":>18} {"1.11 幅度°":>12} {"1.11l 幅度°":>13} {"比值":>8}')
for j in JOINTS:
    ra = np.ptp(A[f'pos_{j}_joint'].values) * 180 / np.pi
    rb = np.ptp(B[f'pos_{j}_joint'].values) * 180 / np.pi
    print(f'{j:>18} {ra:>12.1f} {rb:>13.1f} {rb/max(ra,1e-9)*100:>7.0f}%')

print()
print('=' * 92)
print('三、基座与足底（是否行走 / 是否着地）')
for tag, df in [('1.11 (正常)', A), ('1.11l (LCP)', B)]:
    print(f'  {tag}:')
    print(f'    base_height  mean={df["base_height"].mean():.3f} std={df["base_height"].std():.4f}')
    print(f'    |base_vel_x| mean={np.abs(df["base_vel_x"].values).mean():.4f} m/s')
    print(f'    |base_vel_y| mean={np.abs(df["base_vel_y"].values).mean():.4f} m/s')
    print(f'    foot_force L mean={df["foot_force_l"].mean():.1f} R mean={df["foot_force_r"].mean():.1f}')
    print(f'    foot_z  L range=[{df["foot_z_l"].min()*1000:.0f},{df["foot_z_l"].max()*1000:.0f}]mm  '
          f'R range=[{df["foot_z_r"].min()*1000:.0f},{df["foot_z_r"].max()*1000:.0f}]mm')

print()
print('=' * 92)
print('四、相位驱动 vs 策略驱动：动作与相位的相关性（判断是否"跟随参考轨迹踏步"）')
for tag, df in [('1.11 (正常)', A), ('1.11l (LCP)', B)]:
    ph = df['phase_sin'].values
    cors = []
    for j in ['left_knee_pitch', 'right_knee_pitch', 'left_hip_pitch']:
        a = df[f'action_{j}_joint'].values
        cors.append(np.corrcoef(ph, a)[0, 1])
    print(f'  {tag}: corr(phase_sin, action) knee_L={cors[0]:+.2f} knee_R={cors[1]:+.2f} hip_L={cors[2]:+.2f}')

print()
print('=' * 92)
print('五、pos_des（PD 目标）是否被跟踪 —— 跟踪误差')
for tag, df in [('1.11 (正常)', A), ('1.11l (LCP)', B)]:
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    errs = [np.sqrt((df[f'pos_track_err_{j}_joint'].values[m] ** 2).mean()) * 180 / np.pi for j in JOINTS]
    print(f'  {tag}: 全关节 track_err rms 中位={np.median(errs):.1f}°  max={max(errs):.1f}°')

print()
print('=' * 92)
print('六、LCP 指标分布（策略敏感度）')
for tag, df in [('1.11 (正常)', A), ('1.11l (LCP)', B)]:
    f = df['lcp_jac_frob'].values
    print(f'  {tag}: jac_frob median={np.median(f):.3f} p10={np.percentile(f,10):.3f} '
          f'p90={np.percentile(f,90):.3f}  → LCP loss≈{np.median(df["lcp_sigma_w"].values)**2:.0f}')
