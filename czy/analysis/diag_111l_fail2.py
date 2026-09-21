# -*- coding: utf-8 -*-
"""1.11l 崩塌机理（续）：动作是否"时间恒定"？hip_yaw 大幅摆动何来？蹲姿？
"""
import numpy as np
import pandas as pd

FS = 50.0
A = pd.read_csv('czy/data/exp_ada_1.11/isaac_diag.csv', encoding='utf-8-sig')
B = pd.read_csv('czy/data/exp_ada_1.11l/isaac_diag.csv', encoding='utf-8-sig')

JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]

print('=' * 92)
print('一、逐关节：动作的时间变化 vs 动作的静态幅值（判定 μ 是否时间恒定）')
print(f'{"关节":>18} {"A:Δaction_std":>13} {"B:Δaction_std":>13} {"A:action_std":>12} {"B:action_std":>12} {"B/A Δ比":>9}')
for j in JOINTS:
    da = np.diff(A[f'action_{j}_joint'].values).std()
    db = np.diff(B[f'action_{j}_joint'].values).std()
    sa = A[f'action_{j}_joint'].values.std()
    sb = B[f'action_{j}_joint'].values.std()
    print(f'{j:>18} {da:>13.4f} {db:>13.4f} {sa:>12.3f} {sb:>12.3f} {db/max(da,1e-9)*100:>8.0f}%')

print()
print('=' * 92)
print('二、hip_yaw 之谜：动作恒定但关节摆动 46°？(pos vs pos_des)')
for tag, df in [('1.11', A), ('1.11l', B)]:
    for j in ['left_hip_yaw', 'right_hip_yaw']:
        p = df[f'pos_{j}_joint'].values * 180 / np.pi
        pd_ = df[f'pos_des_{j}_joint'].values * 180 / np.pi
        print(f'  {tag:>5} {j:>15}: pos [{p.min():>6.1f},{p.max():>6.1f}] std={p.std():>5.2f} | '
              f'des [{pd_.min():>6.1f},{pd_.max():>6.1f}] std={pd_.std():>5.2f} | '
              f'跟踪err rms={np.sqrt(((pd_-p)**2).mean()):>5.1f}°')

print()
print('=' * 92)
print('三、蹲姿与稳定性')
for tag, df in [('1.11', A), ('1.11l', B)]:
    h = df['base_height'].values
    print(f'  {tag:>5}: height mean={h.mean():.3f}m range=[{h.min():.3f},{h.max():.3f}]  '
          f'roll={df["base_euler_x"].values.std()*180/np.pi:.2f}° '
          f'pitch={df["base_euler_y"].values.std()*180/np.pi:.2f}° '
          f'|yaw_rate|={np.abs(df["base_ang_vel_z"].values).mean()*180/np.pi:.2f}°/s')
    # 膝/髋 pitch 静态姿态（蹲姿证据）
    kp = df['pos_left_knee_pitch_joint'].values.mean() * 180 / np.pi
    hp = df['pos_left_hip_pitch_joint'].values.mean() * 180 / np.pi
    print(f'        静止姿态: knee_pitch mean={kp:.1f}°  hip_pitch mean={hp:.1f}°')

print()
print('=' * 92)
print('四、动作-观测解耦的直接证据：action 变化与 phase 变化的相关性')
for tag, df in [('1.11', A), ('1.11l', B)]:
    dph = np.diff(df['phase_sin'].values)
    cors = []
    for j in ['left_knee_pitch', 'right_knee_pitch']:
        da = np.diff(df[f'action_{j}_joint'].values)
        cors.append(np.corrcoef(dph, da)[0, 1])
    print(f'  {tag:>5}: corr(Δphase, Δaction) knee_L={cors[0]:+.3f} knee_R={cors[1]:+.3f}')

print()
print('=' * 92)
print('五、足底接触时序（步态是否存在）')
for tag, df in [('1.11', A), ('1.11l', B)]:
    fl, fr = df['foot_force_l'].values, df['foot_force_r'].values
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    l_air = (fl[m] < 5).mean() * 100
    r_air = (fr[m] < 5).mean() * 100
    both = ((fl[m] < 5) & (fr[m] < 5)).mean() * 100
    print(f'  {tag:>5}: L离地={l_air:.0f}% R离地={r_air:.0f}% 双脚同时离地={both:.0f}%')
