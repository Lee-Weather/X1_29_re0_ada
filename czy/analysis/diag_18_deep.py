# -*- coding: utf-8 -*-
"""exp_ada_1.8 深度诊断：抬脚 R/L=113% 反向不对称根因 + 0.2 档速度 + 推进停滞 + 0.6 档 yaw"""
import io
import sys
import numpy as np
import pandas as pd

sys.stdout = io.open('_diag18_out.txt', 'w', encoding='utf-8')

R2D = 180 / np.pi
FF = 5.0
CSV = 'czy/data/exp_ada_1.8/isaac_diag.csv'
df = pd.read_csv(CSV, encoding='utf-8-sig')
print(f"rows={len(df)}")

ph = df['phase_sin'].values
fl, fr = df['foot_force_l'].values, df['foot_force_r'].values
vx, vy, yaw = df['base_vel_x'].values, df['base_vel_y'].values, df['base_yaw'].values
cmd = df['cmd_linear_x'].values

# 分段边界
bounds = []
cur, start = cmd[0], 0
for i in range(1, len(df) + 1):
    if i == len(df) or cmd[i] != cur:
        bounds.append((cur, start, i))
        if i < len(df):
            cur, start = cmd[i], i

# 相位标定
l_sw = (fl < FF)[ph > 0.3].mean()
r_sw = (fr < FF)[ph > 0.3].mean()
psign = 1.0 if r_sw > l_sw else -1.0  # sin>0 => R swing
swL = ph < -0.3
swR = ph > 0.3
stL, stR = ph > 0.3, ph < -0.3

print(f"\n========== 1. 抬脚 R/L=113% 反向不对称：逐段分解 ==========")
# 每段 L/R 抬脚 + 关节执行分解
for cmdv, a, b in bounds:
    if abs(cmdv) < 0.05:
        continue
    n40 = int((b - a) * 0.4)
    a2 = a + n40
    tag = f"seg{cmdv:+.1f}"
    out = [f"  {tag}:"]
    for s, sw in [('L', swL), ('R', swR)]:
        col = 'foot_z_l' if s == 'L' else 'foot_z_r'
        ffc = 'foot_force_l' if s == 'L' else 'foot_force_r'
        fz = df[col].values[a2:b]
        ff = df[ffc].values[a2:b]
        phs = ph[a2:b]
        swm = (phs < -0.3) if s == 'L' else (phs > 0.3)
        # 峰值（相对支撑基准）
        stm = (phs > 0.3) if s == 'L' else (phs < -0.3)
        base = np.median(fz[stm & (ff > FF)]) if (stm & (ff > FF)).sum() > 5 else 0
        pk = []
        i = 0
        while i < len(swm):
            if swm[i]:
                j = i
                while j < len(swm) and swm[j]:
                    j += 1
                if j - i >= 4:
                    pk.append((fz[i:j] - base).max() * 1000)
                i = j
            else:
                i += 1
        pk = np.array(pk)
        out.append(f"{s}: med={np.median(pk):.1f}mm(n={len(pk)})")
    print('  '.join(out))

print(f"\n  -- 摆动相关节幅度分解（膝/髋/踝，每段）--")
for cmdv, a, b in bounds:
    if abs(cmdv) < 0.05:
        continue
    n40 = int((b - a) * 0.4)
    a2 = a + n40
    line = f"  seg{cmdv:+.1f}: "
    for s in ('left', 'right'):
        S = s[0].upper()
        swm = (ph[a2:b] < -0.3) if S == 'L' else (ph[a2:b] > 0.3)
        kn = df[f'pos_{s}_knee_pitch_joint'].values[a2:b]
        hp = df[f'pos_{s}_hip_pitch_joint'].values[a2:b]
        ap = df[f'pos_{s}_ankle_pitch_joint'].values[a2:b]
        te_kn = df[f'pos_track_err_{s}_knee_pitch_joint'].values[a2:b]
        te_hp = df[f'pos_track_err_{s}_hip_pitch_joint'].values[a2:b]
        te_ap = df[f'pos_track_err_{s}_ankle_pitch_joint'].values[a2:b]
        # 摆动窗幅度(p2~p98)
        kn_amp = np.percentile(kn[swm], 98) - np.percentile(kn[swm], 2)
        hp_amp = np.percentile(hp[swm], 98) - np.percentile(hp[swm], 2)
        ap_amp = np.percentile(ap[swm], 98) - np.percentile(ap[swm], 2)
        line += f"|{S} 膝amp={np.degrees(kn_amp):.1f}° 髋amp={np.degrees(hp_amp):.1f}° 踝amp={np.degrees(ap_amp):.1f}° "
        line += f"err(膝{np.degrees(np.sqrt((te_kn[swm]**2).mean())):.1f}/髋{np.degrees(np.sqrt((te_hp[swm]**2).mean())):.1f}/踝{np.degrees(np.sqrt((te_ap[swm]**2).mean())):.1f}°) "
    print(line)

print(f"\n========== 2. 推进停滞 28%/23% 定位 ==========")
bpx = df['base_pos_x'].values
for cmdv, a, b in bounds:
    if abs(cmdv) < 0.05:
        continue
    n40 = int((b - a) * 0.4)
    a2 = a + n40
    ssL = (fr[a2:b] > 5) & (fl[a2:b] < 5)
    ssR = (fl[a2:b] > 5) & (fr[a2:b] < 5)
    for nm, m in [('L摆', ssL), ('R摆', ssR)]:
        adv = []
        i = 0
        while i < len(m):
            if m[i]:
                j = i
                while j < len(m) and m[j]:
                    j += 1
                if j - i >= 10:
                    d = (bpx[a2 + min(j, b - a2 - 1)] - bpx[a2 + i]) * 1000
                    if abs(d) > 10:
                        adv.append(d)
                i = j
            else:
                i += 1
        adv = np.array(adv)
        if len(adv):
            print(f"  seg{cmdv:+.1f} {nm}: med={np.median(adv):.1f} p25={np.percentile(adv,25):.1f} 停滞(<20mm)={int((adv<20).sum())}/{len(adv)}")

print(f"\n========== 3. 0.2 档速度 81% 深挖（vy/yaw/占空比）==========")
for cmdv, a, b in bounds:
    if abs(cmdv) < 0.05:
        continue
    n40 = int((b - a) * 0.4)
    a2 = a + n40
    vxm = vx[a2:b].mean()
    vym = vy[a2:b].mean()
    yawd = np.degrees(yaw[b-1] - yaw[a2])
    lsw = ((fl[a2:b] < 5)).mean()
    print(f"  seg{cmdv:+.1f}: vx={vxm:+.3f}({vxm/cmdv*100:.0f}%) vy={vym:+.3f} yaw漂={yawd:+.1f}° L摆空占比={lsw*100:.0f}%")

print(f"\n========== 4. 0.6 档 yaw -20° 分解（分段 yaw 速率 + 漂 y）==========")
b6 = [bb for c, aa, bb in bounds if abs(c - 0.6) < 1e-6][0]
a6s = b6 - 300
yawr = np.degrees(df['base_vel_yaw'].values[a6s:b6])
print(f"  0.6 档 yaw 速率: mean={yawr.mean():+.2f}°/s min={yawr.min():+.2f} max={yawr.max():+.2f}")
vy6 = df['base_vel_y'].values[a6s:b6]
print(f"  0.6 档 vy: mean={vy6.mean():+.3f} |vy|mean={np.abs(vy6).mean():.3f}")
# 每 60 步 yaw 增量
for k in range(5):
    s0 = a6s + k * 60
    s1 = min(s0 + 60, b6)
    print(f"    t{k}: yaw{np.degrees(yaw[s1-1]-yaw[s0]):+.1f}° vy={vy[s0:s1].mean():+.3f}")

print(f"\n========== 5. 左支撑髋 roll 3.26° 超标 vs 触地踝 pitch ==========")
L_ss = (fl > FF) & (fr < FF)
R_ss = (fr > FF) & (fl < FF)
walk = np.abs(cmd) > 0.05
for nm, m, col in [('L支撑', L_ss & walk, 'pos_left_hip_roll_joint'), ('R支撑', R_ss & walk, 'pos_right_hip_roll_joint')]:
    v = df[col].values[m] * R2D
    print(f"  {nm}: mean={v.mean():+.2f}° std={v.std():.2f} 默认={0.05*R2D if 'L' in nm else -0.05*R2D:+.1f}°")
