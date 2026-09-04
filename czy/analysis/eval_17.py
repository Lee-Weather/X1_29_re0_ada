# -*- coding: utf-8 -*-
"""exp_ada_1.7 验收：最低抬脚红线/左右对称/配对差/速度/保持项（foot_z 真值口径）"""
import numpy as np
import pandas as pd

R2D = 180 / np.pi
CSV = 'czy/data/exp_ada_1.7/isaac_diag.csv'
df = pd.read_csv(CSV, encoding='utf-8-sig')
print(f"rows={len(df)}, dt≈{df['time_s'].iloc[1]-df['time_s'].iloc[0]:.4f}s, dur={df['time_s'].iloc[-1]:.1f}s")

FF = 5.0
FF1 = 1.0

# ---------- 分段速度跟踪（稳态后60%，含起步过渡剥离）----------
print("\n=== 分段速度跟踪（稳态后60%）===")
cmdx = df['cmd_linear_x'].values
segs = []
cur, start = cmdx[0], 0
for i in range(1, len(df) + 1):
    if i == len(df) or cmdx[i] != cur:
        segs.append((cur, start, i))
        if i < len(df):
            cur, start = cmdx[i], i
for cmd, a, b in segs:
    vx = df['base_vel_x'].values[a:b]
    n40 = int(len(vx) * 0.4)
    real = vx[n40:].mean()
    track = real / cmd * 100 if abs(cmd) > 1e-6 else float('nan')
    yaw = (df['base_yaw'].values[b-1] - df['base_yaw'].values[a]) * R2D
    print(f"  cmd={cmd:+.2f} [{a}:{b}] n={b-a}  real={real:+.3f} track={track:.0f}%  yawDrift={yaw:+.1f}°")

# ---------- 抬脚峰值（摆动相 foot_z 相对支撑基准，用真值 foot_z）----------
def lift_peaks(fz, ff, ph, phase_sign=1.0, side='l'):
    # 本侧摆动窗
    sw = (ph * phase_sign < -0.3) if side == 'l' else (ph * phase_sign > 0.3)
    # 支撑基准：本侧支撑（对侧摆动中段）
    st = (ph * phase_sign > 0.3) if side == 'l' else (ph * phase_sign < -0.3)
    base = np.median(fz[st & (ff > FF)])
    rel = fz - base
    peaks, i, n = [], 0, len(fz)
    while i < n:
        if sw[i]:
            j = i
            while j < n and sw[j]:
                j += 1
            if j - i >= 5:
                peaks.append(rel[i:j].max())
            i = j
        else:
            i += 1
    return np.array(peaks) * 1000, rel * 1000

# 相位约定：用力数据标定
ph_raw = df['phase_sin'].values
l_sw = (df['foot_force_l'].values < FF)[ph_raw > 0.3].mean()
r_sw = (df['foot_force_r'].values < FF)[ph_raw > 0.3].mean()
phase_sign = 1.0 if r_sw > l_sw else -1.0
print(f"\n相位标定: sin>0 时 R摆动={r_sw:.0%}/L摆动={l_sw:.0%} -> sign={phase_sign:+.0f}")

fl, rel_l = lift_peaks(df['foot_z_l'].values, df['foot_force_l'].values, ph_raw, phase_sign, 'l')
fr, rel_r = lift_peaks(df['foot_z_r'].values, df['foot_force_r'].values, ph_raw, phase_sign, 'r')
print("\n=== 抬脚峰值（foot_z 真值，相对本侧支撑基准）===")
for nm, p in [('L', fl), ('R', fr)]:
    print(f"  {nm}: n={len(p)} median={np.median(p):.1f}mm p25={np.percentile(p,25):.1f} p75={np.percentile(p,75):.1f} "
          f"min={p.min():.1f} max={p.max():.1f}  拖地步(<10mm)={(p<10).sum()}")

# 红线1: min（剔除起步第一步）
fl_steady = fl[1:] if len(fl) > 3 else fl
fr_steady = fr[1:] if len(fr) > 3 else fr
print(f"\n  [红线1] min抬脚 L={fl.min():.1f}mm (稳态剔除起步: {fl_steady.min():.1f})  R={fr.min():.1f}mm (稳态: {fr_steady.min():.1f})")
print(f"          p10抬脚 L={np.percentile(fl,10):.1f}  R={np.percentile(fr,10):.1f}")

# 红线2: 对称比 + 配对差
sym_ratio = np.median(fr_steady) / max(np.median(fl_steady), 1e-6) * 100
# 配对差（重量化：取前 kk 步按时间对齐）
kk = min(len(fl_steady), len(fr_steady))
pair_diff = np.median(np.abs(fl_steady[:kk] - fr_steady[:kk]))
print(f"\n  [红线2] 对称比 R/L={sym_ratio:.0f}%  抬脚差 L-R={np.median(fl_steady)-np.median(fr_steady):+.1f}mm  median配对差={pair_diff:.1f}mm")

# ---------- 保持项 ----------
print("\n=== 保持项 ===")
# 拖擦（摆动相 foot_z<5mm 比例）
for s_ in ['l', 'r']:
    swing = df[f'foot_force_{s_}'].values < FF1
    low = swing & (df[f'foot_z_{s_}'].values < 0.005)
    print(f"  拖擦 {s_.upper()}: {low.sum()/max(swing.sum(),1):.1f}%")
# 踝抖（摆动相 ankle_pitch |vel|>0.5 过零/s）
for s_ in ['left', 'right']:
    v = df[f'vel_{s_}_ankle_pitch_joint'].values
    ffv = df[f'foot_force_{s_[0]}'].values
    swing = ffv < FF1
    vs = np.where(swing, v, 0)
    sign = np.sign(vs); sign[np.abs(vs) < 0.5] = 0; sign = sign[sign != 0]
    flips = (np.diff(sign) != 0).sum()
    dur = swing.sum() / 50.0
    print(f"  踝抖 {s_[:1].upper()}: {flips/max(dur,1e-6):.1f} 次/s")
# 左支撑髋 roll / 单支撑滑移
L_ss = (df['foot_force_l'].values > FF) & (df['foot_force_r'].values < FF)
hr_l = df['pos_left_hip_roll_joint'].values[L_ss] * R2D
vy_l = df['base_vel_y'].values[L_ss] * 100
print(f"  左支撑髋roll偏离: mean={abs(hr_l.mean()):.2f}° (std {hr_l.std():.2f})  单支撑滑移: |mean|={abs(vy_l.mean()):.1f}cm/s")
# yaw 漂移 0.2 档已在分段给出
# 触地踝 pitch
print("  触地踝pitch(default -12°)（脚跟着地）:")
for s_ in ['left', 'right']:
    ff = df[f'foot_force_{s_[0]}'].values
    ap = df[f'pos_{s_}_ankle_pitch_joint'].values * R2D
    idx = np.where((ff[1:] > FF) & (ff[:-1] <= FF))[0] + 1
    print(f"    {s_[:1].upper()}: median={np.median(ap[idx]):+.1f}° (n={len(idx)})")