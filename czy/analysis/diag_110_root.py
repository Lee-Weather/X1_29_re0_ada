# -*- coding: utf-8 -*-
"""exp_ada_1.10 最新数据深度分析（决策导向）：
   抬脚不对称 R/L=139% 是"物理执行增益差"还是"弧线行为补偿"？
   —— 直接决定 1.11 走物理层(armature 对称化) 还是奖励层(加码惩罚)

Q1 分段时间序列：抬脚比 vs yaw_rate（若强相关→行为；若恒定→物理）
Q2 静止段检验：无弧线时是否仍不对称（是→物理基线偏置）
Q3 执行增益：关节跟踪误差 rms L vs R（执行差直接证据）
Q4 力矩：镜像相位对齐后的周期均值 L vs R
Q5 抖动来源：各关节 vel 差分能量 L vs R
Q6 hip_yaw 基线偏置检查
"""
import numpy as np
import pandas as pd

CSV = 'czy/data/exp_ada_1.10/isaac_diag.csv'
FS = 50.0

df = pd.read_csv(CSV, encoding='utf-8-sig')
cmd = df['cmd_linear_x'].values
ph = df['phase_sin'].values
wz = df['base_ang_vel_z'].values
fzl, fzr = df['foot_z_l'].values * 1000, df['foot_z_r'].values * 1000

edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
segs = [(a, b, cmd[a]) for a, b in zip(edges[:-1], edges[1:]) if b - a > 100]


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


print('=' * 84)
print('Q1 抬脚不对称：分段时间序列（摆动相峰值抬脚 median）')
print(f'{"段cmd":>7} {"yaw_rate°/s":>11} {"L抬脚mm":>9} {"R抬脚mm":>9} {"R/L":>7} {"L峰n":>5} {"R峰n":>5}')
rows = []
for a, b, c in segs:
    phs, zl, zr = ph[a:b], fzl[a:b], fzr[a:b]
    lp = swing_peaks(zl, phs < -0.3)
    rp = swing_peaks(zr, phs > 0.3)
    wrm = np.median(wz[a + int((b - a) * 0.4):b]) * 180 / np.pi
    L = np.median(lp) if len(lp) else np.nan
    R = np.median(rp) if len(rp) else np.nan
    ratio = R / L * 100 if (len(lp) and len(rp) and L > 0) else np.nan
    rows.append((c, wrm, L, R, ratio))
    print(f'{c:>7.2f} {wrm:>11.2f} {L:>9.1f} {R:>9.1f} {ratio:>7.0f}% {len(lp):>5} {len(rp):>5}')

mv = [(c, w, L, R, r) for c, w, L, R, r in rows if abs(c) > 0.05 and not np.isnan(r)]
if len(mv) > 2:
    ws = np.array([x[1] for x in mv]); rs = np.array([x[4] for x in mv])
    print(f'\n  [运动段] yaw_rate vs R/L抬脚比: r={np.corrcoef(ws, rs)[0, 1]:+.2f} (n={len(mv)})  '
          f'R/L mean={rs.mean():.0f}% 范围 {rs.min():.0f}~{rs.max():.0f}%')
print('\n  [静止段检验] 无弧线域是否仍不对称：')
for c, w, L, R, r in rows:
    if abs(c) <= 0.05:
        tag = f'{r:.0f}%' if not np.isnan(r) else '无摆动'
        print(f'    cmd={c:+.2f}: yaw_rate={w:+.2f}°/s  L={L:.1f} R={R:.1f}  R/L={tag}')

mv_mask = np.abs(cmd) > 0.05

print('\n' + '=' * 84)
print('Q3 执行增益：关节跟踪误差 rms（运动段）')
for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']:
    el = df[f'pos_track_err_left_{j}_joint'].values[mv_mask]
    er = df[f'pos_track_err_right_{j}_joint'].values[mv_mask]
    rl = np.sqrt((el ** 2).mean()) * 180 / np.pi
    rr = np.sqrt((er ** 2).mean()) * 180 / np.pi
    print(f'  {j:>12}: L={rl:>6.2f}°  R={rr:>6.2f}°  R/L={rr / rl * 100:>5.0f}%')

print('\n' + '=' * 84)
print('Q4 关节力矩（镜像相位对齐：L 按 ph 分箱、R 按 -ph 分箱）')
nb = 8
bins = np.linspace(-1, 1, nb + 1)
for j in ['hip_pitch', 'hip_roll', 'knee_pitch', 'ankle_pitch']:
    tl = df[f'effort_left_{j}_joint'].values
    tr = df[f'effort_right_{j}_joint'].values
    pairs = []
    for k in range(nb):
        lo, hi = bins[k], bins[k + 1]
        ml = (ph >= lo) & (ph < hi)
        mr = ((-ph) >= lo) & ((-ph) < hi)
        if ml.sum() and mr.sum():
            pairs.append((tl[ml].mean(), tr[mr].mean()))
    if pairs:
        Ls = np.mean([x[0] for x in pairs]); Rs = np.mean([x[1] for x in pairs])
        pk_l = np.abs(tl).mean(); pk_r = np.abs(tr).mean()
        print(f'  {j:>12}: 周期均值 L={Ls:>+7.2f} R={Rs:>+7.2f}Nm  |力矩|均值 L={pk_l:.2f} R={pk_r:.2f} R/L={pk_r / pk_l * 100:.0f}%')

print('\n' + '=' * 84)
print('Q5 关节抖动（运动段 vel 差分 std×FS）')
for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']:
    vl = df[f'vel_left_{j}_joint'].values[mv_mask]
    vr = df[f'vel_right_{j}_joint'].values[mv_mask]
    jl = np.std(np.diff(vl)) * FS
    jr = np.std(np.diff(vr)) * FS
    print(f'  {j:>12}: L={jl:>7.1f}  R={jr:>7.1f}  R/L={jr / max(jl, 1e-6) * 100:>5.0f}%')

print('\n' + '=' * 84)
print('Q6 hip_yaw 基线偏置（静止段）')
st_mask = np.abs(cmd) <= 0.05
for s in ['left', 'right']:
    p = df[f'pos_{s}_hip_yaw_joint'].values[st_mask]
    pdes = df[f'pos_des_{s}_hip_yaw_joint'].values[st_mask]
    print(f'  {s:>5}: pos median={np.median(p) * 180 / np.pi:>+7.2f}°  des median={np.median(pdes) * 180 / np.pi:>+7.2f}°')

print('\n' + '=' * 84)
print('Q7 足底接触力峰值（运动段，左右对比）')
ffl, ffr = df['foot_force_l'].values, df['foot_force_r'].values
print(f'  L: mean={ffl[mv_mask].mean():.1f} p95={np.percentile(ffl[mv_mask], 95):.1f} max={ffl[mv_mask].max():.1f}')
print(f'  R: mean={ffr[mv_mask].mean():.1f} p95={np.percentile(ffr[mv_mask], 95):.1f} max={ffr[mv_mask].max():.1f}')
