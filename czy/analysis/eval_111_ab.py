# -*- coding: utf-8 -*-
"""exp_ada_1.11 vs 1.11l 验收对比：LCP 对训练质量的影响（A/B 对照）"""
import numpy as np
import pandas as pd

FS = 50.0
PAIRS = [('1.11 (无LCP)', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
         ('1.11l (LCP 1e-4)', 'czy/data/exp_ada_1.11l/isaac_diag.csv')]


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


for tag, path in PAIRS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    cmd = df['cmd_linear_x'].values
    ph = df['phase_sin'].values
    fzl, fzr = df['foot_z_l'].values * 1000, df['foot_z_r'].values * 1000
    wz = df['base_ang_vel_z'].values
    print(f"\n{'=' * 78}\n== {tag}")
    print(f"  LCP 列: jac_frob 中位={np.median(df['lcp_jac_frob']):.3f}  "
          f"短历史占比={np.median(df['lcp_frac_short']):.3f}  σ加权={np.median(df['lcp_sigma_w']):.1f}")
    print(f"  {'段cmd':>6} {'track%':>7} {'yaw总°':>7} {'yaw_rate°/s':>11} {'实际vx':>7}")
    for a, b, c in segs_of(df):
        vx = df['base_vel_x'].values[a:b]
        y = df['base_yaw'].values[a:b]
        yaw_tot = (y[-1] - y[0]) * 180 / np.pi
        wr = np.median(wz[a + int((b - a) * 0.4):b]) * 180 / np.pi
        track = (np.median(vx[int((b - a) * 0.4):]) / c * 100) if abs(c) > 0.05 else np.nan
        print(f"  {c:>6.2f} {track:>6.0f}% {yaw_tot:>7.1f} {wr:>11.2f} {np.median(vx[int((b-a)*0.4):]):>7.3f}")
    # 抬脚
    m = np.abs(cmd) > 0.05
    lp = swing_peaks(fzl, (ph < -0.3) & m)
    rp = swing_peaks(fzr, (ph > 0.3) & m)
    if len(lp) and len(rp):
        print(f"  抬脚: L median={np.median(lp):.1f} (n={len(lp)})  R median={np.median(rp):.1f} (n={len(rp)})  "
              f"R/L={np.median(rp)/np.median(lp)*100:.0f}%")
    else:
        print(f"  抬脚: L n={len(lp)}  R n={len(rp)}  (摆动相峰值过少)")
    # 抖动
    for j in ['ankle_roll', 'knee_pitch']:
        jl = np.std(np.diff(df[f'vel_left_{j}_joint'].values[m])) * FS
        jr = np.std(np.diff(df[f'vel_right_{j}_joint'].values[m])) * FS
        print(f"  抖动 {j}: L={jl:.1f} R={jr:.1f}")
