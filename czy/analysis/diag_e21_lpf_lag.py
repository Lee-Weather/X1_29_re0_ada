# -*- coding: utf-8 -*-
"""判别 LPF 是否生效（硬判据：滞后）
反推真实 PD 目标: target_inferred = pos + (tau + KD*vel)/KP
再看它相对"原始 action 目标"(0.5*a + default) 的最优对齐滞后（控制步数）。
  - 无滤波 → 最优滞后 = 0
  - LPF fc=10Hz @100Hz → 群延迟 ≈ 1/(2π*10) = 15.9ms ≈ 1.6 步 → 最优滞后 ~1~2 步
另附 lcp_jac_frob / 速度跟踪 对比。
"""
import numpy as np
import pandas as pd

FS = 100.0
KP, KD = 35.0, 1.5
DEFAULT = 0.0  # ankle_roll / ankle_pitch 默认角（roll=0；pitch=-0.21）
CAND = [('exp2.0', 'czy/data/exp2.0/isaac_diag.csv'),
        ('exp2.1', 'czy/data/exp2.1/isaac_diag.csv')]
MAXLAG = 6

for tag, p in CAND:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    print(f'--- {tag} ---')
    for j, dflt in [('ankle_roll', 0.0), ('ankle_pitch', -0.21)]:
        for s in ['left', 'right']:
            a = df[f'action_{s}_{j}_joint'].values[m].astype(float)
            pos = df[f'pos_{s}_{j}_joint'].values[m].astype(float)
            vel = df[f'vel_{s}_{j}_joint'].values[m].astype(float)
            tau = df[f'effort_{s}_{j}_joint'].values[m].astype(float)
            tgt_inf = pos + (tau + KD * vel) / KP          # 反推真实目标
            a_tgt = 0.5 * a + dflt
            best, bestc = None, -9
            cs = []
            for L in range(MAXLAG + 1):
                x = a_tgt[:len(a_tgt) - L] if L else a_tgt
                y = tgt_inf[L:]
                c = np.corrcoef(x, y)[0, 1]
                cs.append(c)
                if c > bestc:
                    bestc, best = c, L
            print(f'  {j:>12} {s:>5} 最优滞后={best} 步 (corr={bestc:.4f}) | '
                  f'各滞后corr: ' + ' '.join(f'{c:.3f}' for c in cs))
    # LCP 平滑度 & 速度跟踪
    if 'lcp_jac_frob' in df.columns:
        print(f'  lcp_jac_frob: mean={df["lcp_jac_frob"].values[m].mean():.3f}')
    for seg, lo, hi in [('0.2', 500, 1000), ('0.4', 1000, 1500), ('0.6', 1500, 2000)]:
        vx = df['base_vel_x'].values[lo:hi]
        print(f'  track {seg}: 实际 vx = {vx[-600:].mean() if hi <= len(vx) else vx.mean():.3f} '
              f'(指令 {seg})')
    print()
