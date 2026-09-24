# -*- coding: utf-8 -*-
"""判别 exp2.1 回放时 LPF 是否真的生效
play.py 记录的是原始 action；若 LPF 生效，真实 PD 目标 = LPF(action)*scale + default。
比较两种预测与实测 tau 的相关性/残差：
  A) 不滤波: tau = KP*(0.5*a - pos) - KD*vel
  B) 滤波后: tau = KP*(0.5*LPF(a) - pos) - KD*vel
若 B 明显更吻合 → LPF 在回放中生效。
"""
import numpy as np
import pandas as pd

FS = 100.0
KP, KD = 35.0, 1.5
ALPHA = 0.38586954509503757  # 2*pi*10*0.01 / (2*pi*10*0.01 + 1)

for tag, p in [('exp2.0', 'czy/data/exp2.0/isaac_diag.csv'),
               ('exp2.1', 'czy/data/exp2.1/isaac_diag.csv')]:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    print(f'--- {tag} (alpha={ALPHA:.4f}) ---')
    for j in ['ankle_roll', 'ankle_pitch']:
        for s in ['left', 'right']:
            a = df[f'action_{s}_{j}_joint'].values[m].astype(float)
            pos = df[f'pos_{s}_{j}_joint'].values[m].astype(float)
            vel = df[f'vel_{s}_{j}_joint'].values[m].astype(float)
            tau = df[f'effort_{s}_{j}_joint'].values[m].astype(float)
            # 递推一阶低通
            af = np.empty_like(a); af[0] = a[0]
            for k in range(1, len(a)):
                af[k] = af[k - 1] + ALPHA * (a[k] - af[k - 1])
            ta = KP * (0.5 * a - pos) - KD * vel
            tb = KP * (0.5 * af - pos) - KD * vel
            ca = np.corrcoef(ta, tau)[0, 1]
            cb = np.corrcoef(tb, tau)[0, 1]
            ra = np.sqrt(np.mean((ta - tau) ** 2))
            rb = np.sqrt(np.mean((tb - tau) ** 2))
            print(f'  {j:>12} {s:>5}  不滤波 corr={ca:.4f} resid={ra:.3f} | '
                  f'滤波 corr={cb:.4f} resid={rb:.3f} | 胜={("B滤波" if rb < ra else "A不滤波")}')
    print()
