# -*- coding: utf-8 -*-
"""停滞口径修正验证：前进档/后退档分开统计（方向感知）——1.8 vs 1.7 vs 1.6"""
import io
import sys
import numpy as np
import pandas as pd

sys.stdout = io.open('_diag18b_out.txt', 'w', encoding='utf-8')

for tag, csv in [('1.8', 'czy/data/exp_ada_1.8/isaac_diag.csv'),
                 ('1.7', 'czy/data/exp_ada_1.7/isaac_diag.csv'),
                 ('1.6', 'czy/data/exp_ada_1.6/isaac_diag.csv')]:
    df = pd.read_csv(csv, encoding='utf-8-sig')
    fl, fr = df['foot_force_l'].values, df['foot_force_r'].values
    bpx = df['base_pos_x'].values
    cmd = df['cmd_linear_x'].values
    fwd = cmd > 0.05
    print(f"== {tag} 前进档方向感知停滞（d<20 为停滞）==")
    for nm, m in [('L摆', (fr > 5) & (fl < 5) & fwd), ('R摆', (fl > 5) & (fr < 5) & fwd)]:
        adv, stall = [], 0
        i, n = 0, len(m)
        while i < n:
            if m[i]:
                j = i
                while j < n and m[j]:
                    j += 1
                if j - i >= 10:
                    d = (bpx[min(j, n-1)] - bpx[i]) * 1000
                    if abs(d) > 10:
                        adv.append(d)
                        if d < 20:
                            stall += 1
                i = j
            else:
                i += 1
        adv = np.array(adv)
        print(f"  {nm}: n={len(adv)} med={np.median(adv):.1f} p25={np.percentile(adv,25):.1f} 停滞={stall}/{len(adv)}={stall/max(len(adv),1)*100:.0f}%")
sys.stdout.close()
