# -*- coding: utf-8 -*-
"""左右执行对称性跨版本对比（1.6~1.10）：判定不对称是"物理恒定"还是"随配置漂移"
   100% = 左右对称；偏离越大越不对称
"""
import numpy as np
import pandas as pd

FS = 50.0
VERS = [('exp_ada_1.10', 'czy/data/exp_ada_1.10/isaac_diag.csv'),
        ('exp_ada_1.9', 'czy/data/exp_ada_1.9/isaac_diag.csv'),
        ('exp_ada_1.8', 'czy/data/exp_ada_1.8/isaac_diag.csv'),
        ('exp_ada_1.7', 'czy/data/exp_ada_1.7/isaac_diag.csv'),
        ('exp_ada_1.6', 'czy/data/exp_ada_1.6/isaac_diag.csv')]


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


def metrics(csv):
    df = pd.read_csv(csv, encoding='utf-8-sig')
    cmd = df['cmd_linear_x'].values
    ph = df['phase_sin'].values
    m = np.abs(cmd) > 0.05
    fzl, fzr = df['foot_z_l'].values * 1000, df['foot_z_r'].values * 1000
    lp = swing_peaks(fzl, (ph < -0.3) & m)
    rp = swing_peaks(fzr, (ph > 0.3) & m)
    lift = np.median(rp) / np.median(lp) * 100 if len(lp) and len(rp) else np.nan

    def tr(j):
        el = df[f'pos_track_err_left_{j}_joint'].values[m]
        er = df[f'pos_track_err_right_{j}_joint'].values[m]
        return np.sqrt((er ** 2).mean()) / np.sqrt((el ** 2).mean()) * 100

    def jit(j):
        vl = df[f'vel_left_{j}_joint'].values[m]
        vr = df[f'vel_right_{j}_joint'].values[m]
        return np.std(np.diff(vr)) / max(np.std(np.diff(vl)), 1e-9) * 100

    def tq(j):
        tl = np.abs(df[f'effort_left_{j}_joint'].values[m]).mean()
        trr = np.abs(df[f'effort_right_{j}_joint'].values[m]).mean()
        return trr / max(tl, 1e-9) * 100

    return dict(lift=lift, tq_knee=tq('knee_pitch'), tq_ank=tq('ankle_pitch'),
                tq_hip=tq('hip_pitch'), jit_knee=jit('knee_pitch'),
                jit_ankr=jit('ankle_roll'), tr_knee=tr('knee_pitch'),
                tr_ank=tr('ankle_pitch'), tr_hip=tr('hip_pitch'),
                tr_hiproll=tr('hip_roll'))


print(f'{"版本":>13} {"抬脚":>7} {"膝力矩":>7} {"踝力矩":>7} {"髋力矩":>7} '
      f'{"膝抖":>7} {"踝roll抖":>9} {"膝跟踪":>7} {"踝跟踪":>7} {"髋roll跟踪":>10}')
print('-' * 92)
for tag, csv in VERS:
    try:
        d = metrics(csv)
    except FileNotFoundError:
        print(f'{tag:>13}  (缺数据)')
        continue
    print(f'{tag:>13} {d["lift"]:>6.0f}% {d["tq_knee"]:>6.0f}% {d["tq_ank"]:>6.0f}% {d["tq_hip"]:>6.0f}% '
          f'{d["jit_knee"]:>6.0f}% {d["jit_ankr"]:>8.0f}% {d["tr_knee"]:>6.0f}% {d["tr_ank"]:>6.0f}% '
          f'{d["tr_hiproll"]:>9.0f}%')
print('\n(100% = 左右对称；R/L 越偏离 100% 越不对称)')
