# -*- coding: utf-8 -*-
"""踝关节抖动专项诊断 v2（修正频谱方法学：分段 FFT，不拼接不连续段）

核心判据：pos_des（策略指令目标）抖动 vs pos（实际关节角）抖动
  - pos_des 抖 ≈ pos 抖  → 策略主动输出抖动（网络层问题，LCP 该管）
  - pos_des 平滑 但 pos 抖 → 物理/接触层问题（PD 跟踪失败、触地冲击），LCP 管不到
"""
import glob
import os
import numpy as np
import pandas as pd

FS = 50.0

CAND = [('1.6', 'czy/data/exp_ada_1.6/isaac_diag.csv'),
        ('1.7', 'czy/data/exp_ada_1.7/isaac_diag.csv'),
        ('1.8', 'czy/data/exp_ada_1.8/isaac_diag.csv'),
        ('1.9', 'czy/data/exp_ada_1.9/isaac_diag.csv'),
        ('1.10', 'czy/data/exp_ada_1.10/isaac_diag.csv'),
        ('1.11', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('1.11l', 'czy/data/exp_ada_1.11l/isaac_diag.csv'),
        ('exp2.0', 'czy/data/exp2.0/isaac_diag.csv')]
VERS = []
for tag, p in CAND:
    if os.path.exists(p):
        VERS.append((tag, p))
    else:
        g = glob.glob(p.replace('isaac_diag.csv', '*.csv'))
        if g:
            VERS.append((tag, g[0]))


def load(p):
    df = pd.read_csv(p, encoding='utf-8-sig')
    return df


def segs(df, minlen=200):
    """返回连续运动段 (a,b)，且段内 |cmd|>0.05 恒定"""
    cmd = df['cmd_linear_x'].values
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    return [(a, b) for a, b in zip(edges[:-1], edges[1:]) if b - a >= minlen and abs(cmd[a]) > 0.05]


def jit(x):
    return np.std(np.diff(x)) * FS


def seg_psd(df, col, fc=5.0):
    """分段 PSD 平均 → 返回 (主频, >fc 高频占比, 2.5~4Hz 窄带占比)"""
    doms, hfs, nb = [], [], []
    for a, b in segs(df):
        s = df[col].values[a:b].astype(float)
        s = s - s.mean()
        n = len(s)
        w = np.hanning(n)
        P = np.abs(np.fft.rfft(s * w)) ** 2
        f = np.fft.rfftfreq(n, 1 / FS)
        tot = P.sum()
        if tot <= 0:
            continue
        k = np.argmax(P[1:]) + 1
        doms.append(f[k])
        hfs.append(P[f > fc].sum() / tot)
        nb.append(P[(f >= 2.5) & (f <= 4.0)].sum() / tot)
    if not doms:
        return np.nan, np.nan, np.nan
    return np.median(doms), np.mean(hfs), np.mean(nb)


def zcr(x):
    """过零率（Hz）——高频振荡的简易代理"""
    s = x - x.mean()
    return ((s[:-1] * s[1:]) < 0).sum() / 2 / (len(s) / FS)


print('=' * 116)
print('【A】踝关节频谱（分段 FFT 平均，无拼接伪影）  [步态基频 1/0.7=1.43Hz]')
print('=' * 116)
print(f'{"版本":>8} {"roll主频":>9} {"roll>5Hz":>10} {"roll2.5-4Hz":>12} {"pitch主频":>10} '
      f'{"pitch>5Hz":>10} {"roll过零率":>10} {"pitch过零率":>11}')
print('-' * 116)
for tag, p in VERS:
    df = load(p)
    r = {}
    for j in ['ankle_roll', 'ankle_pitch']:
        cols = [f'vel_left_{j}_joint', f'vel_right_{j}_joint']
        d, h, nb = zip(*[seg_psd(df, c) for c in cols])
        r[j] = (np.nanmean(d), np.nanmean(h) * 100, np.nanmean(nb) * 100)
    zr = np.mean([zcr(df[f'vel_left_ankle_roll_joint'].values), zcr(df[f'vel_right_ankle_roll_joint'].values)])
    zp = np.mean([zcr(df[f'vel_left_ankle_pitch_joint'].values), zcr(df[f'vel_right_ankle_pitch_joint'].values)])
    print(f'{tag:>8} {r["ankle_roll"][0]:>9.2f} {r["ankle_roll"][1]:>9.1f}% '
          f'{r["ankle_roll"][2]:>11.1f}% {r["ankle_pitch"][0]:>10.2f} '
          f'{r["ankle_pitch"][1]:>9.1f}% {zr:>10.1f} {zp:>11.1f}')

print()
print('=' * 116)
print('【B】驱动源溯源：策略指令目标抖动 vs 实际关节抖动（关键判据）')
print('=' * 116)
print(f'{"版本":>8} {"roll pos_des抖°":>15} {"roll pos抖°":>12} {"roll指令/实际":>13} '
      f'{"pitch pos_des抖°":>16} {"pitch pos抖°":>13} {"pitch指令/实际":>14}')
print('-' * 116)
for tag, p in VERS:
    df = load(p)
    row = []
    for j in ['ankle_roll', 'ankle_pitch']:
        segs_ = segs(df)
        if not segs_:
            row += [np.nan, np.nan, np.nan]
            continue
        ds, ps = [], []
        for a, b in segs_:
            for sd in ['left', 'right']:
                ds.append(jit(df[f'pos_des_{sd}_{j}_joint'].values[a:b]) * 180 / np.pi)
                ps.append(jit(df[f'pos_{sd}_{j}_joint'].values[a:b]) * 180 / np.pi)
        row += [np.mean(ds), np.mean(ps), np.mean(ds) / max(np.mean(ps), 1e-9)]
    print(f'{tag:>8} {row[0]:>15.2f} {row[1]:>12.2f} {row[2]:>13.2f} '
          f'{row[3]:>16.2f} {row[4]:>13.2f} {row[5]:>14.2f}')

print()
print('=' * 116)
print('【C】分相抖动 + 触地事件（支撑相 vs 摆动相）')
print('=' * 116)
print(f'{"版本":>8} {"roll撑L/R":>13} {"pitch撑L/R":>14} {"roll速率@触地":>13} {"力峰kN L/R":>12} '
      f'{"撑相力std kN":>13}')
print('-' * 116)
for tag, p in VERS:
    df = load(p)
    ph = df['phase_sin'].values
    cmd = df['cmd_linear_x'].values
    m = np.abs(cmd) > 0.05
    o = []
    for j in ['ankle_roll', 'ankle_pitch']:
        v = [np.std(df[f'vel_{sd}_{j}_joint'].values[m & (ph > 0.3 if sd == 'left' else ph < -0.3)]) * FS
             for sd in ['left', 'right']]
        o.append('/'.join(f'{x:.0f}' for x in v))
    rate = np.abs(np.diff(df['vel_left_ankle_roll_joint'].values)) * FS
    force = df['foot_force_l'].values
    st = np.where((force[:-1] < 5) & (force[1:] > 50))[0]
    sr = rate[st].mean() if len(st) > 3 else np.nan
    fl, fr = force[m].max() / 1000, df['foot_force_r'].values[m].max() / 1000
    fs_ = np.std(force[m & (ph > 0.3)]) / 1000
    print(f'{tag:>8} {o[0]:>13} {o[1]:>14} {sr:>13.0f} {fl:>5.1f}/{fr:<5.1f} {fs_:>13.2f}')

print()
print('=' * 116)
print('【D】踝力矩与跟踪误差（执行层负担）')
print('=' * 116)
print(f'{"版本":>8} {"roll err°":>10} {"pitch err°":>11} {"roll|τ|Nm":>11} {"pitch|τ|Nm":>12} '
      f'{"roll τ抖":>10} {"pitch τ抖":>11} {"roll err抖":>11}')
print('-' * 116)
for tag, p in VERS:
    df = load(p)
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    o = []
    for j in ['ankle_roll', 'ankle_pitch']:
        e = np.mean([np.abs(df[f'pos_track_err_{sd}_{j}_joint'].values[m]).mean() for sd in ['left', 'right']]) * 180 / np.pi
        t = np.mean([np.abs(df[f'effort_{sd}_{j}_joint'].values[m]).mean() for sd in ['left', 'right']])
        tj = np.mean([jit(df[f'effort_{sd}_{j}_joint'].values[m]) for sd in ['left', 'right']])
        o.append((e, t, tj))
    ed = np.mean([jit(df[f'pos_track_err_{sd}_ankle_roll_joint'].values[m]) for sd in ['left', 'right']]) * 180 / np.pi
    print(f'{tag:>8} {o[0][0]:>10.2f} {o[1][0]:>11.2f} {o[0][1]:>11.2f} {o[1][1]:>12.2f} '
          f'{o[0][2]:>10.1f} {o[1][2]:>11.1f} {ed:>11.1f}')
