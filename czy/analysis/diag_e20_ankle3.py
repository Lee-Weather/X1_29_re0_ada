# -*- coding: utf-8 -*-
"""踝关节抖动专项诊断 v3（修正采样率 FS=100Hz；用 time_s 反推校验）

判据：
  ① pos_des（策略指令目标）抖 vs pos（实际关节）抖 → 区分"策略层"与"物理层"
  ② 高频抖动量 = std(x - 移动平均)（>5Hz 成分的绝对幅值，单位明确）
  ③ 主频 / 过零率 → 区分相干颤振 vs 步态节律
"""
import glob
import os
import numpy as np
import pandas as pd

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

GAIT = 1 / 0.7  # 1.43 Hz


def load(p):
    df = pd.read_csv(p, encoding='utf-8-sig')
    if 'time_s' in df.columns:
        t = df['time_s'].values
        fs = 1.0 / np.median(np.diff(t))
    else:
        fs = 100.0
    return df, fs


def segs(df, fs, minlen_s=2.0):
    cmd = df['cmd_linear_x'].values
    n = int(minlen_s * fs)
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    return [(a, b) for a, b in zip(edges[:-1], edges[1:]) if b - a >= n and abs(cmd[a]) > 0.05]


def jit(x, fs):
    return np.std(np.diff(x)) * fs


def hp_amp(x, fs, fc=5.0):
    """>fc Hz 成分的绝对幅值：x - 移动平均"""
    w = max(3, int(fs / fc))
    k = np.ones(w) / w
    low = np.convolve(x, k, mode='same')
    return np.std(x - low)


def seg_fft(df, col, fs):
    doms = []
    for a, b in segs(df, fs):
        s = df[col].values[a:b].astype(float)
        s = s - s.mean()
        P = np.abs(np.fft.rfft(s * np.hanning(len(s)))) ** 2
        f = np.fft.rfftfreq(len(s), 1 / fs)
        doms.append(f[np.argmax(P[1:]) + 1])
    return np.median(doms) if doms else np.nan


def zcr(x, fs):
    s = x - x.mean()
    return ((s[:-1] * s[1:]) < 0).sum() / 2 / (len(s) / fs)


df0, FS = load(VERS[-1][1])
print(f'采样率 FS = {FS:.1f} Hz（用 time_s 反推）  步态基频 = {GAIT:.2f} Hz')
print()

print('=' * 118)
print('【A】踝抖动量（运动段，单位 °/s）—— pos_des=策略指令目标，pos=实际关节')
print('=' * 118)
print(f'{"版本":>8} {"roll指令抖":>10} {"roll实际抖":>10} {"指令/实际":>9} '
      f'{"pitch指令抖":>11} {"pitch实际抖":>11} {"roll高频抖":>10} {"pitch高频抖":>11}')
print('-' * 118)
for tag, p in VERS:
    df, fs = load(p)
    o = {}
    for j in ['ankle_roll', 'ankle_pitch']:
        ds, ps, hs = [], [], []
        for a, b in segs(df, fs):
            for sd in ['left', 'right']:
                ds.append(jit(df[f'pos_des_{sd}_{j}_joint'].values[a:b], fs) * 180 / np.pi)
                ps.append(jit(df[f'pos_{sd}_{j}_joint'].values[a:b], fs) * 180 / np.pi)
                hs.append(hp_amp(df[f'vel_{sd}_{j}_joint'].values[a:b], fs) * 180 / np.pi)
        o[j] = (np.mean(ds), np.mean(ps), np.mean(hs))
    print(f'{tag:>8} {o["ankle_roll"][0]:>10.1f} {o["ankle_roll"][1]:>10.1f} '
          f'{o["ankle_roll"][0]/max(o["ankle_roll"][1],1e-9):>9.2f} '
          f'{o["ankle_pitch"][0]:>11.1f} {o["ankle_pitch"][1]:>11.1f} '
          f'{o["ankle_roll"][2]:>10.2f} {o["ankle_pitch"][2]:>11.2f}')

print()
print('=' * 118)
print('【B】频谱性质（主频 / 过零率，单位 Hz）')
print('=' * 118)
print(f'{"版本":>8} {"roll主频":>9} {"pitch主频":>10} {"髋pitch主频":>11} {"膝pitch主频":>11} '
      f'{"roll过零率":>10} {"pitch过零率":>11} {"roll主频/步态":>12}')
print('-' * 118)
for tag, p in VERS:
    df, fs = load(p)
    dr = np.mean([seg_fft(df, f'vel_{sd}_ankle_roll_joint', fs) for sd in ['left', 'right']])
    dp = np.mean([seg_fft(df, f'vel_{sd}_ankle_pitch_joint', fs) for sd in ['left', 'right']])
    dh = np.mean([seg_fft(df, f'vel_{sd}_hip_pitch_joint', fs) for sd in ['left', 'right']])
    dk = np.mean([seg_fft(df, f'vel_{sd}_knee_pitch_joint', fs) for sd in ['left', 'right']])
    zr = np.mean([zcr(df[f'vel_{sd}_ankle_roll_joint'].values, fs) for sd in ['left', 'right']])
    zp = np.mean([zcr(df[f'vel_{sd}_ankle_pitch_joint'].values, fs) for sd in ['left', 'right']])
    print(f'{tag:>8} {dr:>9.2f} {dp:>10.2f} {dh:>11.2f} {dk:>11.2f} {zr:>10.2f} {zp:>11.2f} {dr/GAIT:>12.2f}')

print()
print('=' * 118)
print('【C】分相抖动 + 触地（支撑相=phase 正向）')
print('=' * 118)
print(f'{"版本":>8} {"roll撑L":>9} {"roll撑R":>9} {"roll摆L":>9} {"roll摆R":>9} '
      f'{"力峰kN L/R":>13} {"撑相力std":>10} {"触地速率":>9}')
print('-' * 118)
for tag, p in VERS:
    df, fs = load(p)
    ph = df['phase_sin'].values
    cmd = df['cmd_linear_x'].values
    m = np.abs(cmd) > 0.05
    st_l, st_r = m & (ph > 0.3), m & (ph < -0.3)
    sw_l, sw_r = m & (ph < -0.3), m & (ph > 0.3)
    f_ = df['foot_force_l'].values
    st = np.where((f_[:-1] < 5) & (f_[1:] > 50))[0]
    sr = (np.abs(np.diff(df['vel_left_ankle_roll_joint'].values)) * fs)[st].mean() if len(st) > 3 else np.nan
    print(f'{tag:>8} {jit(df["vel_left_ankle_roll_joint"].values[st_l], fs):>9.0f} '
          f'{jit(df["vel_right_ankle_roll_joint"].values[st_r], fs):>9.0f} '
          f'{jit(df["vel_left_ankle_roll_joint"].values[sw_l], fs):>9.0f} '
          f'{jit(df["vel_right_ankle_roll_joint"].values[sw_r], fs):>9.0f} '
          f'{f_[m].max()/1000:>6.1f}/{df["foot_force_r"].values[m].max()/1000:<6.1f} '
          f'{np.std(f_[st_l])/1000:>10.2f} {sr:>9.0f}')

print()
print('=' * 118)
print('【D】执行层负担：跟踪误差 / 力矩 / 力矩抖动')
print('=' * 118)
print(f'{"版本":>8} {"roll err°":>10} {"pitch err°":>11} {"roll|τ|Nm":>11} {"pitch|τ|Nm":>12} '
      f'{"roll τ抖":>10} {"τ抖/|τ|":>9} {"err抖":>9}')
print('-' * 118)
for tag, p in VERS:
    df, fs = load(p)
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    o = []
    for j in ['ankle_roll', 'ankle_pitch']:
        e = np.mean([np.abs(df[f'pos_track_err_{sd}_{j}_joint'].values[m]).mean() for sd in ['left', 'right']]) * 180 / np.pi
        t = np.mean([np.abs(df[f'effort_{sd}_{j}_joint'].values[m]).mean() for sd in ['left', 'right']])
        tj = np.mean([jit(df[f'effort_{sd}_{j}_joint'].values[m], fs) for sd in ['left', 'right']])
        o.append((e, t, tj))
    ed = np.mean([jit(df[f'pos_track_err_{sd}_ankle_roll_joint'].values[m], fs) for sd in ['left', 'right']]) * 180 / np.pi
    print(f'{tag:>8} {o[0][0]:>10.2f} {o[1][0]:>11.2f} {o[0][1]:>11.2f} {o[1][1]:>12.2f} '
          f'{o[0][2]:>10.0f} {o[0][2]/max(o[0][1],1e-9):>9.1f} {ed:>9.0f}')
