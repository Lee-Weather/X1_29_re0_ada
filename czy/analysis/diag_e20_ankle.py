# -*- coding: utf-8 -*-
"""踝关节抖动专项诊断（exp2.0 vs 1.11 vs 1.11l，并回溯 1.6~1.10 趋势）

核心问题：踝抖动（尤其 ankle_roll）到底是什么性质？
  Q1 是高频振荡（chatter）还是步态基频？
  Q2 是策略主动输出（action 抖）还是物理/接触结果（vel 抖但 action 平滑）？
  Q3 发生在摆动相还是支撑相？触地瞬间是否有冲击尖峰？
  Q4 与跟踪误差的关系（PD 增益不足 vs 参考轨迹本身抖）？
  Q5 与 LCP 平滑度的关系（exp2.0 jac -55% 后踝抖为何只降 30%？）
"""
import glob
import os
import numpy as np
import pandas as pd

FS = 50.0
CYCLE = 0.7

CAND = [
    ('1.6',  'czy/data/exp_ada_1.6/isaac_diag.csv'),
    ('1.7',  'czy/data/exp_ada_1.7/isaac_diag.csv'),
    ('1.8',  'czy/data/exp_ada_1.8/isaac_diag.csv'),
    ('1.9',  'czy/data/exp_ada_1.9/isaac_diag.csv'),
    ('1.10', 'czy/data/exp_ada_1.10/isaac_diag.csv'),
    ('1.11', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
    ('1.11l','czy/data/exp_ada_1.11l/isaac_diag.csv'),
    ('exp2.0','czy/data/exp2.0/isaac_diag.csv'),
]
VERS = []
for tag, p in CAND:
    if os.path.exists(p):
        VERS.append((tag, p))
    else:
        for alt in glob.glob(p.replace('isaac_diag.csv', '*.csv')):
            VERS.append((tag, alt))
            break
print('可用版本:', [t for t, _ in VERS])


def load(p):
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    return df, m


def jit(x, mask):
    """抖动：运动段一阶差分 std × FS"""
    return np.std(np.diff(x[mask])) * FS


def hf_ratio(x, mask, fc=5.0):
    """高频能量占比（>fc Hz）"""
    s = x[mask]
    s = s - s.mean()
    if len(s) < 256:
        return np.nan
    w = np.hanning(len(s))
    P = np.abs(np.fft.rfft(s * w)) ** 2
    f = np.fft.rfftfreq(len(s), 1 / FS)
    return P[f > fc].sum() / max(P.sum(), 1e-12)


def dom_freq(x, mask):
    s = x[mask]
    s = s - s.mean()
    if len(s) < 256:
        return np.nan
    w = np.hanning(len(s))
    P = np.abs(np.fft.rfft(s * w)) ** 2
    f = np.fft.rfftfreq(len(s), 1 / FS)
    k = np.argmax(P[1:]) + 1
    return f[k]


print()
print('=' * 112)
print('Q1/Q2：踝关节抖动分解（运动段，L/R）——vel 抖动 vs action 抖动，判断驱动源')
print('=' * 112)
print(f'{"版本":>8} {"ankle_pitch vel":>17} {"ankle_roll vel":>17} {"ankle_pitch act":>17} '
      f'{"ankle_roll act":>17} {"roll动作/速度比":>14}')
print('-' * 112)
for tag, p in VERS:
    df, m = load(p)
    r = {}
    for j in ['ankle_pitch', 'ankle_roll']:
        v = np.mean([jit(df[f'vel_left_{j}_joint'].values, m), jit(df[f'vel_right_{j}_joint'].values, m)])
        a = np.mean([jit(df[f'action_left_{j}_joint'].values, m), jit(df[f'action_right_{j}_joint'].values, m)])
        r[j] = (v, a)
    ratio = r['ankle_roll'][1] / max(r['ankle_roll'][0], 1e-9)
    print(f'{tag:>8} {r["ankle_pitch"][0]:>17.1f} {r["ankle_roll"][0]:>17.1f} '
          f'{r["ankle_pitch"][1]:>17.4f} {r["ankle_roll"][1]:>17.4f} {ratio:>14.2e}')

print()
print('=' * 112)
print('Q1：频谱性质——主频 & 高频(>5Hz)能量占比  [步态基频 = 1/0.7 = 1.43 Hz]')
print('=' * 112)
print(f'{"版本":>8} {"roll主频Hz":>11} {"roll高频占比":>13} {"pitch主频Hz":>12} {"pitch高频占比":>14} '
      f'{"髋pitch主频":>11} {"膝pitch主频":>12}')
print('-' * 112)
for tag, p in VERS:
    df, m = load(p)
    def stat(j):
        f = np.mean([dom_freq(df[f'vel_left_{j}_joint'].values, m), dom_freq(df[f'vel_right_{j}_joint'].values, m)])
        h = np.mean([hf_ratio(df[f'vel_left_{j}_joint'].values, m), hf_ratio(df[f'vel_right_{j}_joint'].values, m)])
        return f, h
    fr, hr = stat('ankle_roll')
    fp, hp = stat('ankle_pitch')
    fh, _ = stat('hip_pitch')
    fk, _ = stat('knee_pitch')
    print(f'{tag:>8} {fr:>11.2f} {hr*100:>12.1f}% {fp:>12.2f} {hp*100:>13.1f}% '
          f'{fh:>11.2f} {fk:>12.2f}')

print()
print('=' * 112)
print('Q3：分相抖动（摆动=抬脚期 phase 反向 / 支撑=落地期）+ 触地冲击')
print('=' * 112)
print(f'{"版本":>8} {"roll摆/撑L":>13} {"roll摆/撑R":>13} {"pitch摆/撑L":>14} {"pitch摆/撑R":>14} '
      f'{"触地力峰kN L/R":>15} {"触地时roll速率":>13}')
print('-' * 112)
for tag, p in VERS:
    df, m = load(p)
    ph = df['phase_sin'].values
    out = []
    for side, sw in [('l', ph < -0.3), ('r', ph > 0.3)]:
        for j in ['ankle_roll', 'ankle_pitch']:
            v = df[f'vel_{"left" if side=="l" else "right"}_{j}_joint'].values
            swm = sw & m
            stm = (~sw) & m
            out.append(v[swm].std() * FS / max(v[stm].std() * FS, 1e-9))
    fl = df['foot_force_l'].values[m].max() / 1000
    fr_ = df['foot_force_r'].values[m].max() / 1000
    # 触地瞬间（力从 <5 跳到 >50）的 roll 速率
    rate = np.abs(np.diff(df['vel_left_ankle_roll_joint'].values)) * FS
    force = df['foot_force_l'].values
    strike = np.where((force[:-1] < 5) & (force[1:] > 50))[0]
    strike = strike[m[:-1][strike]] if len(strike) else strike
    sr = rate[strike].mean() if len(strike) > 3 else np.nan
    print(f'{tag:>8} {out[0]:>13.2f} {out[2]:>13.2f} {out[1]:>14.2f} {out[3]:>14.2f} '
          f'{fl:>7.1f}/{fr_:<7.1f} {sr:>13.0f}')

print()
print('=' * 112)
print('Q4：踝关节跟踪误差 & 力矩（判断 PD 增益/执行能力）')
print('=' * 112)
print(f'{"版本":>8} {"roll跟踪err°":>12} {"pitch跟踪err°":>13} {"roll力矩Nm":>11} {"pitch力矩Nm":>12} '
      f'{"roll力矩抖":>11} {"pitch力矩抖":>12}')
print('-' * 112)
for tag, p in VERS:
    df, m = load(p)
    r = {}
    for j in ['ankle_roll', 'ankle_pitch']:
        e = np.mean([np.abs(df[f'pos_track_err_left_{j}_joint'].values[m]).mean(),
                     np.abs(df[f'pos_track_err_right_{j}_joint'].values[m]).mean()]) * 180 / np.pi
        t = np.mean([np.abs(df[f'effort_left_{j}_joint'].values[m]).mean(),
                     np.abs(df[f'effort_right_{j}_joint'].values[m]).mean()])
        tj = np.mean([jit(df[f'effort_left_{j}_joint'].values, m), jit(df[f'effort_right_{j}_joint'].values, m)])
        r[j] = (e, t, tj)
    print(f'{tag:>8} {r["ankle_roll"][0]:>12.2f} {r["ankle_pitch"][0]:>13.2f} '
          f'{r["ankle_roll"][1]:>11.2f} {r["ankle_pitch"][1]:>12.2f} '
          f'{r["ankle_roll"][2]:>11.1f} {r["ankle_pitch"][2]:>12.1f}')

print()
print('=' * 112)
print('Q5：踝抖动 vs LCP 平滑度（jac_frob）——检验"LCP 是否真的治踝抖"')
print('=' * 112)
print(f'{"版本":>8} {"jac_frob":>9} {"roll vel抖":>11} {"roll/jac":>9} {"action全局抖":>12} {"roll动作抖":>11}')
print('-' * 112)
JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]
for tag, p in VERS:
    df, m = load(p)
    jf = np.median(df['lcp_jac_frob'].values) if 'lcp_jac_frob' in df.columns else np.nan
    rv = np.mean([jit(df['vel_left_ankle_roll_joint'].values, m), jit(df['vel_right_ankle_roll_joint'].values, m)])
    ga = np.mean([jit(df[f'action_{j}_joint'].values, m) for j in JOINTS])
    ra = np.mean([jit(df['action_left_ankle_roll_joint'].values, m), jit(df['action_right_ankle_roll_joint'].values, m)])
    print(f'{tag:>8} {jf:>9.3f} {rv:>11.1f} {rv/max(jf,1e-9):>9.2f} {ga:>12.4f} {ra:>11.4f}')
