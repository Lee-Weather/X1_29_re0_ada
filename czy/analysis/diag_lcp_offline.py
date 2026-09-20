# -*- coding: utf-8 -*-
"""exp_ada_1.11 离线 LCP 诊断分析：策略 Lipschitz 常数跨版本对比（1.8 / 1.9 / 1.10）

背景：LCP（Lipschitz-Constrained Policies）在 actor loss 上加 w·E‖∂logπ/∂s‖² 让策略变平滑。
     加入前需先验证「策略确实陡峭」——本脚本用回放数据测量该量级。

指标（s = actor 输入，302 维 = 235 短历史 + 64 CNN + 3 状态估计）：
  lcp_jac_frob   = ‖∂μ/∂s‖_F，12×302 雅可比 Frobenius 范数（策略陡峭度，确定性口径）
  lcp_frac_short = 梯度能量落在短历史段(235 维)的占比（敏感度是否集中在传感侧）
  lcp_sigma_w    = σ 加权范数 sqrt(Σ_a ‖J_a‖²/σ_a²)，≈ MimicKit LCP 原式量级
"""
import numpy as np
import pandas as pd

FS = 50.0
VERS = [('1.8', 'czy/data/lcp_diag/diag_1.8.csv'),
        ('1.9', 'czy/data/lcp_diag/diag_1.9.csv'),
        ('1.10', 'czy/data/lcp_diag/diag_1.10.csv')]

LCP_COLS = ['lcp_jac_frob', 'lcp_frac_short', 'lcp_sigma_w']


def segments(df):
    cmd = df['cmd_linear_x'].values
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    return [(a, b, cmd[a]) for a, b in zip(edges[:-1], edges[1:]) if b - a > 100]


def jitter(df, joint, side='left', m=None):
    v = df[f'vel_{side}_{joint}_joint'].values
    if m is not None:
        v = v[m]
    return float(np.std(np.diff(v)) * FS)


print('=' * 96)
print('一、总体统计（全步）')
print(f'{"版本":>6} {"jac_frob中位":>12} {"jac_frob均值":>12} {"jac_frob p90":>12} '
      f'{"短历史占比":>11} {"σ加权中位":>11} {"n步":>6}')
print('-' * 96)
store = {}
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    miss = [c for c in LCP_COLS if c not in df.columns]
    if miss:
        print(f'{tag:>6}  缺列 {miss}；实际 lcp 列: {[c for c in df.columns if c.startswith("lcp")]}')
        continue
    f = df['lcp_jac_frob'].values
    store[tag] = df
    print(f'{tag:>6} {np.median(f):>12.3f} {f.mean():>12.3f} {np.percentile(f, 90):>12.3f} '
          f'{np.median(df["lcp_frac_short"].values):>11.3f} '
          f'{np.median(df["lcp_sigma_w"].values):>11.3f} {len(df):>6}')

if not store:
    raise SystemExit('无可用数据')

print()
print('=' * 96)
print('二、分段对比（cmd 档位；yaw_rate 与弧线对照）')
print(f'{"版本":>6} {"段cmd":>7} {"yaw_rate°/s":>11} {"jac_frob中位":>12} {"jac_frob p90":>12} '
      f'{"短历史占比":>11} {"σ加权中位":>11}')
print('-' * 96)
for tag, df in store.items():
    for a, b, c in segments(df):
        sl = slice(a, b)
        f = df['lcp_jac_frob'].values[sl]
        wz = np.median(df['base_ang_vel_z'].values[a + int((b - a) * 0.4):b]) * 180 / np.pi
        print(f'{tag:>6} {c:>7.2f} {wz:>11.2f} {np.median(f):>12.3f} {np.percentile(f, 90):>12.3f} '
              f'{np.median(df["lcp_frac_short"].values[sl]):>11.3f} '
              f'{np.median(df["lcp_sigma_w"].values[sl]):>11.3f}')

print()
print('=' * 96)
print('三、与关节抖动的相关性（运动段，踝/膝 vel 差分 std×FS）')
print(f'{"版本":>6} {"踝roll抖L":>10} {"踝roll抖R":>10} {"膝抖L":>9} {"膝抖R":>9} '
      f'{"jac_frob中位":>12} {"corr(jac,踝roll抖L)":>18}')
print('-' * 96)
for tag, df in store.items():
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    f = df['lcp_jac_frob'].values
    jl = jitter(df, 'ankle_roll', 'left', m)
    jr = jitter(df, 'ankle_roll', 'right', m)
    kl = jitter(df, 'knee_pitch', 'left', m)
    kr = jitter(df, 'knee_pitch', 'right', m)
    # 逐窗相关：把运动段按 50 步切窗，窗内 jac 中位 vs 窗内踝抖
    xs, ys = [], []
    for a, b, c in segments(df):
        if abs(c) <= 0.05:
            continue
        for s in range(a, b - 50, 50):
            w = slice(s, s + 50)
            xs.append(np.median(f[w]))
            ys.append(np.std(np.diff(df['vel_left_ankle_roll_joint'].values[w])) * FS)
    corr = np.corrcoef(xs, ys)[0, 1] if len(xs) > 3 else np.nan
    print(f'{tag:>6} {jl:>10.1f} {jr:>10.1f} {kl:>9.1f} {kr:>9.1f} '
          f'{np.median(f[m]):>12.3f} {corr:>18.2f}')

print()
print('=' * 96)
print('四、版本间比值（以 1.8 为基准，判断失败版本是否更陡）')
if '1.8' in store:
    base = np.median(store['1.8']['lcp_jac_frob'].values)
    for tag, df in store.items():
        v = np.median(df['lcp_jac_frob'].values)
        print(f'  {tag:>5}: jac_frob 中位 = {v:.3f}  相对 1.8 = {v / base * 100:.0f}%')
else:
    print('  (缺 1.8 基准)')
print('=' * 96)
