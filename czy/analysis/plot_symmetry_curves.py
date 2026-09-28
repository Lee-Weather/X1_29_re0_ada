# -*- coding: utf-8 -*-
"""exp2.0 左右对称性指标「曲线」（时间序列，非单值）
口径沿用项目既有约定（diag_sym_compare.py / _reward_gait_symmetry）：
  符号  S = sym_joint_sign = [-1,-1,-1,+1,+1,-1]  (hip_p/hip_r/hip_y/knee/ankle_p/ankle_r)
  相位对称误差  eps_j(t) = q_L,j(t) - S_j * q_R,j(t - T/2)，T/2 = 0.35s = 35 步
  幅值比        R/L ×100%（100% = 对称）
输出：czy/data/exp2.0/symmetry_curves.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 100.0
CYCLE = 0.7
HALF = int(CYCLE / 2 * FS)      # 35 步
WIN = int(CYCLE * FS)           # 滑窗 = 1 个步态周期 = 70 步
STEP = 5                        # 滑窗步进 0.05s
CSV = 'czy/data/exp2.0/isaac_diag.csv'
OUT = 'czy/data/exp2.0/symmetry_curves.png'

for cand in ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']:
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams['font.sans-serif'] = [cand]
        break
    except Exception:
        continue
plt.rcParams['axes.unicode_minus'] = False

df = pd.read_csv(CSV, encoding='utf-8-sig')
t = df['time_s'].values
cmd = df['cmd_linear_x'].values
ph = df['phase_sin'].values
m = np.abs(cmd) > 0.05

JOINTS = ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']
KEY = ['hip_pitch', 'knee_pitch', 'ankle_pitch', 'ankle_roll']
S = np.array([-1.0, -1.0, -1.0, 1.0, 1.0, -1.0])
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

QL = np.stack([df[f'pos_left_{j}_joint'].values for j in JOINTS], axis=1)
QR = np.stack([df[f'pos_right_{j}_joint'].values for j in JOINTS], axis=1)
TL = np.stack([np.abs(df[f'effort_left_{j}_joint'].values) for j in JOINTS], axis=1)
TR = np.stack([np.abs(df[f'effort_right_{j}_joint'].values) for j in JOINTS], axis=1)

# ---------- 相位对称误差 eps_j(t) ----------
QR_delay = np.roll(QR, HALF, axis=0)
QR_delay[:HALF] = QR[:HALF]
eps = (QL - S * QR_delay) * 180 / np.pi          # 度


def slide(x, fn, win=WIN):
    ts, ys = [], []
    for a in range(0, len(x) - win + 1, STEP):
        ts.append(t[a + win // 2])
        ys.append(fn(x[a:a + win]))
    return np.array(ts), np.array(ys)


ts_e, eps_rms = slide(eps, lambda w: np.sqrt((w ** 2).mean(axis=0)))          # [T,6]
ts_n, eps_norm = slide(eps, lambda w: np.sqrt((w ** 2).sum(axis=1).mean()))   # [T]
# 力矩比用 2 个步态周期窗口（1.4s），提高信噪比
ts_t, ratio_tau = slide(np.stack([TL, TR], axis=2),
                        lambda w: w[:, :, 1].mean(axis=0)
                        / np.maximum(w[:, :, 0].mean(axis=0), 1e-9) * 100,
                        win=2 * WIN)   # [T,6]

# ---------- 抬脚高度比（每周期一点，仅运动段）----------
fzl, fzr = df['foot_z_l'].values * 1000, df['foot_z_r'].values * 1000
edges = np.where((ph[:-1] > 0) & (ph[1:] <= 0))[0]
cyc_t, cyc_ratio, cyc_l, cyc_r = [], [], [], []
for a, b in zip(edges[:-1], edges[1:]):
    if b - a < WIN * 0.6 or not m[a:b].all():
        continue
    seg = slice(a, b)
    lsel, rsel = ph[seg] < -0.3, ph[seg] > 0.3
    if not lsel.any() or not rsel.any():
        continue
    lp, rp = fzl[seg][lsel].max(), fzr[seg][rsel].max()
    if lp <= 1.0:
        continue
    cyc_t.append(t[a + np.argmax(fzl[seg] * lsel)])
    cyc_l.append(lp); cyc_r.append(rp); cyc_ratio.append(rp / lp * 100)
cyc_t, cyc_ratio = np.array(cyc_t), np.array(cyc_ratio)


def m_at(ts):
    """按时间取运动段掩码（避免窗口索引与时间错位）"""
    idx = np.clip(np.round(ts * FS).astype(int), 0, len(m) - 1)
    return m[idx]


print('=== exp2.0 左右对称性指标（100% = 对称；仅运动段 cmd≠0）===')
print(f'{"指标":>16} {"中位":>9} {"P10":>9} {"P90":>9}')
print('-' * 48)
for tag, arr, mk in [('相位误差范数(°)', eps_norm, m_at(ts_n)),
                     ('力矩比 膝(%)', ratio_tau[:, 3], m_at(ts_t)),
                     ('力矩比 踝p(%)', ratio_tau[:, 4], m_at(ts_t)),
                     ('力矩比 踝r(%)', ratio_tau[:, 5], m_at(ts_t)),
                     ('抬脚比 R/L(%)', cyc_ratio, np.ones(len(cyc_ratio), bool))]:
    v = arr[mk]
    print(f'{tag:>16} {np.median(v):>9.2f} {np.percentile(v,10):>9.2f} {np.percentile(v,90):>9.2f}')
print()
for i, j in enumerate(JOINTS):
    e = eps_rms[m_at(ts_e), i]
    print(f'  相位误差 {j:>12}: 中位 {np.median(e):5.2f}°  P90 {np.percentile(e,90):5.2f}°')
print(f'  抬脚高度: L 中位 {np.median(cyc_l):.1f} mm / R 中位 {np.median(cyc_r):.1f} mm'
      f'（{len(cyc_l)} 个周期）')

# ---------- 绘图 ----------
fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

ax = axes[0]
for i, (j, c) in enumerate(zip(JOINTS, COLORS)):
    ax.plot(ts_e, eps_rms[:, i], lw=1.0, color=c, alpha=0.8, label=j)
ax.plot(ts_n, eps_norm, lw=2.4, color='k', label='6 关节范数')
ax.set_ylabel('相位对称误差 (°)', fontsize=10.5)
ax.set_title('① 相位对称误差  ‖q_L(t) − S·q_R(t−T/2)‖（滑窗 RMS，越接近 0 越对称）',
             fontsize=10.5, loc='left')
ax.legend(fontsize=9, ncol=4, loc='upper right')

ax = axes[1]
keep_t = m_at(ts_t)   # 站立段 |τ|→0，比值病态，故只画运动段
for j, c in zip(KEY, ['#1f77b4', '#d62728', '#9467bd', '#8c564b']):
    i = JOINTS.index(j)
    ax.plot(ts_t[keep_t], ratio_tau[keep_t, i], lw=1.5, color=c, alpha=0.9, label=j)
ax.axhline(100, color='k', ls='--', lw=1.2, label='完全对称 = 100%')
ax.set_ylabel('力矩比  R/L (%)', fontsize=10.5)
ax.set_title('② 力矩幅值对称性  |τ_R| / |τ_L|（滑窗均值比，窗口 2 个周期；仅运动段）',
             fontsize=10.5, loc='left')
ax.legend(fontsize=9, ncol=5, loc='upper right')
ax.set_ylim(40, 160)

ax = axes[2]
ax.plot(cyc_t, cyc_ratio, 'o-', color='#d62728', ms=4, lw=1.1, label='抬脚高度比 R/L')
ax.axhline(100, color='k', ls='--', lw=1.2, label='完全对称 = 100%')
ax.set_ylabel('抬脚比  R/L (%)', fontsize=10.5)
ax.set_xlabel('时间 (s)', fontsize=10.5)
ax.set_title('③ 抬脚高度对称性（每个步态周期一点，摆动相峰值比）', fontsize=10.5, loc='left')
ax.legend(fontsize=9, loc='upper right')
ax.set_ylim(0, 200)

# 站立段阴影 + 网格（画在最后，此时 ylim 已确定）
for ax in axes:
    ax.fill_between(t, *ax.get_ylim(), where=~m, color='0.88', alpha=0.7, step='mid', zorder=0)
    ax.grid(alpha=0.3, zorder=1)

fig.suptitle('exp1.3t1', fontsize=16, y=0.995)
fig.text(0.5, 0.962, '左右对称性指标曲线（100% = 完全对称；灰底 = 站立段 cmd=0）',
         ha='center', fontsize=11.5)

plt.tight_layout(rect=[0, 0, 1, 0.955])
plt.savefig(OUT, dpi=140, bbox_inches='tight')
print('\n已保存:', OUT, f'({os.path.getsize(OUT)/1024:.0f} KB)')
