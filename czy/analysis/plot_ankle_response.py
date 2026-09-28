# -*- coding: utf-8 -*-
"""exp2.0 踝关节响应分析图（指令 → 实际位置 → 力矩 全链路）
数据：czy/data/exp2.0/isaac_diag.csv（回放，100Hz，6 段速度阶梯各 5s）
输出：czy/data/exp2.0/ankle_response.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 100.0
CSV = 'czy/data/exp2.0/isaac_diag.csv'
OUT = 'czy/data/exp2.0/ankle_response.png'

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

# 稳态行走窗口：0.4 m/s 段（step 1000~1500）取 2.1s（=3 个步态周期 0.7s）
A, B = 1050, 1260
sl = slice(A, B)
tw = t[sl] - t[A]

DES = {'pitch_L': 'pos_des_left_ankle_pitch_joint', 'pitch_R': 'pos_des_right_ankle_pitch_joint',
       'roll_L': 'pos_des_left_ankle_roll_joint', 'roll_R': 'pos_des_right_ankle_roll_joint'}
POS = {'pitch_L': 'pos_left_ankle_pitch_joint', 'pitch_R': 'pos_right_ankle_pitch_joint',
       'roll_L': 'pos_left_ankle_roll_joint', 'roll_R': 'pos_right_ankle_roll_joint'}
TAU = {'pitch_L': 'effort_left_ankle_pitch_joint', 'pitch_R': 'effort_right_ankle_pitch_joint',
       'roll_L': 'effort_left_ankle_roll_joint', 'roll_R': 'effort_right_ankle_roll_joint'}
FORCE = {'L': 'foot_force_l', 'R': 'foot_force_r'}


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    return x - np.convolve(x, np.ones(w) / w, mode='same')


# ---------- 响应量化（全运动段）----------
m = np.abs(cmd) > 0.05
print('=== exp2.0 踝关节响应指标（运动段 cmd≠0）===')
hdr = (f'{"关节":>10} {"跟踪误差":>9} {"最大误差":>9} {"指令std":>8} {"实际std":>8} {"跟随比":>7} '
       f'{"力矩RMS":>8} {"高频颤振":>9} {"颤振/力矩":>9}')
print(hdr)
print('-' * 96)
rows_metric = {}
for jn in ['pitch', 'roll']:
    for sd in ['L', 'R']:
        k = f'{jn}_{sd}'
        des, pos, tau = df[DES[k]].values[m], df[POS[k]].values[m], df[TAU[k]].values[m]
        err = np.abs(des - pos)
        ratio = np.std(pos) / max(np.std(des), 1e-9)
        hfq = np.sqrt(np.mean(hp(tau) ** 2))
        rmsq = np.sqrt(np.mean(tau ** 2))
        rows_metric[k] = dict(err=np.degrees(err.mean()), ratio=ratio, rms=rmsq, hf=hfq)
        print(f'{jn + sd:>10} {np.degrees(err.mean()):>8.2f}° {np.degrees(err.max()):>8.2f}° '
              f'{np.std(des):>8.4f} {np.std(pos):>8.4f} {ratio:>7.2f} '
              f'{rmsq:>8.2f} {hfq:>9.2f} {hfq / max(rmsq, 1e-9) * 100:>8.1f}%')

# ---------- 绘图 ----------
fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
fig.suptitle('exp1.3t1', fontsize=16, y=0.995)
fig.text(0.5, 0.952, '踝关节响应：指令 → 实际位置 → 力矩（0.4 m/s 稳态行走，'
         '100Hz，步态周期 0.7s；灰色阴影 = 摆动相）', ha='center', fontsize=11.5)

order = [('pitch_L', 0, 0, '① 踝 pitch（左）'), ('pitch_R', 0, 1, '② 踝 pitch（右）'),
         ('roll_L', 1, 0, '③ 踝 roll（左）'), ('roll_R', 1, 1, '④ 踝 roll（右）')]

for k, r, c, title in order:
    ax = axes[r, c]
    sd = k[-1]
    ax.plot(tw, df[DES[k]].values[sl], color='#1f77b4', lw=1.5, label='指令 pos_des')
    ax.plot(tw, df[POS[k]].values[sl], color='#d62728', lw=1.5, label='实际 pos')
    # 摆动相阴影（足底力 < 5N），画在曲线之下
    swing = df[FORCE[sd]].values[sl] < 5
    ax.fill_between(tw, *ax.get_ylim(), where=swing,
                    color='0.85', alpha=0.6, step='mid', zorder=0)
    ax.set_ylabel('角度 (rad)', fontsize=10)
    ax.set_title(f'{title}    '
                 f'[误差 {rows_metric[k]["err"]:.1f}° | 跟随比 {rows_metric[k]["ratio"]:.2f} | '
                 f'τ高频 {rows_metric[k]["hf"]:.2f} Nm]', fontsize=10.5, loc='left')
    ax.grid(alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(tw, df[TAU[k]].values[sl], color='#ff7f0e', lw=1.0, alpha=0.85, label='力矩 τ')
    ax2.set_ylabel('力矩 (Nm)', fontsize=10, color='#ff7f0e')
    ax2.tick_params(axis='y', labelcolor='#ff7f0e')

    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=9, loc='upper right', ncol=3, framealpha=0.9)

for ax in axes[1]:
    ax.set_xlabel('时间 (s)', fontsize=10.5)

plt.tight_layout(rect=[0, 0, 1, 0.965])
plt.savefig(OUT, dpi=140, bbox_inches='tight')
print('\n已保存:', OUT, f'({os.path.getsize(OUT)/1024:.0f} KB)')
