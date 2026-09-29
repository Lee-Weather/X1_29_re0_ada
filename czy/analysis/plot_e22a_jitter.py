# -*- coding: utf-8 -*-
"""exp2.2a「视频抖动」诊断图：9.1Hz 振荡来自策略输出（action），经 PD 变为力矩
输出：czy/data/exp2.2a/jitter_diagnosis.png
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 100.0
A, B = 1150, 1300          # 0.4 m/s 段内 1.5s 窗口
for cand in ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']:
    try:
        matplotlib.font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams['font.sans-serif'] = [cand]
        break
    except Exception:
        continue
plt.rcParams['axes.unicode_minus'] = False

d = {t: pd.read_csv(f'czy/data/{t}/isaac_diag.csv', encoding='utf-8-sig')
     for t in ['exp2.0', 'exp2.2a']}
t0 = d['exp2.0']['time_s'].values[A]
tw = d['exp2.0']['time_s'].values[A:B] - t0
C = {'exp2.0': ('#7f7f7f', 'exp2.0（基线）'), 'exp2.2a': ('#d62728', 'exp2.2a（LCP 3e-5）')}

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

panels = [('action_left_ankle_pitch_joint', '策略 action（踝 pitch）',
           '① 策略输出：exp2.2a 在 9.1 Hz 上持续振荡（exp2.0 平滑）'),
          ('effort_left_ankle_pitch_joint', '力矩 (Nm)', None),
          ('effort_left_knee_pitch_joint', '力矩 (Nm)', None)]
titles = ['① 策略输出 action（踝 pitch）：exp2.2a 含强 9.1 Hz 振荡，>5Hz 能量 67%（exp2.0 仅 7.5%）',
          '② 踝 pitch 力矩：振荡经 PD 原样传出，>5Hz 能量 12.3% → 46.9%',
          '③ 膝 pitch 力矩：受害最重，>5Hz 能量 36.7% → 91.5%（视频中最显眼的抖动）']
ylabs = ['action', '力矩 (Nm)', '力矩 (Nm)']

for ax, (col, ylab, _), title in zip(axes, panels, titles):
    for tag in ['exp2.0', 'exp2.2a']:
        c, lb = C[tag]
        ax.plot(tw, d[tag][col].values[A:B], color=c, lw=1.4 if tag == 'exp2.2a' else 1.6,
                alpha=0.95 if tag == 'exp2.2a' else 0.75, label=lb, zorder=3 if tag == 'exp2.2a' else 2)
    ax.set_ylabel(ylab, fontsize=10)
    ax.set_title(title, fontsize=10.5, loc='left')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc='upper right', ncol=2)

axes[-1].set_xlabel('时间 (s)（0.4 m/s 稳态行走窗口）', fontsize=10.5)
fig.suptitle('exp1.3t1', fontsize=16, y=0.995)
fig.text(0.5, 0.955, 'exp2.2a 抖动溯源：9.1 Hz 振荡源自策略输出，而非机械共振',
         ha='center', fontsize=11.5)

plt.tight_layout(rect=[0, 0, 1, 0.945])
OUT = 'czy/data/exp2.2a/jitter_diagnosis.png'
plt.savefig(OUT, dpi=140, bbox_inches='tight')
print('已保存:', OUT, f'({os.path.getsize(OUT)/1024:.0f} KB)')
