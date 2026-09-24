# -*- coding: utf-8 -*-
"""离线诊断：踝关节指令(pos_des)的高频成分来自「参考轨迹」还是「策略」？
依据 compute_ref_state()（x1_dh_stand_env.py L274-321）离线重建 ref_dof_pos：
  sin_pos = phase_sin（play.py 记录的就是 sin(2π·phase)）
  step_scale  = clip(|vx_cmd|/0.6, 0.3, 1.2)
  ankle_scale = max(step_scale, 0.6)
  L: sp_l = min(sin_pos,0);  delta = sp_l * d[4] * ankle_scale   (|sin_pos|<0.1 时 → +0.05)
  R: sp_r = max(sin_pos,0);  delta = -sp_r * d[10] * ankle_scale (|sin_pos|<0.1 时 → +0.05)
  ref = default + delta        （default: ankle_pitch = -0.21, ankle_roll = 0.0）
  d[4]=d[10]=-0.16; d[5]=d[11]=0.0  → 踝 roll 参考恒为 0
pos_des = 0.5*action + default  → 「指令 delta」= 0.5*action
策略偏离 = 0.5*action - ref_delta
"""
import numpy as np
import pandas as pd

FS = 100.0
D_ANKLE_PITCH = -0.16
D_ANKLE_ROLL = 0.0
DEF_PITCH, DEF_ROLL = -0.21, 0.0
DS_BOUND = 0.1
DS_BIAS = 0.05
V_NOM, S_MIN, S_MAX = 0.6, 0.3, 1.2
ANKLE_S_MIN = 0.6


def hp(x, fc=5.0):
    w = max(3, int(FS / fc))
    k = np.ones(w) / w
    return x - np.convolve(x, k, mode='same')


def rms(x):
    return float(np.sqrt(np.mean(x ** 2)))


def ref_delta_ankle(sin_pos, vx, side):
    ss = np.clip(np.abs(vx) / V_NOM, S_MIN, S_MAX)
    asc = np.maximum(ss, ANKLE_S_MIN)
    sp = np.minimum(sin_pos, 0.0) if side == 'L' else np.maximum(sin_pos, 0.0)
    sgn = 1.0 if side == 'L' else -1.0
    d = sgn * sp * D_ANKLE_PITCH * asc
    d[np.abs(sin_pos) < DS_BOUND] = DS_BIAS
    return d


for tag, p in [('exp2.0', 'czy/data/exp2.0/isaac_diag.csv'),
               ('exp2.1p', 'czy/data/exp2.1p/isaac_diag.csv')]:
    df = pd.read_csv(p, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    sp = df['phase_sin'].values
    vx = df['cmd_linear_x'].values
    print(f'\n===== {tag} =====')
    for j, dflt in [('ankle_pitch', DEF_PITCH), ('ankle_roll', DEF_ROLL)]:
        rows = []
        for s, sd in [('left', 'L'), ('right', 'R')]:
            act = df[f'action_{s}_{j}_joint'].values
            cmd_delta = 0.5 * act                       # 指令 delta（= pos_des - default）
            ref_d = (ref_delta_ankle(sp, vx, sd) if j == 'ankle_pitch'
                     else np.zeros_like(sp))            # 踝 roll 参考恒 0
            pol_d = cmd_delta - ref_d                   # 策略相对参考的偏离
            rows.append((rms(hp(cmd_delta[m])), rms(hp(ref_d[m])), rms(hp(pol_d[m])),
                         np.std(ref_d[m]), np.std(pol_d[m])))
        a, b, c, d, e = [np.mean([r[i] for r in rows]) for i in range(5)]
        tot = a
        print(f'  {j:>12}: 指令高频={tot:.4f} rad | '
              f'参考高频={b:.4f} ({b/max(tot,1e-9)*100:5.1f}%) | '
              f'策略高频={c:.4f} ({c/max(tot,1e-9)*100:5.1f}%)')
        print(f'  {"":>12}  总幅值: 参考 std={d:.4f}  策略偏离 std={e:.4f} rad')

    # 参考轨迹的「跳变」直接验证：双支撑边界处的一步差分
    ss = np.clip(np.abs(vx) / V_NOM, S_MIN, S_MAX)
    asc = np.maximum(ss, ANKLE_S_MIN)
    sp_l = np.minimum(sp, 0.0)
    cont = sp_l * D_ANKLE_PITCH * asc              # 未施加 ds_mask 的连续段
    jump = np.abs(np.diff(cont[m]))[np.abs(np.diff(np.abs(sp[m]) < DS_BOUND)) > 0]
    print(f'  双支撑边界(|sin|<{DS_BOUND})处参考的一步跳变: '
          f'次数={len(jump)}  最大={jump.max() if len(jump) else 0:.4f} rad')
    print(f'  该跳变等价力矩冲击 = KP × 跳变 = 35 × {jump.max() if len(jump) else 0:.4f} '
          f'= {35*(jump.max() if len(jump) else 0):.2f} Nm')
