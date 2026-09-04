# -*- coding: utf-8 -*-
"""步长左右不对称诊断：FK base系脚x位置 -> 迈步前后位移/摆幅；对比 1.6/1.7"""
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET

URDF = 'resources/robots/x1/urdf/X1_12DOF.urdf'
tree = ET.parse(URDF)
J = {}
for j in tree.getroot().findall('joint'):
    o, a = j.find('origin'), j.find('axis')
    J[j.get('name')] = dict(
        xyz=np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()]) if o is not None else np.zeros(3),
        rpy=[float(v) for v in (o.get('rpy') or '0 0 0').split()] if o is not None else [0, 0, 0],
        ax=np.array([float(v) for v in ((a.get('xyz') if a is not None else None) or '1 0 0').split()]),
        parent=j.find('parent').get('link'), child=j.find('child').get('link'))


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def ax_mat(ax, q):
    u = ax / np.linalg.norm(ax)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def build_chain(end):
    ch, jn = [], end
    while jn:
        ch.insert(0, jn)
        jn = next((k for k, v in J.items() if v['child'] == J[jn]['parent']), None)
    return ch


CHAIN = {s: build_chain(f'{s}_ankle_pitch_joint') for s in ['left', 'right']}
QJ = {s: [f'pos_{s}_hip_pitch_joint', f'pos_{s}_hip_roll_joint', f'pos_{s}_hip_yaw_joint',
          f'pos_{s}_knee_pitch_joint', f'pos_{s}_ankle_pitch_joint'] for s in ['left', 'right']}


def fk_foot_x(df, side):
    qs = [df[c].values for c in QJ[side]]
    n = len(df)
    x = np.empty(n)
    for i in range(n):
        p, R = np.zeros(3), np.eye(3)
        for jn, q in zip(CHAIN[side], qs):
            p = p + R @ J[jn]['xyz']
            R = R @ rpy_mat(*J[jn]['rpy']) @ ax_mat(J[jn]['ax'], q[i])
        x[i] = p[0]
    return x


def segs_of(mask, min_len=5):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if j - i >= min_len:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def diagnose(csv, tag):
    df = pd.read_csv(csv, encoding='utf-8-sig')
    ph = df['phase_sin'].values
    l_sw = (df['foot_force_l'].values < 5)[ph > 0.3].mean() if 'foot_force_l' in df else 0
    # 相位约定：sin>0=R摆动（1.6/1.7 均力标定为 +1）
    swL, swR = ph < -0.3, ph > 0.3
    print(f"\n{'='*74}\n== {tag}")
    fx = {s: fk_foot_x(df, s) for s in ['left', 'right']}
    res = {}
    for s, sw in [('L', swL), ('R', swR)]:
        side = 'left' if s == 'L' else 'right'
        segs = segs_of(sw)
        # 剔除站立段（摆动段内 base 位移近 0）
        amp_reach, amp_back, body_adv, excurs = [], [], [], []
        for a, b in segs:
            segx = fx[side][a:b + 1]
            base_adv = df['base_pos_x'].values[min(b, len(df)-1)] - df['base_pos_x'].values[a]
            if abs(base_adv) < 0.02:  # 站立/低速段剔除
                continue
            excurs.append(segx.max() - segx.min())
            # 迈步前(段首≈toe-off 在后) 与 迈步后(段尾≈heel-strike 在前)
            amp_back.append(segx[0])   # 起摆位置（应负=身后）
            amp_reach.append(segx[-1])  # 落脚位置（应正=身前）
        e, r_, b_ = np.array(excurs)*1000, np.array(amp_reach)*1000, np.array(amp_back)*1000
        # 世界系单步位移 ≈ body advance during swing + (reach - back)
        step_len = b_ + e if False else np.array(
            [ (df['base_pos_x'].values[min(bb, len(df)-1)] - df['base_pos_x'].values[aa]) * 1000
              + (fx[side][min(bb, len(df)-1)] - fx[side][aa]) * 1000
              for aa, bb in segs_of(sw)
              if abs(df['base_pos_x'].values[min(bb, len(df)-1)] - df['base_pos_x'].values[aa]) >= 0.02])
        res[s] = dict(exc=e, reach=r_, back=b_, step=step_len)
        print(f"  {s}: n={len(e)}")
        print(f"    FK脚x摆幅(excursion): median={np.median(e):.1f}mm p25={np.percentile(e,25):.1f} p75={np.percentile(e,75):.1f}")
        print(f"    落脚(身前) x: median={np.median(r_):+.1f}mm   起摆(身后) x: median={np.median(b_):+.1f}mm")
        print(f"    世界系单步位移: median={np.median(step_len):.1f}mm p25={np.percentile(step_len,25):.1f} p75={np.percentile(step_len,75):.1f}")
    # 对称比
    if len(res['L']['step']) and len(res['R']['step']):
        r2l = np.median(res['R']['step']) / max(np.median(res['L']['step']), 1e-6) * 100
        print(f"  >>> 步长对称比 R/L = {r2l:.0f}%   差 = {np.median(res['R']['step'])-np.median(res['L']['step']):+.1f}mm")
        r2l_x = np.median(res['R']['exc']) / max(np.median(res['L']['exc']), 1e-6) * 100
        print(f"  >>> 摆幅对称比 R/L = {r2l_x:.0f}%   差 = {np.median(res['R']['exc'])-np.median(res['L']['exc']):+.1f}mm")
    # 关节摆幅（hip_pitch/knee）与跟踪误差
    for s, sw in [('L', swL), ('R', swR)]:
        side = 'left' if s == 'L' else 'right'
        hp = df[f'pos_{side}_hip_pitch_joint'].values
        kn = df[f'pos_{side}_knee_pitch_joint'].values
        hp_amp = [hp[a:b+1].max() - hp[a:b+1].min() for a, b in segs_of(sw)
                  if abs(df['base_pos_x'].values[min(b, len(df)-1)] - df['base_pos_x'].values[a]) >= 0.02]
        kn_amp = [kn[a:b+1].max() - kn[a:b+1].min() for a, b in segs_of(sw)
                  if abs(df['base_pos_x'].values[min(b, len(df)-1)] - df['base_pos_x'].values[a]) >= 0.02]
        te_hp = df[f'pos_track_err_{side}_hip_pitch_joint'].values
        te_kn = df[f'pos_track_err_{side}_knee_pitch_joint'].values
        print(f"  {s} 关节: hip_pitch摆幅={np.median(hp_amp)*180/np.pi:.1f}°  knee摆幅={np.median(kn_amp)*180/np.pi:.1f}°  "
              f"track_err rms: hip={np.sqrt((te_hp**2).mean())*180/np.pi:.1f}° knee={np.sqrt((te_kn**2).mean())*180/np.pi:.1f}°")


diagnose('czy/data/exp_ada_1.7/isaac_diag.csv', 'exp_ada_1.7（用户观察：左步小右步大）')
diagnose('czy/data/exp_ada_1.6/isaac_diag.csv', 'exp_ada_1.6（对照）')
