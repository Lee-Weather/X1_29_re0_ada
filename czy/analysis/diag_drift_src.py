# -*- coding: utf-8 -*-
"""落脚点侧向不对称诊断（exp_ada_1.9 验收复用）：每步净侧移 = 同侧相邻落点世界y差"""
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET

tree = ET.parse('resources/robots/x1/urdf/X1_12DOF.urdf')
J = {}
for j in tree.getroot().findall('joint'):
    o, a = j.find('origin'), j.find('axis')
    J[j.get('name')] = dict(
        xyz=np.array([float(v) for v in ((o.get('xyz') if o is not None else None) or '0 0 0').split()]),
        rpy=[float(v) for v in ((o.get('rpy') if o is not None else None) or '0 0 0').split()],
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


def build(end):
    ch, jn = [], end
    while jn:
        ch.insert(0, jn)
        jn = next((k for k, v in J.items() if v['child'] == J[jn]['parent']), None)
    return ch


def fk_xy(df, side):
    ch = build(f'{side}_ankle_pitch_joint')
    cols = [f'pos_{side}_hip_pitch_joint', f'pos_{side}_hip_roll_joint', f'pos_{side}_hip_yaw_joint',
            f'pos_{side}_knee_pitch_joint', f'pos_{side}_ankle_pitch_joint']
    qs = [df[c].values for c in cols]
    out = np.empty((len(df), 2))
    for i in range(len(df)):
        p, R = np.zeros(3), np.eye(3)
        for jn, q in zip(ch, qs):
            p = p + R @ J[jn]['xyz']
            R = R @ rpy_mat(*J[jn]['rpy']) @ ax_mat(J[jn]['ax'], q[i])
        out[i] = p[:2]
    return out


def diagnose(csv, tag):
    df = pd.read_csv(csv, encoding='utf-8-sig')
    ph = df['phase_sin'].values
    fl, fr = df['foot_force_l'].values, df['foot_force_r'].values
    bpx, bpy, yaw = df['base_pos_x'].values, df['base_pos_y'].values, df['base_yaw'].values
    cmd = df['cmd_linear_x'].values
    fwd = np.abs(cmd) > 0.05
    lnd = []
    for side in ['left', 'right']:
        pxy = fk_xy(df, side)
        cy_, sy_ = np.cos(yaw), np.sin(yaw)
        wx = cy_ * pxy[:, 0] - sy_ * pxy[:, 1] + bpx
        wy = sy_ * pxy[:, 0] + cy_ * pxy[:, 1] + bpy
        sw = (ph < -0.3) if side == 'left' else (ph > 0.3)
        i, n = 0, len(sw)
        while i < n:
            if sw[i] and fwd[i]:
                j = i
                while j < n and sw[j]:
                    j += 1
                if j - i >= 5:
                    k = min(j, n - 1)
                    lnd.append((k, side[0].upper(), wy[k], wx[k]))
                i = j
            else:
                i += 1
    lnd.sort(key=lambda t: t[0])
    same = [ (lnd[i][2] - lnd[i-2][2]) * 1000 for i in range(2, len(lnd)) if lnd[i][1] == lnd[i-2][1] ]
    same = np.array(same)
    widths = np.array([ (lnd[i][2] - lnd[i-1][2]) * 1000 for i in range(1, len(lnd)) ])
    print(f"== {tag} 落脚点侧向 ==")
    print(f"  同侧相邻落点y差(每步净侧移): mean={same.mean():+.1f}mm median={np.median(same):+.1f}mm")
    print(f"  相邻落点y差(步宽交替性): |median|={np.median(np.abs(widths)):.1f}mm signed mean={widths.mean():+.1f}mm")


if __name__ == '__main__':
    diagnose('czy/data/exp_ada_1.9/isaac_diag.csv', 'exp_ada_1.9')
    diagnose('czy/data/exp_ada_1.8/isaac_diag.csv', 'exp_ada_1.8')
    diagnose('czy/data/exp_ada_1.6/isaac_diag.csv', 'exp_ada_1.6')
