# -*- coding: utf-8 -*-
"""exp_ada_1.9 恶化深度归因：+yaw 弧线（yaw +14.7~+19.9°/段、世界净侧移 +56.5mm/步）
五个问题：
  Q1 世界系侧移是弧线几何投影还是基座系真侧向步进？（把落脚位移转回机体系分解）
  Q2 yaw_rate 时间形态：恒定连续转向 vs 步事件脉冲？（决定惩罚项设计：速率级 vs 落脚级）
  Q3 转弯驱动源：hip_yaw 差动 / 髋 roll 侧倾 / 左右步长差（力偶）？
  Q4 lat_vel -1.0 是否真的压小了机体系 vy？（机制确认）
  Q5 弧线从段头就有还是渐起？（起步 vs 稳态行为）
"""
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
    cols = [f'pos_{side}_hip_pitch_joint',
            f'pos_{side}_hip_roll_joint', f'pos_{side}_hip_yaw_joint',
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


def segs_of(df):
    cmd = df['cmd_linear_x'].values
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a > 100:
            out.append((a, b, cmd[a]))
    return out


def diagnose(csv, tag):
    df = pd.read_csv(csv, encoding='utf-8-sig')
    yaw = np.unwrap(df['base_yaw'].values)
    bpx, bpy = df['base_pos_x'].values, df['base_pos_y'].values
    cmd = df['cmd_linear_x'].values
    fwd = np.abs(cmd) > 0.05
    ph = df['phase_sin'].values
    print(f"\n{'='*74}\n== {tag}")

    # Q4: 机体系 vy（行进段稳态）
    for a, b, c in segs_of(df):
        if abs(c) > 0.05:
            ss = slice(a + int((b - a) * 0.4), b)
            vy = df['base_vel_y'].values[ss]
            wx = df['base_vel_x'].values[ss]
            wz = df['base_ang_vel_z'].values[ss]
            print(f"  段 cmd={c:+.1f}: 机体系 vy median={np.median(vy)*100:+.1f}cm/s  "
                  f"|vy|={np.median(np.abs(vy))*100:.1f}  vx={np.median(wx):+.3f}m/s  "
                  f"yaw_rate={np.median(wz)*180/np.pi:+.2f}°/s")

    # Q2/Q5: 每段 yaw 轨迹线性度 + 首尾斜率
    for a, b, c in segs_of(df):
        y = yaw[a:b] - yaw[a]
        t = np.arange(b - a) / 50.0
        k, ic = np.polyfit(t, y, 1)
        r2 = 1 - np.sum((y - (k * t + ic)) ** 2) / max(np.sum((y - y.mean()) ** 2), 1e-12)
        h = (b - a) // 2
        k1 = np.polyfit(np.arange(h) / 50.0, y[:h], 1)[0] * 180 / np.pi
        k2 = np.polyfit(np.arange(h) / 50.0, y[h:], 1)[0] * 180 / np.pi
        print(f"  段 cmd={c:+.1f}: yaw总={k*t[-1]*180/np.pi:+6.1f}°  线性R2={r2:.2f}  "
              f"前半斜率={k1:+.2f}°/s 后半={k2:+.2f}°/s")

    # Q1: 落脚点位移 → 机体系分解（同侧相邻落点对）
    fl = df['foot_force_l'].values
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
                    lnd.append((k, side[0].upper(), wx[k], wy[k]))
                i = j
            else:
                i += 1
    lnd.sort(key=lambda t: t[0])
    body_f, body_s, sides = [], [], []
    for i in range(2, len(lnd)):
        if lnd[i][1] == lnd[i - 2][1]:
            k = lnd[i][0]
            ym = yaw[k]
            dx, dy = lnd[i][2] - lnd[i - 2][2], lnd[i][3] - lnd[i - 2][3]
            body_f.append(np.cos(ym) * dx + np.sin(ym) * dy)   # 沿航向前向
            body_s.append(-np.sin(ym) * dx + np.cos(ym) * dy)  # 航向左侧为正
            sides.append(lnd[i][1])
    body_f, body_s, sides = map(np.array, (body_f, body_s, sides))
    mv = np.abs(body_f) > 0.02
    for s in ['L', 'R']:
        m = mv & (sides == s)
        if m.sum():
            print(f"  Q1 同侧落点位移(机体系) {s}: n={m.sum()} 前向(步长)={np.median(body_f[m])*1000:.0f}mm  "
                  f"侧向={np.median(body_s[m])*1000:+.1f}mm  |侧向|p25={np.percentile(np.abs(body_s[m]),25)*1000:.1f}mm")

    # Q3: 转弯驱动源（行进段稳态）
    for a, b, c in segs_of(df):
        if abs(c) > 0.05:
            ss = slice(a + int((b - a) * 0.4), b)
            hyd = (df['pos_des_left_hip_yaw_joint'].values - df['pos_des_right_hip_yaw_joint'].values)[ss]
            rolld = (df['pos_left_hip_roll_joint'].values - df['pos_right_hip_roll_joint'].values)[ss]
            print(f"  Q3 段 cmd={c:+.1f}: hip_yaw_des L-R median={np.median(hyd)*180/np.pi:+.2f}°  "
                  f"hip_roll L-R median={np.median(rolld)*180/np.pi:+.2f}°")

    # 落脚点基座系前后错位（左右步长差力偶）
    fbx = []
    fsd = []
    for i in range(len(lnd)):
        k = lnd[i][0]
        ym = yaw[k]
        dx, dy = lnd[i][2] - bpx[k], lnd[i][3] - bpy[k]
        fbx.append(np.cos(ym) * dx + np.sin(ym) * dy)
        fsd.append(-np.sin(ym) * dx + np.cos(ym) * dy)
    fbx, fsd, fsd_ = np.array(fbx), np.array(fsd), np.array([s for s in sides] + [''] * 0)
    sd2 = np.array([lnd[i][1] for i in range(len(lnd))])
    for s in ['L', 'R']:
        m = (sd2 == s) & (np.abs(fbx) > 0.02)
        if m.sum():
            print(f"  Q3 落脚点(基座系) {s}: 前向x median={np.median(fbx[m])*1000:+.1f}mm  侧向y median={np.median(fsd[m])*1000:+.1f}mm")

if __name__ == '__main__':
    diagnose('czy/data/exp_ada_1.10/isaac_diag.csv', 'exp_ada_1.10')
    diagnose('czy/data/exp_ada_1.9/isaac_diag.csv', 'exp_ada_1.9')
    diagnose('czy/data/exp_ada_1.8/isaac_diag.csv', 'exp_ada_1.8')
