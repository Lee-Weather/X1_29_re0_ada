# -*- coding: utf-8 -*-
"""抬脚高度 + 左右对称诊断 v3：URDF 链 FK + 双脚差分口径（免疫 base 运动）"""
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET

URDF = 'resources/robots/x1/urdf/X1_12DOF.urdf'

tree = ET.parse(URDF)
joints = {}
for j in tree.getroot().findall('joint'):
    name = j.get('name')
    o, a = j.find('origin'), j.find('axis')
    xyz = np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()]) if o is not None else np.zeros(3)
    rpy = np.array([float(v) for v in (o.get('rpy') or '0 0 0').split()]) if o is not None else np.zeros(3)
    axis = np.array([float(v) for v in (a.get('xyz') or '1 0 0').split()]) if a is not None else np.array([1, 0, 0])
    joints[name] = dict(parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                        xyz=xyz, rpy=rpy, axis=axis)


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_mat(axis, q):
    u = axis / np.linalg.norm(axis)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def fk_pos(chain, qmap):
    p, R = np.zeros(3), np.eye(3)
    for jn in chain:
        J = joints[jn]
        p = p + R @ J['xyz']
        R = R @ rpy_mat(*J['rpy']) @ axis_mat(J['axis'], qmap.get(jn, 0.0))
    return p


def build_chain(end_joint):
    chain, jn = [], end_joint
    while jn is not None:
        chain.insert(0, jn)
        parent = joints[jn]['parent']
        jn = next((k for k, v in joints.items() if v['child'] == parent), None)
    return chain


CHAIN = {s: build_chain(f'{s}_ankle_pitch_joint') for s in ['left', 'right']}
QDEF = {}
for s in ['left', 'right']:
    QDEF.update({f'{s}_hip_pitch_joint': 0.4 if s == 'left' else -0.4,
                 f'{s}_knee_pitch_joint': 0.49, f'{s}_ankle_pitch_joint': -0.21,
                 f'{s}_hip_roll_joint': 0.05 if s == 'left' else -0.05,
                 f'{s}_hip_yaw_joint': -0.31 if s == 'left' else 0.31,
                 f'{s}_ankle_roll_joint': 0.0})
Z0 = {s: fk_pos(CHAIN[s], QDEF)[2] for s in ['left', 'right']}
print(f"FK 基准高度: L={Z0['left']*1000:.2f}mm R={Z0['right']*1000:.2f}mm (镜像应相等)")
dzt = fk_pos(CHAIN['left'], {**QDEF, 'left_knee_pitch_joint': 0.69})[2] - Z0['left']
print(f"膝 0.49->0.69 自检: {dzt*1000:+.1f}mm")


def fk_z_series(df, side):
    qh = df[f'pos_{side}_hip_pitch_joint'].values
    qk = df[f'pos_{side}_knee_pitch_joint'].values
    qa = df[f'pos_{side}_ankle_pitch_joint'].values
    n = len(df)
    z = np.empty(n)
    for i in range(n):
        z[i] = fk_pos(CHAIN[side], {f'{side}_hip_pitch_joint': qh[i],
                                    f'{side}_knee_pitch_joint': qk[i],
                                    f'{side}_ankle_pitch_joint': qa[i]})[2] - Z0[side]
    return z


def steps_peaks(v, mask, min_len=5):
    peaks, segs, i, n = [], [], 0, len(v)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if j - i >= min_len:
                peaks.append(v[i:j].max())
                segs.append((i, j))
            i = j
        else:
            i += 1
    return np.array(peaks), segs


def diagnose(df, tag, has_force=False, phase_sign=1.0):
    print(f"\n{'='*72}\n== {tag}  rows={len(df)}")
    ph_raw = df['phase_sin'].values
    if has_force:
        l_sw = (df['foot_force_l'].values < 5)[ph_raw > 0.3].mean()
        r_sw = (df['foot_force_r'].values < 5)[ph_raw > 0.3].mean()
        phase_sign = 1.0 if r_sw > l_sw else -1.0
        print(f"相位标定(力): sin>0 时 R 摆动 {r_sw:.0%} / L 摆动 {l_sw:.0%} -> sign={phase_sign:+.0f}")
    ph = ph_raw * phase_sign  # 力标定: sin>0 => 右摆动, sin<0 => 左摆动

    zl = fk_z_series(df, 'left')
    zr = fk_z_series(df, 'right')
    zd = zl - zr  # 双脚踝世界高差（base 系差=世界系差）

    res = {}
    ds_mask = np.abs(ph) < 0.2  # 双支撑窗：双脚贴地，zd 为稳定零点基准
    for s in ['L', 'R']:
        # 摆动窗：sin<0 = 左摆动, sin>0 = 右摆动
        sw = ((ph < -0.3) if s == 'L' else (ph > 0.3))
        # 抬升信号：摆动脚相对支撑脚高差（L 摆动时 zd 上升）
        sig = zd if s == 'L' else -zd
        base = np.median(sig[ds_mask])  # 双支撑零点
        rel = sig - base
        peaks, segs = steps_peaks(rel, sw)
        res[s] = (peaks, rel, sw)
        if len(peaks):
            p_ = np.array(peaks) * 1000
            print(f"  {s}: 步数={len(p_)} 抬升峰值 median={np.median(p_):.1f}mm "
                  f"p25={np.percentile(p_,25):.1f} p75={np.percentile(p_,75):.1f} "
                  f"min={p_.min():.1f} max={p_.max():.1f}  拖地步(<10mm)={(p_<10).sum()}")
        else:
            print(f"  {s}: 无有效摆动段")
    pl_, pr_ = res['L'][0] * 1000, res['R'][0] * 1000
    if len(pl_) and len(pr_):
        k = min(len(pl_), len(pr_))
        print(f"  右/左抬升比: {np.median(pr_[:k])/max(np.median(pl_[:k]),1e-6)*100:.0f}%  差: {np.median(pl_[:k])-np.median(pr_[:k]):+.1f}mm")
    return res, zd, ph


# ---- 仿真 exp_ada_1.6（力标定 + 差分口径校验）----
sim = pd.read_csv('czy/data/exp_ada_1.6/isaac_diag.csv', encoding='utf-8-sig')
res_s, zd_s, ph_s = diagnose(sim, '仿真 exp_ada_1.6 isaac_diag', has_force=True)

print("\n-- 差分口径校验（FK 双脚差 vs foot_z 真值差，双支撑零点）--")
ds = np.abs(ph_s) < 0.2
sw_l = ph_s < -0.3  # 左摆动窗
truth_d = sim['foot_z_l'].values - sim['foot_z_r'].values
rel_fk = (zd_s - np.median(zd_s[ds])) * 1000
rel_tr = (truth_d - np.median(truth_d[ds])) * 1000
c = np.corrcoef(rel_fk[sw_l], rel_tr[sw_l])[0, 1]
print(f"  左摆动窗: corr={c:.3f}  FK 摆动均值={rel_fk[sw_l].mean():.1f}mm 真值={rel_tr[sw_l].mean():.1f}mm")
print(f"  FK 左摆动窗内峰值 median={np.median([rel_fk[a:b].max() for a, b in steps_peaks(rel_fk, sw_l)[1]]):.1f}mm  真值={np.median([rel_tr[a:b].max() for a, b in steps_peaks(rel_tr, sw_l)[1]]):.1f}mm")

# ---- 真机 ----
robot = pd.read_csv('czy/diff/walk_diag_20260901_170239.csv', encoding='utf-8-sig')
res_r, zd_r, ph_r = diagnose(robot, '真机 walk_diag_20260901_170239', phase_sign=1.0)

# ---- 真机分段（按 cmd_linear_x）左右峰值 + 低抬步定位 ----
print(f"\n{'='*72}\n== 真机分段抬脚（cmd_linear_x 分段）==")
rel_l = res_r['L'][1]
rel_r = res_r['R'][1]
sw_l_m = res_r['L'][2]
sw_r_m = res_r['R'][2]
cmdx = robot['cmd_linear_x'].values
seg_id = np.zeros(len(robot), dtype=int)
for i in range(1, len(robot)):
    seg_id[i] = seg_id[i-1] + (1 if cmdx[i] != cmdx[i-1] else 0)
for sg in sorted(set(seg_id)):
    m = seg_id == sg
    cm = cmdx[m][0]
    n_steps = m.sum() / 50.0
    pl_, _ = steps_peaks(rel_l[m], sw_l_m[m])
    pr_, _ = steps_peaks(rel_r[m], sw_r_m[m])
    pl_, pr_ = pl_ * 1000, pr_ * 1000
    if len(pl_) and len(pr_):
        print(f"  seg{sg} cmd={cm:+.2f} ({n_steps:.0f}s): L median={np.median(pl_):.1f}mm(n={len(pl_)})  "
              f"R median={np.median(pr_):.1f}mm(n={len(pr_)})  R/L={np.median(pr_)/max(np.median(pl_),1e-6)*100:.0f}%  "
              f"Lmin={pl_.min():.1f} Rmin={pr_.min():.1f}")
    else:
        print(f"  seg{sg} cmd={cm:+.2f} ({n_steps:.0f}s): 站立/步数不足 (L={len(pl_)} R={len(pr_)})")

# 低抬步定位（<35mm）
print("\n== 真机低抬步定位（峰值<35mm）==")
for s, rel, swm in [('L', rel_l, sw_l_m), ('R', rel_r, sw_r_m)]:
    pks, segs = steps_peaks(rel, swm)
    for pk, (a, b) in zip(pks, segs):
        if pk * 1000 < 35:
            ts = robot['timestamp_ns'].values
            print(f"  {s}: {pk*1000:.1f}mm @ idx {a}~{b}  cmd={cmdx[a]:+.2f}  cycle={robot['cycle_time'].values[a]:.2f}s  t={(ts[min(b, len(ts)-1)]-ts[0])/1e9:.1f}s")
