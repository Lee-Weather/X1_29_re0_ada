# -*- coding: utf-8 -*-
"""exp_ada_1.8 验收（口径与 eval_17.py / 步长专项完全一致，三版对比 1.8/1.7/1.6）"""
import numpy as np
import pandas as pd

R2D = 180 / np.pi
FF, FF1 = 5.0, 1.0


def lift_and_track(csv, tag):
    df = pd.read_csv(csv, encoding='utf-8-sig')
    print(f"\n{'='*76}\n== {tag}")

    # ---------- 分段速度跟踪（稳态后60%） ----------
    cmdx = df['cmd_linear_x'].values
    segs, cur, start = [], cmdx[0], 0
    for i in range(1, len(df) + 1):
        if i == len(df) or cmdx[i] != cur:
            segs.append((cur, start, i))
            if i < len(df):
                cur, start = cmdx[i], i
    trs = {}
    for cmd, a, b in segs:
        vx = df['base_vel_x'].values[a:b]
        n40 = int(len(vx) * 0.4)
        real = vx[n40:].mean()
        tr = real / cmd * 100 if abs(cmd) > 1e-6 else float('nan')
        yaw = (df['base_yaw'].values[b-1] - df['base_yaw'].values[a]) * R2D
        if abs(cmd) > 1e-6:
            trs[cmd] = tr
        print(f"  seg cmd={cmd:+.2f}: track={tr:.0f}%  yawDrift={yaw:+.2f}°")

    # ---------- 抬脚（相对本侧支撑基准） ----------
    ph_raw = df['phase_sin'].values
    l_sw = (df['foot_force_l'].values < FF)[ph_raw > 0.3].mean()
    r_sw = (df['foot_force_r'].values < FF)[ph_raw > 0.3].mean()
    phase_sign = 1.0 if r_sw > l_sw else -1.0

    def peaks(fz, ff, ph, sign, side):
        sw = (ph * sign < -0.3) if side == 'l' else (ph * sign > 0.3)
        st = (ph * sign > 0.3) if side == 'l' else (ph * sign < -0.3)
        base = np.median(fz[st & (ff > FF)])
        rel = fz - base
        out, i, n = [], 0, len(fz)
        while i < n:
            if sw[i]:
                j = i
                while j < n and sw[j]:
                    j += 1
                if j - i >= 5:
                    out.append(rel[i:j].max())
                i = j
            else:
                i += 1
        return np.array(out) * 1000

    fl = peaks(df['foot_z_l'].values, df['foot_force_l'].values, ph_raw, phase_sign, 'l')
    fr = peaks(df['foot_z_r'].values, df['foot_force_r'].values, ph_raw, phase_sign, 'r')
    fls, frs = fl[1:], fr[1:]
    kk = min(len(fls), len(frs))
    print(f"  抬脚: L median={np.median(fls):.1f} p10={np.percentile(fls,10):.1f} min={fls.min():.1f} | "
          f"R median={np.median(frs):.1f} p10={np.percentile(frs,10):.1f} min={frs.min():.1f}")
    print(f"        对称比 R/L={np.median(frs)/max(np.median(fls),1e-6)*100:.0f}%  配对差={np.median(np.abs(fls[:kk]-frs[:kk])):.1f}mm  <30mm步: L={int((fls<30).sum())} R={int((frs<30).sum())}")

    # ---------- 保持项 ----------
    drags, jits = {}, {}
    for s_ in ['l', 'r']:
        swing = df[f'foot_force_{s_}'].values < FF1
        drags[s_] = (swing & (df[f'foot_z_{s_}'].values < 0.005)).sum() / max(swing.sum(), 1) * 100
    for s_ in ['left', 'right']:
        v = df[f'vel_{s_}_ankle_pitch_joint'].values
        swing = df[f'foot_force_{s_[0]}'].values < FF1
        vs = np.where(swing, v, 0)
        sign = np.sign(vs)
        sign[np.abs(vs) < 0.5] = 0
        sign = sign[sign != 0]
        jits[s_] = (np.diff(sign) != 0).sum() / max(swing.sum() / 50.0, 1e-6)
    L_ss = (df['foot_force_l'].values > FF) & (df['foot_force_r'].values < FF)
    hr_l = df['pos_left_hip_roll_joint'].values[L_ss] * R2D
    R_ss = (df['foot_force_r'].values > FF) & (df['foot_force_l'].values < FF)
    hr_r = df['pos_right_hip_roll_joint'].values[R_ss] * R2D
    slip_l = np.abs(df['base_vel_y'].values[L_ss]).mean() * 100
    td = {}
    for s_ in ['l', 'r']:
        ff = df[f'foot_force_{s_}'].values
        ap = df[f'pos_{"left" if s_=="l" else "right"}_ankle_pitch_joint'].values * R2D
        idx = np.where((ff[1:] > FF) & (ff[:-1] <= FF))[0] + 1
        td[s_] = np.median(ap[idx]) if len(idx) else float('nan')
    print(f"  保持: 拖擦 L={drags['l']:.0f}% R={drags['r']:.0f}% | 踝抖 L={jits['left']:.1f} R={jits['right']:.1f}/s")
    print(f"        支撑髋roll: L={abs(hr_l.mean()):.2f}° R={abs(hr_r.mean()):.2f}° | 滑移(L单撑|vy|)={slip_l:.1f}cm/s")
    print(f"        触地踝pitch: L={td['l']:+.1f}° R={td['r']:+.1f}° (default -12°)")
    return df, phase_sign


def step_len(csv, tag, phase_sign):
    """世界系步长（全口径，与 09-04 诊断一致：sin>0=R摆）+ 单支撑推进/停滞 + 占空比"""
    df = pd.read_csv(csv, encoding='utf-8-sig')
    ph = df['phase_sin'].values
    fl, fr = df['foot_force_l'].values, df['foot_force_r'].values
    bpx = df['base_pos_x'].values
    print(f"  -- {tag} 步长/推进（全口径）--")
    swL, swR = (ph < -0.3), (ph > 0.3) if phase_sign > 0 else (ph < -0.3)
    if phase_sign > 0:
        swL, swR = (ph < -0.3), (ph > 0.3)
    else:
        swL, swR = (ph > 0.3), (ph < -0.3)
    # 世界 x：脚 x(base系) 需 FK；此处用 base_pos + 相位窗（对同 yaw 小漂移够用）
    # 与 09-04 诊断一致使用 FK
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

    def fk_x(side):
        ch = build(f'{side}_ankle_pitch_joint')
        cols = [f'pos_{side}_hip_pitch_joint', f'pos_{side}_hip_roll_joint', f'pos_{side}_hip_yaw_joint',
                f'pos_{side}_knee_pitch_joint', f'pos_{side}_ankle_pitch_joint']
        qs = [df[c].values for c in cols]
        x = np.empty(len(df))
        for i in range(len(df)):
            p, R = np.zeros(3), np.eye(3)
            for jn, q in zip(ch, qs):
                p = p + R @ J[jn]['xyz']
                R = R @ rpy_mat(*J[jn]['rpy']) @ ax_mat(J[jn]['ax'], q[i])
            x[i] = p[0]
        return x

    xl, xr = fk_x('left'), fk_x('right')
    yaw = df['base_yaw'].values
    cy, sy = np.cos(yaw), np.sin(yaw)
    wxl = cy * xl - sy * 0 + bpx
    wxr = cy * xr - sy * 0 + bpx
    lands = []
    for s, sw, wx in [('L', swL, wxl), ('R', swR, wxr)]:
        i, n = 0, len(sw)
        while i < n:
            if sw[i]:
                j = i
                while j < n and sw[j]:
                    j += 1
                if j - i >= 5 and abs(bpx[min(j, n-1)] - bpx[i]) >= 0.02:
                    lands.append((min(j, n-1), s, wx[min(j, n-1)]))
                i = j
            else:
                i += 1
    lands.sort(key=lambda t: t[0])
    Ld, Rd = [], []
    for i in range(1, len(lands)):
        dx = (lands[i][2] - lands[i-1][2]) * 1000
        (Ld if lands[i][1] == 'L' else Rd).append(dx)
    Ld, Rd = np.array(Ld), np.array(Rd)
    Ld, Rd = Ld[np.abs(Ld) > 30], Rd[np.abs(Rd) > 30]
    if len(Ld) and len(Rd):
        ml, mr = np.median(Ld), np.median(Rd)
        print(f"     落脚间距: L n={len(Ld)} {ml:.1f}mm | R n={len(Rd)} {mr:.1f}mm | L/R={ml/max(mr,1e-6)*100:.0f}%")
    # 单支撑推进
    ssL = (fr > 5) & (fl < 5)
    ssR = (fl > 5) & (fr < 5)
    for nm, m in [('L摆(R独撑)', ssL), ('R摆(L独撑)', ssR)]:
        adv, i, n = [], 0, len(m)
        stall = tot = 0
        while i < n:
            if m[i]:
                j = i
                while j < n and m[j]:
                    j += 1
                if j - i >= 10:
                    d = (bpx[min(j, n-1)] - bpx[i]) * 1000
                    if abs(d) > 10:
                        tot += 1
                        adv.append(d)
                        if d < 20:
                            stall += 1
                i = j
            else:
                i += 1
        adv = np.array(adv)
        if len(adv):
            print(f"     {nm}: n={len(adv)} median={np.median(adv):.1f}mm p25={np.percentile(adv,25):.1f} 停滞={stall}/{tot}={stall/max(tot,1)*100:.0f}%")
    # 占空比
    mov = np.abs(df['cmd_linear_x'].values) > 0.05
    lsw = (fl < 5) & mov
    rsw = (fr < 5) & mov
    print(f"     摆动占空比(行进段): L={lsw.sum()/max(mov.sum(),1)*100:.0f}% R={rsw.sum()/max(mov.sum(),1)*100}%")


for csv, tag in [('czy/data/exp_ada_1.8/isaac_diag.csv', 'exp_ada_1.8'),
                 ('czy/data/exp_ada_1.7/isaac_diag.csv', 'exp_ada_1.7'),
                 ('czy/data/exp_ada_1.6/isaac_diag.csv', 'exp_ada_1.6')]:
    df, ps = lift_and_track(csv, tag)
    step_len(csv, tag, ps)
