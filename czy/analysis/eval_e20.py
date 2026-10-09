# -*- coding: utf-8 -*-
"""exp2.0 验收对比（LCP 三态）：
   1.11  = 无 LCP（对照）
   1.11l = LCP 无 warmup，w=1e-4（致瘫）
   exp2.0= LCP warmup=1500，w=1e-5（本次）
"""
import numpy as np
import pandas as pd

FS = 50.0
VERS = [('1.11 (无LCP)', 'czy/data/exp_ada_1.11/isaac_diag.csv'),
        ('1.11l (LCP无warmup)', 'czy/data/exp_ada_1.11l/isaac_diag.csv'),
        ('exp2.0 (LCP+warmup)', 'czy/data/exp2.0/isaac_diag.csv'),
        ('exp2.1 (LPF fc=10Hz)', 'czy/data/exp2.1/isaac_diag.csv'),
        ('exp2.1p (LCP3e-5+KP28)', 'czy/data/exp2.1p/isaac_diag.csv'),
        ('exp2.2b (KP28 only)', 'czy/data/exp2.2b/isaac_diag.csv'),
        ('exp2.2a (LCP3e-5 only)', 'czy/data/exp2.2a/isaac_diag.csv'),
        ('exp2.3a (rerun exp2.0)', 'czy/data/exp2.3a/isaac_diag.csv'),
        ('exp2.3b (smooth -0.05)', 'czy/data/exp2.3b/isaac_diag.csv')]


def swing_peaks(z, mask, min_len=5):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if j - i >= min_len:
                out.append(z[i:j].max())
            i = j
        else:
            i += 1
    return np.array(out)


def segs_of(df):
    cmd = df['cmd_linear_x'].values
    edges = [0] + list(np.where(np.abs(np.diff(cmd)) > 1e-6)[0] + 1) + [len(cmd)]
    return [(a, b, cmd[a]) for a, b in zip(edges[:-1], edges[1:]) if b - a > 100]


JOINTS = [f'{s}_{j}' for s in ['left', 'right']
          for j in ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll']]

print('=' * 104)
print('一、核心验收指标')
print(f'{"版本":>22} {"track0.2/0.4/0.6":>18} {"yaw0.2/0.4/0.6°":>20} {"|yaw_rate|中位":>13} '
      f'{"抬脚R/L":>9} {"双脚离地":>9} {"Δaction":>9} {"jac_frob":>9}')
print('-' * 104)
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    cmd = df['cmd_linear_x'].values
    wz = df['base_ang_vel_z'].values
    m = np.abs(cmd) > 0.05
    trs, yaws = [], []
    for a, b, c in segs_of(df):
        if abs(c) <= 0.05:
            continue
        vx = df['base_vel_x'].values[a:b]
        trs.append(np.median(vx[int((b - a) * 0.4):]) / c * 100)
        y = df['base_yaw'].values[a:b]
        yaws.append((y[-1] - y[0]) * 180 / np.pi)
    tr3 = '/'.join(f'{x:.0f}' for x in trs[:3])
    yw3 = '/'.join(f'{x:+.1f}' for x in yaws[:3])
    yr = np.median(np.abs(wz[m])) * 180 / np.pi
    ph = df['phase_sin'].values
    lp = swing_peaks(df['foot_z_l'].values * 1000, (ph < -0.3) & m)
    rp = swing_peaks(df['foot_z_r'].values * 1000, (ph > 0.3) & m)
    lift = np.median(rp) / np.median(lp) * 100 if len(lp) and len(rp) else np.nan
    both_air = ((df['foot_force_l'].values[m] < 5) & (df['foot_force_r'].values[m] < 5)).mean() * 100
    da = np.mean([np.diff(df[f'action_{j}_joint'].values).std() for j in JOINTS])
    jf = np.median(df['lcp_jac_frob'].values)
    print(f'{tag:>22} {tr3:>18} {yw3:>20} {yr:>13.2f} {lift:>8.0f}% {both_air:>8.0f}% {da:>9.4f} {jf:>9.3f}')

print()
print('=' * 104)
print('二、步态健康度（是否行走）')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    print(f'  {tag:>22}: |vx|={np.abs(df["base_vel_x"].values[m]).mean():.3f} m/s  '
          f'height={df["base_height"].mean():.3f}m  '
          f'knee={df["pos_left_knee_pitch_joint"].values.mean()*180/np.pi:.1f}°  '
          f'hip={df["pos_left_hip_pitch_joint"].values.mean()*180/np.pi:.1f}°')

print()
print('=' * 104)
print('三、LCP 指标（平滑度，越小越平滑）')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    f = df['lcp_jac_frob'].values
    sw = df['lcp_sigma_w'].values
    print(f'  {tag:>22}: jac_frob 中位={np.median(f):.3f} (p10={np.percentile(f,10):.3f} '
          f'p90={np.percentile(f,90):.3f})  σ加权={np.median(sw):.1f}  短历史占比={np.median(df["lcp_frac_short"].values):.3f}')

print()
print('=' * 104)
print('四、关节抖动（运动段 vel 差分 std×FS）')
print(f'{"版本":>22} {"踝roll L/R":>16} {"膝 L/R":>16} {"髋pitch L/R":>16}')
for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    row = []
    for j in ['ankle_roll', 'knee_pitch', 'hip_pitch']:
        jl = np.std(np.diff(df[f'vel_left_{j}_joint'].values[m])) * FS
        jr = np.std(np.diff(df[f'vel_right_{j}_joint'].values[m])) * FS
        row.append(f'{jl:.1f}/{jr:.1f}')
    print(f'{tag:>22} {row[0]:>16} {row[1]:>16} {row[2]:>16}')

print()
print('=' * 104)
print('五、新增标准指标（exp2.3 方案 §7）：踝τ高频 / 力矩>5Hz能量占比 / 谱峰 / 踝稳态误差')
print(f'{"版本":>22} {" ankle tau_hf r/p":>17} {"knee>5Hz":>9} {"hip>5Hz":>8} {"ank>5Hz":>8} '
      f'{"ank pk(Hz)":>11} {"ank_err_roll":>13}')
print('-' * 104)
FS_HI = 100.0


def hp_t(x, fc=5.0):
    w = max(3, int(FS_HI / fc))
    return np.std(x - np.convolve(x, np.ones(w) / w, mode='same'))


def hf_ratio(x):
    x = x - x.mean()
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / FS_HI)
    P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    lo = P[(f > 0) & (f <= 5)].sum()
    hi = P[(f > 5) & (f <= 50)].sum()
    return hi / (lo + hi) * 100


def peak_hz(x):
    x = x - x.mean()
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / FS_HI)
    P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    sel = (f >= 1) & (f <= 50)
    return f[sel][np.argmax(P[sel])]


for tag, path in VERS:
    df = pd.read_csv(path, encoding='utf-8-sig')
    m = np.abs(df['cmd_linear_x'].values) > 0.05
    t_roll = np.mean([hp_t(df[f'effort_{s}_ankle_roll_joint'].values[m]) for s in ['left', 'right']])
    t_pitch = np.mean([hp_t(df[f'effort_{s}_ankle_pitch_joint'].values[m]) for s in ['left', 'right']])
    e_knee = hf_ratio(df['effort_left_knee_pitch_joint'].values[m])
    e_hip = hf_ratio(df['effort_left_hip_pitch_joint'].values[m])
    e_ank = hf_ratio(df['effort_left_ankle_pitch_joint'].values[m])
    pk = peak_hz(df['effort_left_ankle_pitch_joint'].values[m])
    ae = np.mean([np.mean(np.abs(df[f'pos_des_{s}_ankle_roll_joint'].values[m]
                                 - df[f'pos_{s}_ankle_roll_joint'].values[m]))
                  for s in ['left', 'right']])
    tau = '%.2f/%.2f' % (t_roll, t_pitch)
    print(f'{tag:>22} {tau:>17} {e_knee:>8.1f}% {e_hip:>7.1f}% {e_ank:>7.1f}% '
          f'{pk:>11.1f} {ae:>13.3f}')
