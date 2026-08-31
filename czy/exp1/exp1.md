# 实验记录

## 实验索引

| 编号 | 日期 | 摘要 | 状态 | Task ID | GM账号 | checkpoint |
| --- | --- | --- | --- | --- | --- | --- |
| exp0 | 2026-08-28 | 基线：切换 physically_mirrored URDF + 右踝 pitch 符号修复后本机从零训练 6000 轮；回放全程稳定行走、起停正常，速度跟踪 71% 未达标 | ⚠️部分达标（已测试） | 本机训练（RTX A6000） | 无（本机训练） | model_6000.pt |
| exp0.1 | 2026-08-28 | 逐关节 armature 对齐真机阶跃辨识（膝 0.25 / 髋Pitch 0.16 / 髋Yaw 逐侧），Flux 云端从零 6000 轮；回放稳态跟踪 99%（exp0 为 71%），达标 | ✅达标（已测试） | TASK_20260828_129 | limxmtcm4nrkwbk70j@emalupe.com | model_6000.pt |
| exp0.2 | 2026-08-31 | 侧向速度抑制：新增 lat_vel 线性惩罚 + feet_distance 0.2→0.3，从 exp0.1 ckpt6000 云端续训 3000 轮；站立净漂移 -0.094→±0.002 消除 ✅，稳态 \|vy\| 0.126~0.136 略超 0.12 ⚠️，0.6 档跟踪 104% | ⚠️部分达标（已测试） | TASK_20260831_027 | limxmtcm4nrkwbk70j@emalupe.com | model_8999.pt |
| exp_ada_1 | 2026-08-31 | 跨版本系列首训：速度域聚焦 [-0.2,0.6]+关闭指令 curriculum+参考摆幅速度自适应（sagittal × step_scale）+**URDF 切回 X1_12DOF 旧约定**；新账号 L20 从零 6000 轮 | 🔄训练中 | TASK_20260831_160 | limxmtcm5s0yriv75d@emalupe.com | 待定 |

---

## 实验 exp0：physically_mirrored URDF 切换 + 右踝符号修复基线

### 1. 上一实验结果与教训

> 本轮为 exp 系列首个实验（基线），无上一轮数据。
> 背景：检查发现 `X1DHStandCfg.asset.file` 指向不存在的 `x1.urdf`，仓库实际提供 `X1_12DOF.urdf`（旧约定）与 `X1_12DOF_physically_mirrored.urdf`（新约定）两个 URDF；后者右踝 pitch 轴已翻转（`0 0 1` → `0 0 -1`），与 skill post-201-5 记录的"dof10 符号约定反转"一致。
>
> **本轮要解决的具体问题**：
> - 修复资源引用缺失，统一训练/回放/sim2sim 的 URDF 约定
> - 验证右踝 pitch 符号修复后策略能否正常学得稳定行走

### 2. 本轮修改目标

- 目标1：修复 URDF 引用缺失，切换到 `X1_12DOF_physically_mirrored.urdf`
- 目标2：右踝 pitch 符号取反适配新 URDF 轴约定，保证默认位姿与参考步态左右物理对称
- 目标3：本机（非远程服务器）完成从零训练并回放验收
- 验收标准：全程不摔倒、起停正常；前进 0.6 m/s 跟踪 ≥ 80%；Mean reward ≥ 120，Mean episode length ≥ 2100

### 3. 修改内容

### 修改一：URDF 资源路径修复

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `asset.file` | `.../x1/urdf/x1.urdf`（不存在） | `.../x1/urdf/X1_12DOF_physically_mirrored.urdf` | 新 URDF 引用 `../../meshes/`，已建软链接 `resources/robots/meshes` → `Models-meshes/SSOT/Models/meshes` |

**理由**：旧路径文件不存在，训练无法加载资源；新旧 URDF 全量数值对比确认唯一功能差异为右踝 pitch 轴翻转（FK 验证世界轴点积 +0.998 → -0.998）。

### 修改二：右踝 pitch 符号取反（核心）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `default_joint_angles['right_ankle_pitch_joint']` | -0.21 | **+0.21** | 保持物理位姿不变（FK 验证：不改则右踝物理旋转误差 24.06°） |
| `final_swing_joint_delta_pos[10]` | -0.16 | **+0.16** | 保持右踝摆动参考与左踝物理方向对称 |

**理由**：轴翻转后 `q_new = -q_old` 才是同一物理角；参考步态（`compute_ref_state`）依赖左右对称的物理摆动方向。

### 修改三：训练轮数

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `max_iterations` | 20000 | 6000 | 本机单卡训练时长控制（实测 ~4.5 h） |

### 4. 修改文件

- `humanoid/envs/x1/x1_dh_stand_config.py`：修改一、二、三
- `resources/robots/meshes`：新增软链接（mesh 路径解析）
- `humanoid/envs/base/base_task.py`：新增 `enable_headless_render` 支持（headless 相机离屏录制，仅影响回放）
- `humanoid/scripts/play.py`：速度阶梯 0→0.6→0、诊断 CSV 输出、视频叠加 y/z 速度（回放工具，不影响训练）

### 5. 训练参数

| 参数 | 值 |
| --- | --- |
| 训练方式 | 从零 |
| GM账号 | 无（本机训练） |
| max_iterations | 6000 |
| save_interval | 100 |
| num_envs | 4096 |
| seed | 5 |
| learning_rate | 1e-5（fixed） |
| 算力 | NVIDIA RTX A6000 51GB，本机 ThinkStation P720 |
| 镜像 | 无（conda env F1：python3.8 / torch 1.12.1+cu113 / isaacgym preview4） |
| 代码仓库 | 本地 `/home/robot/czy/X1_29_re0_ada`（pip install -e .） |
| 启动命令 | `python humanoid/scripts/train.py --task=x1_dh_stand --run_name=ankle_mirror_6000 --headless` |

### 6. 预期与验收

**目标指标**（训练日志，6000 轮）：

| 指标 | 上一轮 | 本轮目标 | 异常信号 |
| --- | --- | --- | --- |
| Mean reward | —（首个） | ≥ 120 | < 80 |
| Mean episode length | —（首个） | ≥ 2100 | < 1500 |
| 回放前进跟踪（0.6 m/s） | —（首个） | ≥ 80% | < 60% |
| 回放全程 | —（首个） | 不摔倒、起停正常 | 中途摔倒 |

### 7. 实验结果

> 训练任务：本机后台训练，2026-08-28 11:17 ~ 15:5x（约 4.5 h），run 目录 `2026-08-28_11-17-24ankle_mirror_6000`
> 最终 checkpoint：`model_6000.pt`（9.9 MB，已归档 `czy/data/exp0/`）

#### 最终结果（iter 5999 / 回放 model_6000）

| 指标 | 目标 | 实测 | 判定 |
| --- | --- | --- | --- |
| Mean reward | ≥ 120 | 147.8 | ✅ |
| Mean episode length | ≥ 2100 | 2210（上限 2400） | ✅ |
| 全程不摔倒/起停正常 | 是 | 全程最低高度 0.590 m，起停干净 | ✅ |
| 前进 0.6 m/s 跟踪 | ≥ 80% | 稳态 0.426 m/s（**71%**） | ❌ |

#### 训练趋势

| iter | Mean reward | Mean episode length |
| --- | --- | --- |
| 99 | 14.2 | 465 |
| 600 | 48.1 | 1268 |
| 1200 | 78.0 | 1844 |
| 2400 | 114.6 | 2233 |
| 3600 | 132.8 | 2244 |
| 4800 | 147.8 | 2242 |
| 5400 | 146.5 | 2179 |
| 5999 | 147.8 | 2210 |

#### 各奖励项最终值

| 奖励项 | 权重 | 最终值 | 说明 |
| --- | --- | --- | --- |
| tracking_lin_vel | 1.8 | 1.030 | 速度跟踪良好 |
| feet_contact_number | 2.0 | 1.628 | 步态接触时序符合参考 |
| ref_joint_pos | 2.2 | 1.162 | 参考关节跟踪良好 |

#### 回放分段数据（速度阶梯 0→0.6→0，各 10s，`isaac_diag.csv`）

| 阶段 | 实际 vx | \|vy\| | \|vz\| | 身体高度 | 偏航漂移 |
| --- | --- | --- | --- | --- | --- |
| 站立 10s | +0.002 m/s | 0.008 | 0.021 | 0.612±0.009 m | +1.0° |
| 前进 0.6 m/s | +0.396 m/s（稳态 0.426，71%） | 0.146 | 0.108 | 0.612±0.007 m | +5.7° |
| 减速 10s | +0.026 m/s | 0.018 | 0.011 | 0.610±0.002 m | -1.3° |

**结论**：⚠️ 部分达标——右踝符号修复后策略成功学得稳定行走（不摔倒、站立精准、起停干净、偏航漂移 < 6°），但前进速度跟踪 71% 未达 80% 目标；对比中途 model_4800 回放（稳态 0.446 m/s，74%），4800→6000 轮速度跟踪未继续提升，疑似 reward 已在该配置下进入平台期。

**根因分析**（速度跟踪未达标）：

- Mean reward 于 ~4800 轮进入平台期（143~150 震荡），继续训练无增益
- 训练指令范围 `lin_vel_x ∈ [-0.4, 1.2]` 均匀采样，0.6 m/s 处样本密度有限；且 `tracking_lin_vel` 用 exp(-error²·5)，0.17 m/s 误差时该项仍有 0.87，梯度不足以逼平误差
- 侧向速度 |vy|≈0.14 偏大，说明部分推进分量耗散在横向，与步宽/髋 roll 控制相关
- 本轮 armature 仍为旧配置（统一 [0.0001, 0.05]），真机辨识的对齐值（膝 0.25 等）尚未参与训练

**下一轮方向**：

- exp0.1（微调）：加载 model_6000 续训，收紧 `commands.ranges.lin_vel_x` 上限至 0.6 或提高 `tracking_sigma`/`tracking_lin_vel` 权重，强化速度精度
- 引入真机辨识的 armature 逐关节配置（已改好 config，本实验未包含），预期改善动力学保真与速度响应
- 观察侧向速度：如仍 ~0.14，考虑提高 `feet_distance`/`orientation` 权重

---

## 实验 exp0.1：逐关节 armature 对齐真机辨识

### 1. 上一实验结果与教训

> 数据：exp0 训练日志 + `czy/data/exp0/isaac_diag.csv`
> - Mean reward 147.8（≥120 ✅）、Mean episode length 2210/2400（≥2100 ✅）
> - 回放前进 0.6 m/s 稳态跟踪仅 0.426 m/s（**71%**，❌ 未达 80%）；侧向 |vy|≈0.14 m/s 偏大
> - 4800→6000 轮跟踪无提升（0.446→0.426），reward 进入平台期
>
> **核心教训**：
> - URDF 右踝符号修复后步态可正常习得，验证了 URDF 约定修复正确
> - 速度跟踪瓶颈不在训练轮数；本轮引入真机动力学保真（armature），从零重训

### 2. 本轮修改目标

- 目标1：仿真关节有效惯量（M_ii + armature）对齐真机阶跃辨识结果
- 目标2：云端从零 6000 轮重训，验证 armature 对齐后的步态与速度跟踪
- 验收标准：Mean reward ≥ 120；ep_len ≥ 2100；前进 0.6 m/s 稳态跟踪 ≥ 80%（较 exp0 的 71% 提升）；不摔倒

### 3. 修改内容

### 修改一：逐关节 armature 对齐真机阶跃辨识

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `randomize_joint_armature_each_joint` | False | **True** | 旧配置逐关节范围不生效，全部关节统一 [0.0001, 0.05] |
| `joint_1/7_armature_range`（双髋Pitch） | [0.0001, 0.05] | **[0.09, 0.23]** | 辨识 J≈0.43−M_ii 0.271≈0.16，左右对称化 |
| `joint_3/9_armature_range`（双髋Yaw） | [0.0001, 0.05] | **[0.003, 0.018]** | 逐侧辨识 L 0.0148 / R 0.0060 |
| `joint_4/10_armature_range`（双膝） | [0.0001, 0.05] | **[0.18, 0.32]** | CORE：辨识 J≈0.36−0.113≈0.25，真机为 URDF 的 3.2 倍，左右一致 |
| `joint_2/8`（髋Roll）、`joint_5/6/11/12`（双踝） | [0.0001, 0.05] | [0.0001, 0.05]（不变） | 辨识不可靠/无数据，最小干预 |
| `joint_11/12_armature_range` | 缺失 | [0.0001, 0.05] | 补齐 each_joint=True 循环必需 |

**理由**：真机有效惯量 = URDF M_ii + armature；辨识文档（GENERAL_JOINT_STEP_DYNAMICS_ANALYSIS_WORKFLOW §12）给出膝 3.2 倍、髋Pitch 1.5~1.7 倍的明确惯量缺口，armature 是该缺口的仿真等价物。闭环反推 J 含执行器/延迟/摩擦，不采用。

### 4. 修改文件

- `humanoid/envs/x1/x1_dh_stand_config.py`：修改一（domain_rand armature 段）

### 5. 训练参数

| 参数 | 值 |
| --- | --- |
| 训练方式 | 从零（Flux 云端） |
| GM账号 | limxmtcm4nrkwbk70j@emalupe.com |
| max_iterations | 6000 |
| save_interval | 100 |
| num_envs | 4096 |
| seed | 5 |
| learning_rate | 1e-5（fixed） |
| 算力 | ESKU000001（1×4090D 24G） |
| 镜像 | BJX00000001 / V000124（isaac-gym-v19） |
| 代码仓库 | https://github.com/Lee-Weather/X1_29_re0_ada.git @ main，训练代码 commit `df2fe9f`（mesh 相对软链接修复 `0def6aa`） |
| 启动命令 | `gm-run X1_29_re0_ada/humanoid/scripts/train.py --task=x1_dh_stand --run_name=exp0_1_armature --headless --max_iterations=6000` |

### 6. 预期与验收

**目标指标**（训练日志，6000 轮）：

| 指标 | exp0 | 本轮目标 | 异常信号 |
| --- | --- | --- | --- |
| Mean reward | 147.8 | ≥ 120 | < 80 |
| Mean episode length | 2210 | ≥ 2100 | < 1500 |
| 回放前进跟踪（0.6 m/s） | 71% | ≥ 80% | < 60% |
| 回放侧向 \|vy\| | 0.146 | ≤ 0.10 | > 0.20 |

### 7. 实验结果

> 训练任务：TASK_20260828_129（Flux 云端，1×4090D，2026-08-28 启动 ~08-31 完成）
> 最终 checkpoint：`model_6000.pt`（9.9 MB，已归档 `czy/data/exp0.1/`；首次任务 TASK_20260828_113 因 mesh 绝对路径软链接失败，修复 `0def6aa` 后重跑成功）

#### 最终结果（iter 5999 / 回放 model_6000，速度阶梯 0→0.6→0）

| 指标 | 目标 | exp0 实测 | exp0.1 实测 | 判定 |
| --- | --- | --- | --- | --- |
| Mean reward | ≥ 120 | 147.8 | 147~153（末期） | ✅ |
| Mean episode length | ≥ 2100 | 2210 | 2214~2319 | ✅ |
| 前进 0.6 m/s 稳态跟踪 | ≥ 80% | 71%（0.426） | **99%（0.594）** | ✅ |
| 全程不摔倒/起停正常 | 是 | ✅ | ✅（最低高度 0.587 m） | ✅ |
| 回放侧向 \|vy\|（观察项） | ≤ 0.10 | 0.146 | 0.154 | ⚠️ 未改善 |

#### 回放分段数据（`isaac_diag.csv`）

| 阶段 | 实际 vx | \|vy\| | \|vz\| | 身体高度 | 偏航漂移 |
| --- | --- | --- | --- | --- | --- |
| 站立 10s | -0.000 m/s | 0.018 | 0.022 | 0.613±0.009 m | -1.6° |
| 前进 0.6 m/s | +0.530 m/s（整段）/ **0.594（稳态 12-19s，99%）** | 0.154 | 0.103 | 0.605±0.007 m | -8.6° |
| 减速 10s | +0.044 m/s | 0.023 | 0.014 | 0.610±0.003 m | -1.7° |

**结论**：✅ 达标——逐关节 armature 对齐真机辨识后，前进稳态速度跟踪从 exp0 的 71%（0.426 m/s）跃升至 **99%（0.594 m/s）**，站立/减速/稳定性保持同水平，训练指标（reward 147~153、ep_len 2214~2319）持平略优。真机关节动力学保真是速度跟踪瓶颈的关键变量，假设得到验证。

**遗留观察项**：

- 侧向 |vy|≈0.154 未改善（目标 ≤0.10），偏航漂移 -8.6°（exp0 +5.7°），方向随机、幅值小，暂不影响验收
- 前进段身体高度 0.605 略低于 exp0 的 0.612（armature 增大后膝更"沉"），在 soft 范围内

**下一轮方向（晋级 exp1）**：

- 阶段 1 目标已达成（armature 对齐 + 跟踪 ≥80%），晋级 exp1
- exp1 方向：侧向速度抑制（提高 `feet_distance`/`orientation` 权重）或全速度域验收（-0.4~1.0 m/s 阶梯）+ sim2sim/真机部署验证

---

## 实验 exp0.2：侧向速度抑制（净漂移消除）

### 1. 上一实验结果与教训

> 数据：exp0.1 训练日志 + `czy/data/exp0.1/isaac_diag.csv`（前进稳态 12-19s 窗口分析）
> - Mean reward 147~153（✅）、ep_len 2214~2319（✅）、稳态跟踪 **99%**（✅），不摔倒（✅）
> - **遗留**：侧向 |vy|=0.154（目标 ≤0.10）；偏航漂移 -8.6°
> - **新增根因分析**（CSV 频谱 + 统计分离）：
>   - vy 均值 = **-0.094 m/s（恒定净侧漂）**，std=0.158（摆动分量）
>   - 频谱 0.5~3Hz 无主频（>0.003 均为噪声级）→ 漂移非步态固有摆动
>   - 摆动底值估算：消除漂移后 |vy| ≈ std·√(2/π) ≈ 0.126
>   - **奖励结构缺口**：`vel_mismatch_exp` 只惩罚侧向角速度与垂直线速度，**无侧向线速度惩罚**；`tracking_lin_vel` 的 exp(-err²·5) 对 vy=0.09 仅衰减 4%，无有效梯度
>
> **核心教训**：净侧漂无约束地存在于策略中；armature 对齐已验证动力学保真路线有效，本轮在此基础上做奖励结构微调。

### 2. 本轮修改目标

- 目标1：消除侧向净漂移（稳态 vy 均值 \|mean\| ≤ 0.03 m/s）
- 目标2：稳态 \|vy\| ≤ 0.12（理想 ≤ 0.10；摆动底值 ~0.13 为物理下限，随惩罚加入摆动幅值预期同步下降）
- 目标3：前进稳态跟踪保持 ≥ 85%（不因新惩罚项明显回退）
- 目标4：不摔倒，reward/ep_len 不低于 exp0.1 的 90%
- 验收标准：目标 1~3 全部满足且不摔倒

### 3. 修改内容

### 修改一：新增侧向线速度线性惩罚项（核心）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `_reward_lat_vel`（新增） | 无 | `abs(base_lin_vel[:,1]) × 掩码` | 侧向速度绝对值线性惩罚 |
| 指令掩码 | — | `abs(commands[:,1]) ≤ 0.05` 时生效 | 训练侧向指令段（vy∈[-0.4,0.4]）豁免，避免与 tracking_lin_vel 的 vy 跟踪直接冲突；回放 cmd_y=0 恒生效 |
| `rewards.scales.lat_vel` | 无 | **-0.6** | 每步罚 = 0.6×0.02×\|vy\|；稳态 0.15 时约占总奖励 3%，消除漂移净激励 ≈0.0027/步（含 tracking 恢复），全 episode ≈6 分信号量 |

**理由**：净漂移 -0.094 m/s 在现有奖励下无直接约束——`tracking_lin_vel` 高斯型 exp(-err²·5) 对 vy=0.094 仅衰减 4.3%（约 0.0015/步，无梯度）；`vel_mismatch_exp` 只含侧向角速度与垂直线速度。线性 \|·\| 惩罚梯度恒定，对小偏置敏感、不随幅值饱和；权重 -0.6 精确针对漂移偏置，不压制自然摆动（底值 \|vy\|≈0.126，每步仅罚 0.0006~0.0012，不诱导并腿/拖步）。

### 修改二：feet_distance 权重提高（辅助）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `rewards.scales.feet_distance` | 0.2 | **0.3** | 增大步宽支撑约束，降低横向重心调整需求 |

**理由**：双足支撑面越稳，横向速度补偿需求越小；与修改一互补（一个压速度、一个稳步态）。

### 4. 修改文件

- `humanoid/envs/x1/x1_dh_stand_env.py`：新增 `_reward_lat_vel` 函数
- `humanoid/envs/x1/x1_dh_stand_config.py`：`scales.lat_vel = -0.6`、`scales.feet_distance = 0.3`

### 5. 训练参数

| 参数 | 值 |
| --- | --- |
| 训练方式 | **续训**（Flux 云端 trainType=2，源：TASK_20260828_129 @ checkpoint 6000） |
| GM账号 | limxmtcm4nrkwbk70j@emalupe.com |
| max_iterations | 3000（微调续训） |
| save_interval | 100 |
| num_envs | 4096 |
| seed | 5 |
| learning_rate | 1e-5（fixed） |
| 算力 | ESKU000005（1×L20 48G，用户指定） |
| 镜像 | BJX00000001 / V000124（isaac-gym-v19） |
| 代码仓库 | https://github.com/Lee-Weather/X1_29_re0_ada.git @ main，commit `2f2fe9e` |
| 启动命令 | `gm-run X1_29_re0_ada/humanoid/scripts/train.py --task=x1_dh_stand --run_name=exp0_2_lat_vel --headless --max_iterations=3000 --resume --load_run exp0_1_cloud --checkpoint 6000` |

**续训说明**：奖励结构新增一项导致 value 目标变化，lr=1e-5 足够小可平稳过渡；checkpoint 元数据由 `flux task model list`（TASK_20260828_129）获取，不猜测路径。

### 6. 预期与验收

**目标指标**（回放，3000 轮，同一速度阶梯 0→0.6→0）：

| 指标 | exp0.1 | 本轮目标 | 异常信号 |
| --- | --- | --- | --- |
| 稳态 vy 均值（净漂移） | -0.094 | \|mean\| ≤ 0.03 | > 0.06 |
| 稳态 \|vy\| | 0.154 | ≤ 0.12 | > 0.15 |
| 稳态 vx 跟踪 | 99% | ≥ 85% | < 70% |
| Mean reward | 147~153 | ≥ 135 | < 120 |
| 不摔倒 | ✅ | ✅ | 中途摔倒 |

**风险预案**：若续训后跟踪回退 >15%，说明 lat_vel 权重过大，回退权重至 -0.3 重训；若漂移方向翻转或幅值不变，检查 `feet_distance` 是否被步态频率约束顶住（对比 foot_z/接触力相移）。

### 7. 实验结果

> 训练任务：TASK_20260831_027（Flux 云端，1×L20，2026-08-31 完成于 10:45，iter 6000→8998 续训 3000 轮）
> 最终 checkpoint：`model_8999.pt`（已归档 `czy/data/exp0.2/`）

#### 最终结果（iter 8998 / 本地回放 model_8999）

| 指标 | 目标 | exp0.1 实测 | 实测 | 判定 |
| --- | --- | --- | --- | --- |
| Mean reward | ≥ 135 | 147~153 | 157~169（末期 157.4） | ✅ |
| Mean episode length | ≥ 1890（exp0.1 的 90%） | 2214~2319 | 2228~2378（末期 2228） | ✅ |
| 站立净漂移 \|mean vy\| | ≤ 0.03 | -0.094 | **+0.001 ~ +0.002** | ✅ |
| 稳态 \|vy\| | ≤ 0.12 | 0.154 | 0.126~0.136 | ❌ 略超 |
| 0.6 m/s 稳态跟踪 | ≥ 85% | 99% | 104.5%（0.627） | ✅ |
| 不摔倒/起停正常 | ✅ | ✅ | ✅（身高 0.607~0.614） | ✅ |

#### 训练趋势（GM 曲线关键点）

| iter | Mean reward | Mean episode length |
| --- | --- | --- |
| 6000（续训起点） | 1.0（value 重校准伪值） | 35.4 |
| 6500 | 157.8 | 2294.9 |
| 7000 | 165.9 | 2330.0 |
| 7500 | 168.9（峰值） | 2377.7 |
| 8000 | 166.3 | 2339.5 |
| 8500 | 165.2 | 2346.4 |
| 8998 | 157.4 | 2227.6 |

#### 各奖励项最终值（末 50 iter 均值）

| 奖励项 | 权重 | 最终值 | 说明 |
| --- | --- | --- | --- |
| lat_vel | -0.6 | -0.014 | 罚项小且平稳，未过度抑制 |
| tracking_lin_vel | 1.8 | 0.989 | 跟踪保持高水平 |
| feet_distance | 0.3 | 0.279 | 步宽约束生效 |
| ref_joint_pos | 2.2 | 1.165 | 与 exp0.1（1.162）持平 |

#### 回放分段数据（model_8999，本地，阶梯 0→0.2→0.4→0.6→-0.2→0 各 10s，稳态取后 6s，`isaac_diag.csv`）

| 阶段 | 稳态 vx | 净漂移 vy_mean | \|vy\| | 身高 | 偏航漂移 |
| --- | --- | --- | --- | --- | --- |
| 站立 | -0.003 | +0.001 | 0.002 | 0.610 | +0.2° |
| 前进 0.2 | +0.275（超调 +38%） | +0.010 | 0.136 | 0.610 | +12.6° |
| 前进 0.4 | +0.465（超调 +16%） | -0.016 | 0.133 | 0.608 | +10.7° |
| 前进 0.6 | +0.627（104.5%） | -0.041 | 0.134 | 0.607 | +9.0° |
| 后退 -0.2 | -0.140（70%，欠速） | +0.012 | 0.126 | 0.614 | +2.4° |
| 停止 | -0.001 | +0.002 | 0.002 | 0.611 | -0.1° |

**结论**：⚠️ 部分达标——lat_vel 惩罚彻底消除净侧漂（-0.094 → ±0.002，远优于 0.03 目标），前进跟踪不回退（0.6 档 104.5%，reward 157~169 反超 exp0.1），但摆动分量 \|vy\|=0.126~0.136 仍略超 0.12 目标；核心目标（净漂移消除）完全达成。

**根因分析**（\|vy\| 未进 0.12）：

- 净漂移已消除，残余 \|vy\| 为步态固有摆动分量，接近预估物理下限 ~0.126（std·√(2/π)），符合修改一设计预期"不压制自然摆动"
- 偏航漂移行走段 +9~12.6° 偏大（10s 短段内含速度切换瞬态，占比被放大），列为观察项
- 低速 0.2 超调 +38% 与固定摆幅在低速段"迈大步"一致，为 exp1.0 step_scale 自适应提供直接依据；后退欠速（-0.140）同样印证参考轨迹与速度不匹配是全域跟踪瓶颈

**下一轮方向**：

- exp1.0（已就绪，commit `140010d`）：速度域 [-0.2,0.6] + 参考摆幅速度自适应，直接针对低速超调与后退欠速
- \|vy\| 摆动分量与偏航漂移：随 exp1.0 回放复测，不单独开实验

> 注：本实验回放使用 exp1.0 新增的 6 段速度阶梯（commit `140010d` 的 play.py），非旧 3 段阶梯。

---

## 实验 exp_ada_1：速度域聚焦 [-0.2, 0.6] + 参考轨迹迈步幅度速度自适应 + URDF 切回旧约定（跨版本系列首训）

> 编号说明：应用户要求启用跨版本新命名 `exp_ada_1`（X1_29_re0 → X1_29_re0_ada），不沿用 exp{n} 编号规则。本节方案由 exp1.0（旧账号两次提交 TASK_20260831_114/116 均人工停止、未开训）演进而来，新增 URDF 切换。

### 1. 上一实验结果与教训

> 数据：exp0.1 训练日志 + `czy/data/exp0.1/isaac_diag.csv`；exp0.2 训练中（TASK_20260831_027）
> - exp0.1 稳态跟踪 **99%（0.594 m/s @ cmd 0.6）**，Mean reward 147~153，ep_len 2214~2319，不摔倒（✅）
> - 但仅验收了 0.6 m/s 单一速度点；训练指令 `lin_vel_x ∈ [-0.4, 1.2]` 均匀采样，全域（尤其后退 -0.2 与低速 0.2）从未验收
> - 遗留：侧向 |vy|=0.154（exp0.2 lat_vel 惩罚处理中）；偏航漂移 -8.6°
>
> **核心教训**：
> - 代码审查证实：`compute_ref_state` 的摆动幅度为**固定值**（`final_swing_joint_delta_pos` 恒定）+ 固定步频（`cycle_time=0.7s`），与指令速度**无关**——固定摆幅只在 ~0.6 m/s 一个速度点与指令匹配（exp0.1 的 99% 恰在此点），低速段参考步态"迈大步"、高速段"迈小步"，全域跟踪无参考支撑
> - 旧域上界 1.2 m/s 需要的摆幅约为当前标称的 2 倍，超出参考步态能力；聚焦部署域 [-0.2, 0.6] 与参考能力对齐
> - **curriculum 陷阱**：`update_command_curriculum` 在跟踪 reward >80% 上限时自动将 `lin_vel_x` 范围向外扩（下界 -0.25/次、上界 +0.5/次，直至 ±max_curriculum=1.5）。只改 ranges 不关 `commands.curriculum`，收窄会被训练过程冲掉
> - 本轮要解决的具体问题：参考轨迹摆幅随速度自适应 + 指令域聚焦部署速度范围 + 全速度域验收

### 2. 本轮修改目标

- 目标1：指令域聚焦 `lin_vel_x ∈ [-0.2, 0.6]`（部署速度域，含后退），并关闭指令 curriculum 保证域不再漂移
- 目标2：参考轨迹迈步幅度随 |vx_cmd| 线性自适应（sagittal 关节 × step_scale），低速小步、高速大步，步频保持 0.7s 不变
- 目标3：全速度域阶梯验收（-0.2 / 0.2 / 0.4 / 0.6 m/s），替代以往单点验收
- 验收标准：各档稳态跟踪 ≥ 80%（0.6 档）或稳态误差 ≤ 0.15 m/s（低速档）；Mean reward ≥ 120；ep_len ≥ 2100；全程不摔倒；站立段净侧漂 |mean vy| ≤ 0.03（继承 exp0.2 验收）

### 3. 修改内容

### 修改一：指令域聚焦 + 关闭指令 curriculum（核心前置）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `commands.ranges.lin_vel_x` | [-0.4, 1.2] | **[-0.2, 0.6]** | 部署速度域；样本密度从 1.6 m/s 带宽集中到 0.8 m/s，翻倍 |
| `commands.curriculum` | True | **False** | 关闭范围自动扩张；否则训练中 lin_vel_x 会被扩回 ±1.5 上限，域聚焦失效 |

**理由**：exp0 根因分析指出 [-0.4, 1.2] 均匀采样导致 0.6 处样本密度有限；上界 1.2 超出固定参考步态能力。收窄到 [-0.2, 0.6] 后 0.6 处密度 ×2。curriculum 关闭是必要配套——`update_command_curriculum`（base legged_robot.py L823）在跟踪良好时每 episode 将下界 -0.25、上界 +0.5 逐步外扩。

### 修改二：参考轨迹迈步幅度速度自适应（核心）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `compute_ref_state` 摆幅 | 固定 `final_swing_joint_delta_pos[i]` | sagittal 关节（idx 0/3/4/6/9/10）× `step_scale`（逐 env） | 摆幅随指令速度缩放；roll/yaw 关节（1/2/5/7/8/11）保持固定（额状面稳定性/足尖朝向与前进速度弱相关） |
| `rewards.ref_vel_nominal`（新增） | — | **0.6** | 标称锚点速度：当前固定摆幅对应的匹配速度（exp0.1 验证 99% 跟踪的点） |
| `rewards.ref_step_scale_min`（新增） | — | **0.3** | 低速下限，避免摆幅过小陷入摩擦死区（cmd 0.2 → scale 0.33 恰在下限之上） |
| `rewards.ref_step_scale_max`（新增） | — | **1.2** | 高速上限（当前域内 |vx|≤0.6 → scale≤1.0，上限仅留安全裕量） |

**step_scale 定义**：`step_scale = clamp(|vx_cmd| / ref_vel_nominal, s_min, s_max)`，逐 env 张量。

**几何自洽性**：步频固定 2/0.7 = 2.857 步/s。步长 ≈ k·摆幅（k 为腿长几何因子），摆幅 × s 且 v_target ∝ s（由定义 s = v/0.6），故"参考步长-速度"关系自动线性自洽：0.6 m/s → 步长 ~0.21 m（锚点，exp0.1 实测匹配）；0.2 m/s → ~0.07 m；-0.2 m/s → ~0.07 m（sin 步态前后对称，后退天然支持，但**后退从未验收过**，列为重点观察项）。

**实现要点**（`compute_ref_state`）：

```python
step_scale = (self.commands[:, 0].abs() / self.cfg.rewards.ref_vel_nominal).clamp(
    self.cfg.rewards.ref_step_scale_min, self.cfg.rewards.ref_step_scale_max)  # (num_envs,)
d = self.cfg.rewards.final_swing_joint_delta_pos
# 左腿 sagittal：0=hip_pitch, 3=knee, 4=ankle_pitch
self.ref_dof_pos[:, 0] = -sin_pos_l * d[0] * step_scale
self.ref_dof_pos[:, 3] = -sin_pos_l * d[3] * step_scale
self.ref_dof_pos[:, 4] = -sin_pos_l * d[4] * step_scale
# 右腿 sagittal：6=hip_pitch, 9=knee, 10=ankle_pitch
self.ref_dof_pos[:, 6] = sin_pos_r * d[6] * step_scale
self.ref_dof_pos[:, 9] = sin_pos_r * d[9] * step_scale
self.ref_dof_pos[:, 10] = sin_pos_r * d[10] * step_scale
# roll/yaw 关节（1/2/5/7/8/11）维持原固定 delta 写法不变
```

**配套一致性**（无需额外改动，自动成立）：
- 观测已含 `sin_pos/cos_pos`（相位）与 `commands[:,:3]×scale`（速度指令）→ 策略可由 vx 指令预判摆幅，参考可观测
- `ref_joint_pos` 奖励直接用 `ref_dof_pos` → 惩罚自动一致
- `_get_stance_mask`/`feet_contact_number` 只依赖相位（步频不变）→ 占空比不变
- 站立段（|cmd|<0.05）`sw_switch` 已冻结相位 → step_scale 不影响站立

### 修改三：URDF 切回旧约定 X1_12DOF（exp0 修改一/二的逆操作）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| `asset.file` | `X1_12DOF_physically_mirrored.urdf` | **`X1_12DOF.urdf`** | 右踝 pitch 轴 `0 0 -1` → `0 0 1`（与左踝反向），与 MJCF（xyber_x1_serial.xml 右踝轴 0 0 1）约定一致；mesh 走 `../meshes/` 已存在，无需软链接 |
| `default_joint_angles['right_ankle_pitch_joint']` | +0.21 | **-0.21** | 轴翻转的逆变换，物理位姿不变 |
| `final_swing_joint_delta_pos[10]` | +0.16 | **-0.16** | 参考摆动符号回退；左右 delta 同为 -0.16，经相位取正/取负后 ref 左正右负，物理摆动对称 |

**理由**：用户决策切回旧 URDF 约定。副作用：与 exp0.1/exp0.2 的 checkpoint（新约定训练）不兼容，不可续训/回放旧模型——本实验从零训练，无影响。真机部署时 dof10 需在推理端再做符号映射（post-201-5 记录真机约定与 mirrored URDF 一致）。

### 修改四：回放验收速度阶梯扩展（工具，不影响训练）

| 参数 | 旧值 | 新值 | 说明 |
| --- | --- | --- | --- |
| play 速度阶梯 | 0→0.6→0（各 10s） | **0→0.2→0.4→0.6→-0.2→0**（各 10s） | 全速度域验收，含后退档；分段统计沿用 isaac_diag.csv |

### 4. 修改文件

- `humanoid/envs/x1/x1_dh_stand_config.py`：修改一（commands 段）+ 修改二（rewards 段新增 3 参数）
- `humanoid/envs/x1/x1_dh_stand_env.py`：修改二（`compute_ref_state` 增加 step_scale）
- `humanoid/scripts/play.py`：修改三（速度阶梯）

### 5. 训练参数

| 参数 | 值 |
| --- | --- |
| 训练方式 | 从零（指令域+参考轨迹+URDF 三重改动，分布变化大，不用续训） |
| GM账号 | limxmtcm5s0yriv75d@emalupe.com（新账号，公开仓库无需平台 Git 凭证） |
| max_iterations | 6000 |
| save_interval | 100 |
| num_envs | 4096 |
| seed | 5 |
| learning_rate | 1e-5（fixed） |
| 算力 | ESKU000005（1×L20 48G，用户指定；历史：旧账号 TASK_20260831_114 4090D / TASK_20260831_116 L20 均已停止未开训） |
| 镜像 | BJX00000001 / V000124（isaac-gym-v19） |
| 代码仓库 | https://github.com/Lee-Weather/X1_29_re0_ada.git @ main，commit `c019a0c`（速度域聚焦 + step_scale 自适应摆幅 + 阶梯回放 + URDF 切回旧约定） |
| 启动命令 | `gm-run X1_29_re0_ada/humanoid/scripts/train.py --task=x1_dh_stand --run_name=exp_ada_1 --headless --max_iterations=6000` |

**继承说明**：config 已含 exp0.2 的 `lat_vel=-0.6`、`feet_distance=0.3` 与 exp0.1 的逐关节 armature，本轮从零训练全部继承。

### 6. 预期与验收

**目标指标**（回放 model_6000，阶梯 0→0.2→0.4→0.6→-0.2→0）：

| 指标 | exp0.1（仅 0.6 档） | 本轮目标 | 异常信号 |
| --- | --- | --- | --- |
| 0.6 m/s 稳态跟踪 | 99% | ≥ 80% | < 70% |
| 0.4 m/s 稳态误差 | 未验收 | ≤ 0.15 m/s | > 0.25 |
| 0.2 m/s 稳态误差 | 未验收 | ≤ 0.15 m/s | > 0.25（疑似摩擦死区/摆幅下限顶住） |
| -0.2 m/s 稳态误差 | 未验收（后退首次） | ≤ 0.20 m/s | 无法后退/摔倒是转 |
| Mean reward | 147~153 | ≥ 120 | < 100 |
| Mean episode length | 2214~2319 | ≥ 2100 | < 1500 |
| 站立段净侧漂 | -0.094（exp0.1） | \|mean\| ≤ 0.03（继承 exp0.2 验收） | > 0.06 |
| 全程不摔倒/起停正常 | ✅ | ✅ | 中途摔倒 |

**风险预案**：
- 低速档误差大：`ref_step_scale_min` 0.3 → 0.4（接受"参考比指令略大"的稳健小步）
- 后退档异常：先单独回放 -0.2 观察步态方向；若策略拒绝后退，考虑后退段 phase 取反（sin→-sin）
- 0.6 档回退 >15%：检查 step_scale 上限是否被误设 <1.0
- reward 前期低于 exp0.1 同期：正常（参考轨迹随速度变化，探索空间更大），3000 轮后应追平

### 7. 实验结果

> 待训练完成后补充。
