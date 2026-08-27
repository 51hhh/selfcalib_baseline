# selfcalib_baseline —— 自然场景（无标定板）离线联合标定 t_RS + td 的复现基线

> 创建日期：2026-08-26
> 定位：作为 `../kalibr_baseline`（标定板基线）与 `../rscalib`（自研纯 C++ 板法移植）的**姊妹项目**，
> 收录"**录一段运动视频 + 同步 IMU → 离线标定卷帘读出时间 t_RS 与相机-IMU 时间偏移 td**"的
> 几条**自然场景 / 无标定板**方法的可复现基线与自研复现。
> 用途：为 gyro-EIS 系统提供不依赖标定板、不依赖 LED 频闪的 t_RS/td 标定路线，并与板法基线交叉验证。

本 README 既是项目说明，也是**完整复现计划**（Plan）。调研出处见 `../README.md`（根调研文档）。

### 进度状态（2026-08-26 更新）
- ✅ **阶段 0 完成**：目录骨架 + `.gitignore` 建好；`common/frames_to_rosbag.py` 合成自检 + 实测通过
  （`real_frames/uzh_cam0` → 1619 帧 + 26836 IMU，读回校验一致）。所用 Python 环境为仓库根的 `../.venv`（已装 `rosbags`）。
- ✅ **数据就绪**：`download_tumrsvi.sh` 已下 **TUM-RSVI seq1**（4.24 GB，已用 rosbags 校验）——
  40.4 s，`/cam0/image_raw`(GS) 808 帧 + `/cam1/image_raw`(RS) 808 帧 @20Hz，`/imu0` 8119 @200Hz，
  `/vrpn_client/raw_transform` 4848（OptiTrack 真值）；camchain 种子已入库。
- ✅ **三方法脚手架就位**（bundle 均 `git bundle verify` 通过、记录完整历史）：
  - **ctrlvio/**：Dockerfile.melodic + `ct_odometry_tumrs.yaml` + `run_ctrlvio.sh`；数据路径已统一为 `datasets/tum_rsvi/`。
  - **swift_vio/**：源码经 **Software Heritage 存档恢复**（原 GitHub/Bitbucket 均 404；bundle 含 RS 分支
    `wbl/RS_factor`、`wbl/newRS1`，依赖 `okvis` 含 `RSCameraReprojectionFactor` 分支、`vio_common`）；配置 `sigma_td/sigma_tr>0`、`image_readout_time=0.03018`。
  - **karpenko/**：纯 Eigen+OpenCV+yaml-cpp（无 ROS）**已编译通过**，`synthetic_selfcheck` **自检通过**（ts 误差 0、td 误差 ~1µs）。
- ⏳ **下一步**：在 Docker 内构建并跑通 ctrlvio（阶段 1）/ swift_vio（阶段 2），在 TUM-RSVI seq1 上验证
  RS 目 t_RS≈0.03018 s、GS 目≈0；`OpenVINS/MINS 本轮不实现`（用户明确暂缓）。

---

## 0. 背景与目标（为什么做这个项目）

- 现状：板法侧已复现完毕——`kalibr_baseline`（双 Kalibr ROS1 基线 + Huai 2022 连续时间 §4.1 RS 仿真闭环）
  与 `rscalib`（纯 C++ 无 ROS 移植，功能 2 `imucam_rs` 一次出 lineDelay+td+T_cam_imu）。
- 需求（根调研文档 §0.2）：**录一段相机运动视频（含同步 gyro/IMU）即完成 t_RS 与 td 标定；
  标定板或纯自然场景皆可；不使用额外硬件（无 LED 频闪）、不查现成相机数据库。**
- 本项目聚焦其中的**自然场景路线**，且严格限定"**离线**"（先录制、后批处理），不做实时 VIO。

### 硬约束（贯穿全项目）
1. **td 本质是 gyro↔video 时钟差，纯图像标不出 td**——所有方法必须带同步 IMU/gyro。
2. **t_RS 需运动激发**——旋转对 RS 信号最强；滤波/连续时间路线还需**加速度计 + 充分 6 轴激励**，
   Karpenko 纯陀螺法可省加速度计。
3. 本地已确认数据 `../real_frames/uzh_cam0/`（UZH-FPV cam0，1619 帧 640×480 灰度 + `video_ts.txt`
   + `imu.txt/csv` 7 列含**加速度计**）——数据形态可喂全部路线；但它是**全局快门(GS)**，只能作
   sanity-check（期望标出 t_RS≈0），真值非零 RS 需用 TUM-RSVI 或仿真注入。

---

## 1. 方法调研结论速览（2026-08-26 实地核实）

| 方法 | 出 t_RS | 出 td | 需加计 | 开源可跑 | ROS | 离线方式 | 本项目定位 |
|---|---|---|---|---|---|---|---|
| **Ctrl-VIO**（RA-L 2022，§2.9） | ✅ 每行延时(在线优化) | ❌ **锁死无输出** | ✅ | ✅ APRIL-ZJU/Ctrl-VIO（**无 LICENSE**，一次性 dump 2023-08） | ROS1 Melodic | **直接读 rosbag** 离线批处理 | **阶段1：t_RS 高精度基线** |
| **KSWF / swift_vio**（Huai T-RO 2022，§2.4） | ✅ readout | ✅ | ✅ | 原仓 GitHub/Bitbucket **均 404**；可从 Software Heritage 派生 `wbl1997/my_swift_vio` 恢复；**BSD-3** | ROS1 kinetic/melodic（核心库可 `USE_ROS=OFF`） | `load_input_option=1` 直读视频+IMU csv；`dump_output_option=3` 导出全部标定量到 CSV | **阶段2：唯一自然场景联合 t_RS+td** |
| **Karpenko 2011**（§2.8） | ✅ ts | ✅ gyro delay | ❌ **仅陀螺** | alex-golts/Video-Stabilization（MATLAB+C++mex，**GPL-3**，唯一可跑；Davoudi 空仓、crisp 需已知 t_RS） | 无需 | 独立离线（坐标下降/LM，重投影残差） | **阶段4：最贴 gyro-EIS，自写 C++ 复现** |
| **OpenVINS**（Yang/Huang T-RO 2023，§2.5） | ❌ **开源版无 RS 模型** | ✅ `calib_cam_timeoffset` | ✅ | ✅ rpng/open_vins，**GPL-3**，官方 Docker，仍活跃 | ROS1/2/ROS-free；`ros1_serial_msckf` 离线但仍需 roscore | rosbag 串行离线 | **阶段3：td 交叉核对基线（可选）** |
| **MINS / MVIS**（§2.6） | ❌（含 t_RS 的 **MVIS 无开源**；`rpng/mins` 是另一篇 2309.15390，只标 td） | ✅ `do_calib_dt` | ✅ | ✅ rpng/mins，GPL-3 | ROS1/2/ROS-free + 官方 Docker | rosbag 回放 | **不采用**（过重且不标 t_RS） |

**社区实现核实结论**：遍历 open_vins/mins 的 fork、分支、issue/PR，**没有任何社区实现把 RS readout 加进 OpenVINS 或 MINS**
（维护者在 issue #464 明确 RS 未真正支持）；Basalt 也无 RS 标定。**能跑的自然场景 RS 估计器唯 Ctrl-VIO 一家**。

**据此的采纳决策**：
- **Ctrl-VIO**（t_RS）+ **swift_vio/KSWF**（t_RS+td）为两条**可运行外部基线**；
- **Karpenko** 为**自研复现**（纯陀螺、最贴 gyro-EIS，且复用 `rscalib` 资产）；
- **OpenVINS** 作为**只标 td 的交叉核对**（可选，验证各法 td 一致性）；
- **MINS/MVIS 不采用**。

---

## 2. 数据集计划

| 数据集 | 链接 | 传感器 | 真值 t_RS / td | 用途 | 状态 |
|---|---|---|---|---|---|
| **TUM-RSVI**（首选） | https://cvg.cit.tum.de/data/datasets/rolling-shutter-dataset ；CDN https://cdn3.vision.in.tum.de/rolling/ | 双目（**左 GS + 右 RS** 同序列）1280×1024@20Hz + BMI160 六轴 200Hz + OptiTrack 真值轨迹 | **line delay≈29.4737µs/行 → 整帧 0.03018s**（1024×29.47µs），GS 目≈0 | 三法主测集，自带 t_RS 正/负样本；bag 与 EuRoC 双格式 | **seq1 已下载+校验**（4.24GB，40.4s，cam0/cam1 各 808 帧 + imu0 8119 + vrpn 真值） |
| **TUM-VI** | 已在 `../kalibr_baseline/datasets/tumvi` 流程内 | GS 双目 + 六轴 IMU + 动捕 | t_RS≈0、td 已标定 | GS sanity-check（KSWF 论文同款用法） | 复用 |
| **UZH-FPV / real_frames** | `../real_frames/uzh_cam0`（已抽好帧+ts+imu，含加计） | GS 640×480 + 六轴 | t_RS≈0 | 本地即用 sanity-check | 已有 |
| **LiU GoPro-Gyro** | http://www.cvl.isy.liu.se/research/datasets/gopro-gyro-dataset/ | 1080p30 视频 + **仅陀螺** csv | 参考 readout≈0.0317s（非计量金标准） | Karpenko 纯陀螺法实测 | 待下载 |
| **kalibr_baseline 仿真闭环** | `../kalibr_baseline/run_sim.sh` | GS 拟合 B 样条 → 注入已知 line_delay 合成 RS | **注入值即真值**（137500/82500/51563/41250 ns 四档） | 造带真值数据；**需改造**：观测从 AprilGrid 换随机自然路标才能喂自然场景法 | 待改造 |

> WHU-RSVI / SenseTime-RSVI（Ctrl-VIO 论文引用）声称公开但**下载页未独立核实**；"WHOI RS-VIO 数据集"查无实据，判定误记。
> 单位换算备忘：Ctrl-VIO 输出**每行延时** t_r，整帧 t_RS = t_r × 图像高度；本项目 profile 用整帧 t_RS，导出时统一换算并标注。

---

## 3. 项目结构（仿 kalibr_baseline 范式）

沿用 kalibr_baseline 的可复现哲学：**入库=可版本化部分**（README/脚本/配置种子/源码 bundle/patches），
**重资产不入库**（`datasets/`、`logs/`、`results/` 产物、upstream 源码克隆）。

```
selfcalib_baseline/
├── README.md                     # 本文件（说明 + 计划）
├── .gitignore                    # 排除 datasets/ logs/ results/ 与 upstream 克隆
├── common/
│   ├── frames_to_rosbag.py       # frames+video_ts.txt+imu.txt → ROS1 bag（rosbags 库，无需装 ROS）
│   │                             #   逆用 ../rscalib/test/real_equiv/extract_bag.py 的约定
│   └── sim_natural_rs.py         # 改造 kalibr_baseline 闭环：GS 样条+注入 line_delay+随机自然路标 → 带真值 RS 序列
├── ctrlvio/                      # 阶段1：t_RS 基线
│   ├── README.md                 # 构建/运行/结果读取
│   ├── docker/Dockerfile.melodic # ROS1 Melodic + Ceres1.14 + OpenCV3.3 + vendored basalt/sophus
│   ├── docker/bundles/ctrlvio.bundle   # 源码 provenance（无父根快照，校验树哈希）
│   ├── config/*.yaml             # ld_init/fix_ld=false/ld_lower/ld_upper/time_offset 种子
│   └── run_ctrlvio.sh            # 读 bag 离线跑 → 抓 "estimated line delay"
├── swift_vio/                    # 阶段2：t_RS + td 联合
│   ├── README.md
│   ├── docker/Dockerfile.melodic # ROS1 + Ceres14 + Eigen + BRISK + gtsam(可选) + SuiteSparse
│   ├── docker/bundles/swift_vio.bundle # 从 Software Heritage/wbl1997 恢复后打 bundle 锁 provenance
│   ├── patches/                  # 恢复源码到可编译所需的改动（若有）
│   ├── config/*.yaml             # sigma_td>0 开td / sigma_tr>0 开readout / imageDelay / image_readout_time
│   └── run_swift_vio.sh          # load_input_option=1 或 node_synchronous；dump_output_option=3 导出 CSV
├── openvins/                     # 阶段3（可选）：td 交叉核对
│   ├── README.md
│   ├── docker/                   # 复用官方 Dockerfile_ros1_20_04
│   ├── config/                   # kalibr_imucam_chain.yaml + estimator_config.yaml（calib_cam_timeoffset:true）
│   └── run_openvins_serial.sh    # ros1_serial_msckf 离线
├── karpenko/                     # 阶段4：自研纯陀螺复现（C++）
│   ├── README.md
│   ├── src/                      # 复用 rscalib I/O + B样条 + gyro先验；新增 KLT + 坐标下降/LM
│   └── config/
├── datasets/                     # 不入库；配置种子入库（tum_rsvi camchain/imu 种子等）
└── results/                      # 不入库；参考真值/对照配置种子入库
```

---

## 4. 复现计划（分阶段，含依赖 / ROS·Docker / 数据 / 验证）

按"可运行外部基线优先，自研复现殿后"排序。每阶段独立可交付，可分次推进。

### 阶段 0：脚手架 + 数据转换（无 ROS，纯 Python）
- 建目录骨架 + `.gitignore`（仿 kalibr_baseline）。
- `common/frames_to_rosbag.py`：把 `../real_frames/uzh_cam0`（帧序列 + `video_ts.txt` + `imu.txt`）
  打成 ROS1 bag（图像→`/cam0/image_raw`，IMU→`/imu0`），用 `rosbags` 库**无需装 ROS**（参照
  `../rscalib/test/real_equiv/extract_bag.py` 的反向）。含合成自检。
- 依赖：Python + `rosbags`、numpy、opencv。**无 ROS、无 Docker。**
- 交付：`real_frames/uzh_cam0` → `uzh_cam0.bag`，供各外部基线离线回放。

### 阶段 1：Ctrl-VIO 基线（t_RS）★首选起步
- 目标：自然场景离线标出**每行延时 t_r**，换算整帧 t_RS。
- 步骤：
  1. `git clone https://github.com/APRIL-ZJU/Ctrl-VIO`，打无父根 bundle 入 `ctrlvio/docker/bundles/`（锁 provenance）；
  2. 写 `Dockerfile.melodic`（ROS1 Melodic + Ceres 1.14 + OpenCV 3.3；basalt-headers/Sophus 仓库已 vendored）；
  3. 下载 **TUM-RSVI** `dataset-seqX.bag`；配置 `ld_init=0`、`fix_ld=false`、`ld_upper≈35µs`、`time_offset` 给初值；
  4. `run_ctrlvio.sh`：`roslaunch ctrlvio odometry.launch bag_path:=...`，从控制台/ glog 抓 `estimated line delay`。
- 依赖/环境：**必须 ROS1（Docker 内）**；无官方 Docker，需自建。
- 验证：TUM-RSVI 右目(RS) 应收敛到 **≈29.47µs/行（0.03018s 整帧）**；左目(GS)/TUM-VI 应≈0。
- 局限：**拿不到 td**（代码锁死）；许可证未声明，仅作内部复现/对照，勿分发其代码。

### 阶段 2：swift_vio / KSWF 基线（t_RS + td）
- 目标：自然场景**同时**离线标 t_RS + td（+内参/外参/IMU 内参），是唯一满足联合需求的外部实现。
- 步骤：
  1. **恢复源码**：原仓 404，从 Software Heritage 存档或派生 `github.com/wbl1997/my_swift_vio`（含 RS 分支）取回，
     连同依赖 `JzHuai0108/vio_common`；核对可编译后打 bundle 入 `swift_vio/docker/bundles/`；
  2. 写 `Dockerfile.melodic`（ROS1 + Ceres14 + Eigen≥3.3.4 + Boost + glog + BRISK + gtsam(可选) + SuiteSparse）；
  3. 离线入口二选一：`load_input_option=1`（**直读 `../real_frames` 的帧序列 + imu.txt**，最省事）或
     `swift_vio_node_synchronous`（同步读 bag）；
  4. 配置 `sigma_td=5e-3`（开 td）、`sigma_tr=5e-3`（开 readout）、`imageDelay`/`image_readout_time` 初值；
     `dump_output_option=3` 导出全部标定量 CSV。
- 依赖/环境：**ROS1 only（无 ROS2）**，编译链偏老，Docker 内隔离；**必须有加速度计**（本地数据已满足）。
- 验证：GS 数据（TUM-VI/UZH/real_frames）期望 t_RS≈0、td≈已标定值（对齐 KSWF 论文做法）；
  TUM-RSVI 期望 t_RS≈0.03018s。与 `rscalib` 板法 td/lineDelay 交叉对照。
- 风险：仓库恢复可能缺文件/难编译（作者自陈 td 有时发散）——预留"恢复失败则以 Ctrl-VIO+Karpenko 覆盖需求"的退路。

### 阶段 3（可选）：OpenVINS 基线（仅 td 交叉核对）
- 目标：不追求 t_RS（开源版无 RS），仅用其成熟 td 估计交叉验证阶段 2/4 的 td。
- 步骤：官方 `Dockerfile_ros1_20_04`；`common/frames_to_rosbag.py` 产 bag；相机用 Kalibr 格式 camchain +
  `estimator_config.yaml` 开 `calib_cam_timeoffset:true`；`ros1_serial_msckf` 离线串行跑，从状态取收敛 td。
- 依赖：ROS1 + Ceres + OpenCV + Boost（官方 Docker）；离线仍需 roscore。
- 注意：采集须**充分 6 轴激励**（见 Yang/Huang 退化分析），否则 td 退化不准。

### 阶段 4：Karpenko 自研复现（纯陀螺 t_RS + td）★最贴 gyro-EIS
- 目标：不装 ROS、纯陀螺、单段视频离线同时出 f / t_RS(ts) / td / gyro drift，与现有 EIS 管线同构。
- 算法蓝本：alex-golts 的 `objective_fun.cpp`（重投影残差）+ `camera_param_search.m`（坐标下降）。
  模型：像素成像时刻 `t(i,y)=ti+ts·y/h`；帧间 warp `W=K·R(t(j,yj))·Rᵀ(t(i,yi))·K⁻¹`；
  目标 `J=Σ‖xj−W·xi‖²`，坐标下降（步长不降则反向÷3）或换 Ceres/LM 解 4~5 维 {f,ts,td,ωd} + 轴排列枚举。
- 复用 `../rscalib` 资产：I/O 层（`image_source`/`imu_source`/`config_reader`）、B 样条逐行 keypointTime 机制、
  `findTimeshiftCameraImuPrior()`（td 互相关先验）、`findOrientationPriorCameraToImu()`（gyro 旋转先验）。
- **需新写**（rscalib 里没有）：KLT/SIFT 相邻帧特征跟踪 + RANSAC（OpenCV 直接有）、陀螺积分+SLERP 姿态、
  低维优化器（坐标下降几十行，或接 rscalib 的 aslam 后端/自带 Ceres）。
- 依赖：C++ / Eigen / OpenCV（**无 ROS**）；GPL-3 注意——参考算法重写，勿直接拷贝 alex-golts 代码。
- 数据：本地 `real_frames`（陀螺列即可）做 sanity（GS→ts≈0）；LiU GoPro-Gyro 做真值实测（≈0.0317s）；
  kalibr_baseline 仿真注入档做已知真值回归。
- 验证：与 Ctrl-VIO（t_RS）、swift_vio（t_RS+td）、rscalib 板法三方交叉对照。

---

## 5. 端到端验证矩阵

| 数据 | 期望 t_RS | 期望 td | 用于验证的阶段 |
|---|---|---|---|
| real_frames/uzh_cam0 (GS) | ≈0 | ≈UZH 标定值 | 2,4 (sanity) |
| TUM-VI (GS) | ≈0 | 数据集标定值 | 2,3 (sanity) |
| TUM-RSVI 右目 (RS) | ≈0.03018s（29.47µs/行） | 数据集标定值 | 1,2 |
| TUM-RSVI 左目 (GS) | ≈0 | 同上 | 1,2 (正/负样本) |
| kalibr_baseline 仿真注入（改自然路标） | =注入档 137.5/82.5/51.56/41.25µs | =注入 td | 1,2,4 (已知真值回归) |
| LiU GoPro-Gyro | ≈0.0317s | 方法自估 | 4 |

跨方法一致性：同一序列上 Ctrl-VIO 的 t_RS、swift_vio 的 (t_RS,td)、Karpenko 的 (ts,td)、rscalib 板法结果两两比对，
差异应在 <1% / 数十 µs 量级（参照 kalibr_baseline 单双目 line_delay 差 <0.01% 的判据）。

---

## 6. 待决策 / 风险
- [ ] **首个落地阶段**：建议阶段 1（Ctrl-VIO，读 bag 最快出 t_RS 且有真值可对）；若优先"联合 t_RS+td"则直上阶段 2。
- [x] swift_vio 源码恢复：已从 **Software Heritage 存档恢复**（含 RS 分支，bundle 已入库并校验）；仍待**验证 Docker 内可编译**（阶段 2 执行时确认）。
- [x] TUM-RSVI 已下载 seq1（4.24GB，已校验）；LiU GoPro-Gyro 链接活性待验（仅阶段 4 需要）。
- [ ] 许可证：Ctrl-VIO 无 license、alex-golts GPL-3——本项目仅作内部复现/对照，外发需重写而非拷贝。
- [ ] `sim_natural_rs.py` 把 AprilGrid 观测改随机自然路标的工作量（复用 kalibr 样条+注入机制）。

---

## 附：关键出处
- 根调研文档：`../README.md`（§2.4/2.5/2.6/2.8/2.9、§4 开源清单）。
- 板法基线：`../kalibr_baseline/README.md`（bundle/patches/docker/仿真闭环范式）。
- 自研板法移植与可复用资产：`../rscalib/`（I/O 层、B 样条逐行 keypointTime、td/gyro 先验、IMU 误差项）。
- 论文/仓库链接见 §1、§2 表内。所有仓库状态经 2026-08-26 GitHub API / WebFetch 实地核实。
