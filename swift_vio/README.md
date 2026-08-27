# swift_vio / KSWF —— 自然场景离线联合标定 t_RS + td 基线（阶段 2）

> selfcalib_baseline 的**阶段 2**（见 `../README.md` §4）：自然场景、无标定板，**同时**离线标定
> 卷帘逐行读出时间 t_RS（readout / line delay）与相机-IMU 时间偏移 td，兼带内参/外参/IMU 内参。
> 这是本项目里**唯一满足"联合 t_RS + td"需求**的可运行外部实现（Ctrl-VIO 锁死 td、OpenVINS 无 RS 模型）。
>
> 论文：Huai et al., *Continuous-Time Spatiotemporal Calibration of a Rolling Shutter Camera-IMU System*
> （及 T-RO 2022 系列）。算法：KSWF（Keyframe-based Sliding Window Filter），MSCKF/EKF 家族，构建于 OKVIS。
> 许可证：**BSD-3**（继承 OKVIS / ETH Zurich）。仅 **ROS1**（kinetic / melodic）。

本目录保存**可版本化部分**：源码 provenance bundle + Dockerfile + 还原/构建/运行脚本 + 标定配置种子。
**不入库**：`upstream/`（恢复过程的裸仓库与中间产物）、`ws/`（还原出的构建工作区）、`results/`（运行产物）。

> ⚠️ **本目录尚未构建 / 未运行**：本机 docker 需交互式 sudo，按分工只交付脚手架（Dockerfile + 脚本 + 配置 +
> 溯源 bundle），**未实际 build、未实际标定**。用户需自行执行 §2 / §3 的命令（含下载数据集）。

---

## 目录结构

```
swift_vio/
├── README.md                       # 本文件
├── restore_swift_vio.sh            # 从 bundle 还原源码树（校验树哈希）→ ws/src/
├── run_swift_vio.sh                # 容器内同步 rosbag 离线标定，dump_output_option=3 导 csv
├── config/
│   └── config_tum_rs_calib.yaml    # TUM-RSVI 标定配置种子（已开 sigma_td / sigma_tr）
├── docker/
│   ├── Dockerfile.melodic          # ROS1 Melodic + Ceres1.14 + Eigen/Boost/glog + (可选)gtsam
│   ├── build.sh                    # sudo docker build（构建上下文含 ws/src 源码）
│   └── bundles/                    # 自包含 git bundle（入库锁 provenance）
│       ├── swift_vio.bundle        # 9.7MB  master d502c14 (+ main / wbl:RS_factor / wbl:newRS1)
│       ├── vio_common.bundle       # 4.4MB  master 2a8e604
│       └── okvis.bundle            # 9.9MB  master 65e30d6（.gitmodules 指定的 wbl1997/okvis 分叉）
└── patches/                        # 空（恢复源码开箱即编译，见 patches/README.md）
```

---

## 1. 源码恢复溯源（重要）

**原仓库全部 404**：`github.com/JzHuai0108/swift_vio`、Bitbucket `jzhuai/swift_vio`、以及调研阶段以为可用的
镜像 `github.com/wbl1997/my_swift_vio` 现均已失效。

**恢复成功**：改从 **Software Heritage** 存档取回（snapshot `07262ecabf72adb1b55d79f3a4b7db3a1353340a`），
经 vault git-bare cooking API 逐分支 cook → 下载（`/raw/` 端点 302 跳转 Azure blob，需 `curl -L`）→
重建统一裸仓库 `upstream/swift_vio_unified.git`，含分支 `master` / `main` / `wbl/RS_factor` / `wbl/newRS1`
（实际源码在 `master` + RS 分支；`main` 仅 40KB README）。依赖 `vio_common`、子模块 `thirdparty/okvis`
（`.gitmodules` 指向 `github.com/wbl1997/okvis.git`，该分叉当前仍可达，已一并打 bundle 作双保险）。

**已核实**：`LICENSE` = OKVIS BSD-3；`config/config_tum_rs_640_50_20_radtan.yaml` 存在且含全部标定键；
离线选项 `load_input_option` / `dump_output_option` 存在（`src/io_wrap/CommonGflags.cpp`）。

**provenance 校验**（`restore_swift_vio.sh` 自动执行，与 kalibr_baseline 同范式——校验树哈希而非 commit SHA）：

| 组件 | bundle | 分支 | commit | 树哈希（内容指纹） |
|---|---|---|---|---|
| swift_vio  | swift_vio.bundle  | master | `d502c14` | `db04037d69b91de822440699beac2dd11532c49c` |
| vio_common | vio_common.bundle | master | `2a8e604` | `29499c1fc209fb32afc4682998c6fb4caa52c5d2` |
| okvis      | okvis.bundle      | master | `65e30d6` | `2c666ed248c7598001dba6b87b3711b2210e235b` |

> bundle 自包含、可 `git bundle verify`，不依赖上游存活；即使 upstream / SWH 日后消失，仍可完整还原。

---

## 2. 构建（需用户在有 sudo 的机器上执行）

```bash
# (1) 从 bundle 还原源码到构建上下文（校验树哈希）
./restore_swift_vio.sh docker/ws/src

# (2) 构建 ROS1 Melodic 镜像 swift_vio:melodic（docker 需 sudo）
./docker/build.sh
#   可选平滑器：WITH_GTSAM=ON ./docker/build.sh    （本标定用 HybridFilter，默认 OFF）
#   非 sudo 环境：DOCKER=podman ./docker/build.sh
```

镜像内容（对应 recovered README 的 build dependencies）：ROS Melodic (Ubuntu 18.04) + Ceres 1.14 +
系统 Eigen 3.3.4（Ubuntu18 满足 Ceres14 要求，无需本地 supplant）+ Boost + glog/gflags + SuiteSparse +
OpenCV + catkin_tools；`catkin build vio_common swift_vio --cmake-args -DUSE_ROS=ON`。

---

## 3. 运行离线标定（需用户执行）

### 3.1 下载数据集 —— TUM-RSVI（主测集）

- 主页：<https://cvg.cit.tum.de/data/datasets/rolling-shutter-dataset>
- CDN：<https://cdn3.vision.in.tum.de/rolling/>
- 双目：**左 GS + 右 RS 同序列**，1280×1024@20Hz + BMI160 六轴 200Hz + OptiTrack 真值轨迹。
- **真值 line delay ≈ 29.4737 µs/行 → 整帧 readout ≈ 0.03018 s**（1024 行 × 29.47µs）。
- 取任一 `dataset-seqX.bag`（ROS bag 格式）放到本机，话题为 `/cam0/image_raw`（左 GS）、
  `/cam1/image_raw`（右 RS）、`/imu0`。

### 3.2 跑同步离线标定

```bash
# TUM-RSVI（默认 config = config/config_tum_rs_calib.yaml，已开 td + readout 自标定）
./run_swift_vio.sh /path/to/dataset-seq1.bag
```

内部等价于（容器内）：

```
rosrun swift_vio swift_vio_node_synchronous config/config_tum_rs_calib.yaml \
  --bagname=<bag> --camera_topics="/cam0/image_raw,/cam1/image_raw" --imu_topic="/imu0" \
  --load_input_option=1 --dump_output_option=3 --output_dir=results/<bag名>
```

- `load_input_option=1`：从磁盘（此处 rosbag）读输入，不订阅实时话题 → 纯离线批处理。
- `dump_output_option=3`：导出 nav 状态 **+ 全部标定参数**（readout t_r、td、内外参、IMU 内参）到 csv。
- 结果在 `results/<bag名>/`：`run.log`（控制台）+ `*.csv`（末行即收敛标定量）。

### 3.3 GS sanity（负样本，可选）

把本地 `../real_frames/uzh_cam0`（帧 + `video_ts.txt` + `imu.txt`，含加速度计）用
`../common/frames_to_rosbag.py` 转成**单目** bag，再用单目变体 config（`monocular_input: true`、删 cam1）跑；
GS 相机期望标出 **t_RS ≈ 0**。TUM-VI（GS 双目）同理，对齐 KSWF 论文的 GS 用法。

---

## 4. 配置种子说明（`config/config_tum_rs_calib.yaml`）

派生自恢复源码 `config/config_tum_rs_640_50_20_radtan.yaml`，**与上游唯一实质区别是打开自标定**：

| 键 | 上游 | 本种子 | 含义 |
|---|---|---|---|
| `camera_params.sigma_td` | 0.0 | **5e-3** | >0 即在线估计 td（相机-IMU 时间偏移） |
| `camera_params.sigma_tr` | 0.0 | **5e-3** | >0 即在线估计 t_r（卷帘逐行读出时间） |

其余保持上游 TUM-RSVI 真值/初值：`image_readout_time: 0.03018`（整帧读出初值）、`imageDelay/image_delay: 0.0`
（td 初值，VISensor 同款）、`algorithm: HybridFilter`、`model_type: BG_BA`、IMU 噪声（sigma_g_c 0.004 等）、
双目内外参（radtan、down_scale 2）。`projection_opt_mode` / `extrinsic_opt_mode` 仍为 `FIXED`——先只联合标
时序量（t_RS + td）；需连同内外参一起标时改为 `Optimized` 并给对应 sigma。

> 上游 README 明确：`sigma_td` / `sigma_tr` 仅对 **HybridFilter / TFVIO** 生效（本配置即 HybridFilter）。

---

## 5. 验证判据（跑通后核对）

| 数据 | 期望 t_RS（整帧 readout） | 期望 td | 说明 |
|---|---|---|---|
| TUM-RSVI 右目 (RS) | **≈ 0.03018 s** | 数据集标定值 | 主判据 |
| TUM-RSVI 左目 (GS) | ≈ 0 | 同上 | 正/负样本对照 |
| real_frames/uzh_cam0 (GS) | ≈ 0 | ≈ UZH 标定值 | 本地 sanity |
| TUM-VI (GS) | ≈ 0 | 数据集标定值 | 对齐论文 GS 用法 |

跨方法一致性：与 Ctrl-VIO 的 t_RS、Karpenko 的 (ts, td)、`../rscalib` 板法的 (lineDelay, td) 两两比对，
差异应在数十 µs / <1% 量级。

**风险（上游作者自陈）**：td 有时发散 / 需充分 6 轴激励才可观。若 td 不收敛，退而只信 t_RS，td 交由
OpenVINS（阶段 3）或 rscalib 板法交叉给出。**若整条路线不可用**，`../README.md` §4 的退路成立：
以 Ctrl-VIO（t_RS）+ Karpenko（t_RS + td，纯陀螺）覆盖联合需求。

---

## 附：关键出处
- 恢复源码统一仓库：`upstream/swift_vio_unified.git`（不入库；bundle 已锁 provenance）。
- 上游 README 构建/运行原文、离线选项表：见恢复源码 `README.md`（`load_input_option` / `dump_output_option`
  / `sigma_td` / `sigma_tr` 语义即摘自其中）。
- 项目总计划与阶段划分：`../README.md` §1（方法表）、§4 阶段 2。
