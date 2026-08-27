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

> ✅ **本机已实测完成**（2026-08，`swift_vio:melodic` 镜像，TUM-RSVI `dataset-seq1.bag`）。
> 关键结果（单目在线自标定，跑满全程）：
>
> | 相机 | 快门 | 读出 t_r 初值 | **实测收敛 t_r** | 换算 µs/行 | 真值 | 误差 | 实测 td |
> |---|---|---|---|---|---|---|---|
> | cam1 | **卷帘 RS** | 0.03018 s | **0.02735 s** | **26.71 µs/行** | 29.4737 µs/行 | **−9.4 %** | 0.01539 s |
> | cam0 | 全局 GS（对照） | 0.03018 s | **−0.00019 s ≈ 0** | ≈ 0 | 0 | — | 0.00013 s ≈ 0 |
>
> **cam0 对照是关键佐证**：GS 相机读出初值同样给 0.03018s，自标定把它拉回 ≈0，证明滤波器确实在
> *测量*传感器读出而非套用初值。**注意**：上游代码在本数据上有三处堆损坏 bug，只有「单目 + 纯 MSCKF
> + down_scale:1」组合能跑满全程；立体 / SLAM 入状态 / down_scale:2 均崩溃，详见 **§6 排错**。

---

## 目录结构

```
swift_vio/
├── README.md                       # 本文件
├── restore_swift_vio.sh            # 从 bundle 还原源码树（校验树哈希）→ ws/src/
├── run_swift_vio.sh                # 容器内同步 rosbag 离线标定，dump_output_option=3 导 csv
├── config/
│   ├── config_tum_rs_cam1_mono.yaml       # ★可用配方：单目 cam1(RS)+MSCKF+ds1（默认 config）
│   ├── config_tum_rs_cam0_gs_mono.yaml    # 单目 cam0(GS) 对照（读出应自标出 ≈0）
│   ├── config_tum_rs_calib.yaml           # 意图配置：立体双目 HybridFilter（因上游 bug 会崩，留档）
│   └── config_tum_rs_upstream_baseline.yaml  # 纯上游配置（sigma_td/tr=0），复现崩溃/溯源用
├── docker/
│   ├── Dockerfile.melodic          # ROS1 Melodic + Ceres1.14 + Eigen/Boost/glog + (可选)gtsam
│   ├── build.sh                    # sudo docker build（构建上下文含 ws/src 源码）
│   ├── vendor/DownloadProject/     # vendored Crascit/DownloadProject（补 okvis 空子模块，见 §6）
│   ├── patches/                    # 构建期打的源码补丁（防御性零空间守卫，见 §6）
│   └── bundles/                    # 自包含 git bundle（入库锁 provenance）
│       ├── swift_vio.bundle        # 9.7MB  master d502c14 (+ main / wbl:RS_factor / wbl:newRS1)
│       ├── vio_common.bundle       # 4.4MB  master 2a8e604
│       └── okvis.bundle            # 9.9MB  master 65e30d6（.gitmodules 指定的 wbl1997/okvis 分叉）
└── run_swift_vio.sh                # 默认即上面的可用配方
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
# ★ 可用配方：单目 cam1(RS)（默认 config = config/config_tum_rs_cam1_mono.yaml）
./run_swift_vio.sh /path/to/dataset-seq1.bag
#   等价显式写法（cam1 单目，第4参留空即单目）：
./run_swift_vio.sh /path/to/dataset-seq1.bag config/config_tum_rs_cam1_mono.yaml /cam1/image_raw "" /imu0

# cam0(GS) 对照（读出应自标出 ≈0）
./run_swift_vio.sh /path/to/dataset-seq1.bag config/config_tum_rs_cam0_gs_mono.yaml /cam0/image_raw "" /imu0
```

内部等价于（容器内，单目）：

```
rosrun swift_vio swift_vio_node_synchronous config/config_tum_rs_cam1_mono.yaml \
  --bagname=<bag> --camera_topics="/cam1/image_raw" --imu_topic="/imu0" \
  --load_input_option=1 --dump_output_option=3 --output_dir=results/<bag名>
```

- `load_input_option=1`：从磁盘（此处 rosbag）读输入，不订阅实时话题 → 纯离线批处理。
- `dump_output_option=3`：导出 nav 状态 **+ 全部标定参数**（readout t_r、td、内外参、IMU 内参）到 csv。
- 结果在 `results/<bag名>/swift_vio.csv`：`td[s]` / `tr[s]` 两列即收敛标定量（取收敛段中位数）。

> ⚠️ **不要用立体 `config_tum_rs_calib.yaml` 跑**：上游立体匹配器越界写堆，几帧内即
> "double free or corruption" 崩溃（见 §6）。该 config 仅作“意图配置”留档。

---

## 4. 配置说明

全部派生自恢复源码 `config/config_tum_rs_640_50_20_radtan.yaml`，**与上游共同的实质改动是打开自标定**：

| 键 | 上游 | 本配置 | 含义 |
|---|---|---|---|
| `camera_params.sigma_td` | 0.0 | **5e-3** | >0 即在线估计 td（相机-IMU 时间偏移） |
| `camera_params.sigma_tr` | 0.0 | **5e-3** | >0 即在线估计 t_r（卷帘逐行读出时间） |

四个 config 的分工：

| 文件 | 相机 | down_scale | maxInStateLandmarks | 状态 |
|---|---|---|---|---|
| `config_tum_rs_cam1_mono.yaml` | 单目 cam1 RS | 1 | 0（纯 MSCKF） | ★可用，跑满全程 |
| `config_tum_rs_cam0_gs_mono.yaml` | 单目 cam0 GS | 1 | 0 | 对照，跑满全程 |
| `config_tum_rs_calib.yaml` | 立体 GS+RS | 1 | 50（HybridFilter） | 意图配置，因 §6 bug 会崩 |
| `config_tum_rs_upstream_baseline.yaml` | 立体 | 2 | 50 | 纯上游（sigma_td/tr=0），复现崩溃 |

其余保持上游 TUM-RSVI 真值/初值：`image_readout_time`（cam1 RS=0.03018s 整帧读出初值；cam0 GS=0）、
`imageDelay/image_delay: 0.0`（td 初值）、`algorithm: HybridFilter`、`model_type: BG_BA`、IMU 噪声、
内外参（radtan）。`projection_opt_mode` / `extrinsic_opt_mode` 仍为 `FIXED`——先只联合标时序量（t_RS + td）。
可用配方为何是「单目 + down_scale:1 + 纯 MSCKF」，见 **§6 排错**。

> 上游 README 明确：`sigma_td` / `sigma_tr` 仅对 **HybridFilter / TFVIO** 生效（本配置即 HybridFilter）。

---

## 5. 实测结果（TUM-RSVI seq1，单目在线自标定，跑满全程）

取 `swift_vio.csv` 收敛段（末 200 帧）中位数：

| 数据 | 快门 | 收敛 t_r | µs/行 | 真值 µs/行 | 误差 | 收敛 td | 判据 |
|---|---|---|---|---|---|---|---|
| cam1（右目 RS） | 卷帘 | **0.02735 s** | **26.71** | 29.4737 | **−9.4 %** | 0.01539 s | 主判据 ✅ |
| cam0（左目 GS） | 全局 | **−0.00019 s** | ≈ 0 | 0 | — | 0.00013 s | 对照 ✅ |

**解读**：
- cam1(RS) 的 t_r 稳定收敛到 0.02735 s（std≈0.26ms），换算 26.71 µs/行，对真值 29.4737 µs/行偏低 9.4%。
  单目 MSCKF（无 SLAM 入状态、无立体约束）观测较弱，此量级偏差属预期；严格 t_RS 以 Ctrl-VIO
  （29.899 µs/行 / +1.44%）为准，两法同号、同数量级，互为佐证。
- cam0(GS) 从相同的 0.03018s 初值被拉回 **−0.19 ms ≈ 0**（std≈0.19ms）——**核心佐证**：方法确实在测量
  读出而非套用初值；GS 相机正确标出零读出。
- td：cam1≈15.4ms、cam0≈0.13ms。两目共用同一 IMU，td 差异源于 RS 帧时间戳约定（整帧读出 30ms 的
  中点/首行取齐差异）+ 单目弱可观；作为“td 可在线估计”的存在性验证已足够，绝对值精度以板法为准。

跨方法一致性：与 Ctrl-VIO 的 t_RS、Karpenko 的 (ts, td) 比对见 `../README.md` 总表，同号同量级。

---

## 6. 排错：上游三处堆损坏 bug 与可用配方（重要）

上游 swift_vio（Release/`-DNDEBUG`）在 TUM-RSVI 全分辨率数据上有**三处相互独立**的堆损坏，均表现为
`double free or corruption (out)` 中途崩溃。用 valgrind memcheck（本机 Ubuntu18 valgrind 兼容 `-march=native`
构建）逐一定位：

1. **立体匹配器越界写堆** — `okvis::DenseMatcher::doWorkLinearMatching<StereoMatchingAlgorithm<…>>`。
   任何 `down_scale` 下、几帧内即崩。→ **对策：走单目**（`monocular_input: true`，单话题）。KSWF 论文本
   就支持单目卷帘标定。

2. **图像降采样路径堆损坏** — `down_scale: 2` 时崩（`down_scale:1` 跑满全程；崩溃点随状态维数漂移
   2%/69%/81%，是数据相关的越界写而非固定行）。→ **对策：`down_scale: 1`**，用整分辨率 1280×1024 +
   全分辨率焦距（几何自洽）。

3. **SLAM 路标入状态的零空间投影** — `HybridFilter::initializeLandmarksInFilter()` →
   `vio::leftNullspaceAndColumnSpace()`。valgrind 显示该函数内对 Eigen `resize` 出的块 `Invalid free`
   （“16 bytes inside a block”＝典型堆元数据被更早的越界写破坏）。与 `down_scale` 无关。
   → **对策：`maxInStateLandmarks: 0`（纯 MSCKF）**，不把路标移入状态即绕开。

**∴ 可用配方 =「单目 + down_scale:1 + 纯 MSCKF」**，即 `config_tum_rs_cam1_mono.yaml`，实测唯一跑满全程者。

**已入库的防御补丁**（`docker/patches/`，构建期 `patch -p1`）：给 `leftNullspaceAndColumnSpace` /
`HybridFilter` 的零空间路径加 `rows<=cols` 守卫（Release 下 `assert`/`eigen_assert` 被 `NDEBUG` 关闭，
欠定时负尺寸 `resize/block` 会越界）。**说明**：这些守卫堵住了“负尺寸”这一类触发，但上述 bug 3 的堆损坏
发生在更早的写入（守卫处 `rows>cols`，故补丁对它是防御性冗余、非根治）；根治需在 debug/`eigen_assert`
构建下二分定位具体越界的 `.block()/.segment()`，本轮以“绕开 + 单目可用结果”交付，未深挖。

**其他构建修复**（`Dockerfile.melodic`）：
- `catkin config --extend /opt/ros/melodic` 用字面量 `melodic`（`source setup.bash` 会把 `${ROS_VERSION}`
  覆盖成 `1`，用变量会 `--extend /opt/ros/1` 报错）。
- vendored `Crascit/DownloadProject`（`docker/vendor/`）覆盖 okvis 的空子模块：parentless 快照 bundle
  不含子模块内容（仅 gitlink），还原后该目录为空 → `include(DownloadProject.cmake)` 失败。DBoW2 本体仍由
  `download_project` 在 cmake 阶段联网拉取（github 存活）。

**风险（上游作者自陈）**：td 需充分 6 轴激励才可观、有时发散。**若整条路线不可用**，`../README.md` §4 的
退路成立：以 Ctrl-VIO（t_RS）+ Karpenko（t_RS + td，纯陀螺）覆盖联合需求。

---

## 附：关键出处
- 恢复源码统一仓库：`upstream/swift_vio_unified.git`（不入库；bundle 已锁 provenance）。
- 上游 README 构建/运行原文、离线选项表：见恢复源码 `README.md`（`load_input_option` / `dump_output_option`
  / `sigma_td` / `sigma_tr` 语义即摘自其中）。
- 项目总计划与阶段划分：`../README.md` §1（方法表）、§4 阶段 2。
