# ctrlvio —— Ctrl-VIO 卷帘每行延时(t_RS)离线标定基线（阶段1）

自然场景、无标定板、离线读 rosbag，用连续时间 B 样条 VIO **在线优化卷帘相机每行延时 line_delay**。
本子项目仿 `../../kalibr_baseline` 范式：源码以自包含 **bundle** 锁 provenance + **Dockerfile** + **run 脚本** + **config 种子**。

> **能力边界**：Ctrl-VIO 只标 **t_RS（每行延时）**，**不输出 td**（相机-IMU 时偏在代码里被锁死，
> `trajectory_estimator_options.h` 中 `lock_t_offset=true`）。要 td 见 `../swift_vio`（联合）或 `../karpenko`（纯陀螺）。
> **许可**：上游仓库**无 LICENSE**（默认保留所有权利），本目录仅作**内部复现/对照**，**勿分发其代码**。

## Provenance
- 上游：https://github.com/APRIL-ZJU/Ctrl-VIO
- 快照 commit：`eb95020af80b710ba13e1c708875abbb98b01e84`（"upload code"，2023-08-23，单次提交）
- `docker/bundles/ctrlvio.bundle`：无父根快照，tree=`3c7f43557fa9524d2d7ad3be7125268b7b417934`，与上游 commit 树**逐位一致**（已 `git clone` 回验通过）。
- 已核实关键实现与调研一致：`src/estimator/trajectory_estimator.cpp:311` `AddParameterBlock(line_delay,1)` + 上下界；
  `src/estimator/odometry_manager.cpp:289` `ROS_INFO("estimated line delay: %fus", ...)`；
  `config/ct_odometry_tumrs.yaml` 的 `ld_init/fix_ld/ld_lower/ld_upper/time_offset`。

## 依赖 / 环境
- ROS1 **Melodic**（Ubuntu 18.04）、Eigen3、**Ceres 1.14**、**OpenCV 3.x**、yaml-cpp、glog/gflags。
- basalt-headers（样条）与 Sophus 已 **vendored** 在上游源码（`src/spline`、`src/sophus_lib`），无需另装。
- 全程在 **Docker** 内（本机 docker 需 `sudo`）。无官方镜像，用本目录 Dockerfile 自建。

## 四步复现

### 1) 构建镜像
```bash
cd ctrlvio
./docker/build.sh            # = sudo docker build -t ctrlvio:melodic -f docker/Dockerfile.melodic .
```

### 2) 下载数据集（TUM-RSVI）
```bash
../common/download_tumrsvi.sh seq1        # 下到 ../datasets/tum_rsvi/dataset-seq1.bag
# 或手动：https://cvg.cit.tum.de/data/datasets/rolling-shutter-dataset
#         CDN: https://cdn3.vision.in.tum.de/rolling/calibrated/dataset-seq1.bag
```
TUM-RSVI：双目 1280×1024@20Hz，**左目 /cam0=全局快门、右目 /cam1=卷帘快门**（同序列）；
BMI160 六轴 200Hz（/imu0）；OptiTrack 真值轨迹。默认配置标定 **右目 RS**（`/cam1/image_raw`）。

### 3) 运行（离线）
```bash
./run_ctrlvio.sh tum_rsvi/dataset-seq1.bag
```
- 无显示环境安全：脚本用 `rosrun` 起节点并绕开 launch 里 `required` 的 rviz；容器内自起 `roscore`。
- config 目录 bind-mount 到 `/work/config`，改参数无需重建镜像。
- 输出落到 `../results/ctrlvio/dataset-seq1/`（`run.log` + glog `*.INFO`）。

### 4) 读结果
- 控制台：`estimated line delay: <x>us`（每行延时，取收敛末值）。
- glog：`[line_delay] <秒>`。
- **换算整帧 t_RS = line_delay(每行) × image_height(1024)**。

## 实测结果（TUM-RSVI seq1 `/cam1` 右目 RS，本机已跑）

镜像 `ctrlvio:melodic` 实跑 `dataset-seq1.bag`，`ld_init: 0.0` 从零初值在线优化，约 1s 内收敛并稳定：

| 量 | 实测收敛值 | 真值 | 误差 |
|---|---|---|---|
| line_delay（每行延时） | **29.899 µs/行**（末值 29.8986，末100中位≈29.91） | 29.4737 µs/行 | **+1.44 %** |
| 整帧 t_RS = ld × 1024 | **0.030616 s** | 0.03018 s | +1.44 % |

- 收敛日志：glog `[line_delay] 2.9899e-05`（388 个样本，末段抖动 <0.02µs）；
  控制台 `estimated line delay: 29.8986us`。产物在 `../results/ctrlvio/dataset-seq1/`（不入库）。
- **+1.44% 是本项目对 t_RS 精度最高的结果**（连续时间 B 样条 + 完整 VIO 观测），
  作为其余方法的 t_RS 基准：swift_vio 单目 26.71µs/行(−9.4%)、Karpenko 立体差分
  0.03057s(+1.28%) 均与之同号同量级。

## 验证判据
| 输入 | 相机 | 期望 line_delay(每行) | 期望整帧 t_RS | 实测 |
|---|---|---|---|---|
| TUM-RSVI `/cam1`（右目） | 卷帘 RS | ≈ **29.47 µs** | ≈ **0.03018 s** | ✅ 29.899µs / 0.030616s |
| TUM-RSVI `/cam0`（左目，改 cam_yaml 的 image_topic 试） | 全局 GS | ≈ 0 | ≈ 0 | 未跑（对照可选） |

上游报告：从 0 初值约 1 秒内收敛，WHU 合成集标定误差 ~0.02–3 µs，对初值鲁棒——本机实测与之一致。

## 配置要点（config/ct_odometry_tumrs.yaml）
- `ld_init: 0.0`、`fix_ld: false`（在线标）、`ld_upper: 0.000035`（35µs/行上界）。
- `CameraExtrinsics.time_offset`：仅输入常量，**代码锁死不优化**——本基线拿不到 td。
- 切换到 GS 目做 sanity：把 `config/tumrs/cam_tumrs.yaml` 的 `image_topic` 改成 `/cam0/image_raw`。

## 局限与去向
- 只出 t_RS；td 需 `../swift_vio`（联合 t_RS+td）或 `../karpenko`（纯陀螺 t_RS+td）。
- 环境老（Melodic/Ceres1.14/OpenCV3），已用 Docker 隔离。
- 交叉验证：同序列 t_RS 与 swift_vio、Karpenko、`../../rscalib` 板法结果对照，并分别报告输入模型、
  估计类型（绝对量或立体差分）和相对真值误差；不设置跨方法统一的 `<1%` 判据。
