# common —— 数据准备工具（无 ROS）

跨方法共用的数据准备脚本。均**无需安装 ROS**。

## frames_to_rosbag.py
把 `../rscalib` 约定的"图序 + `video_ts.txt` + `imu.txt`"打成 ROS1 bag，供只吃 rosbag 的外部基线
（swift_vio 同步节点 / OpenVINS）离线回放。是 `../../rscalib/test/real_equiv/extract_bag.py` 的逆操作。

```bash
pip install rosbags                                   # 唯一依赖（+ numpy/opencv）
python3 frames_to_rosbag.py --selftest               # 合成自检（序列化往返，已验证通过）
python3 frames_to_rosbag.py ../../real_frames/uzh_cam0 ../results/uzh_cam0.bag
#  -> /cam0/image_raw (mono8) + /imu0 (sensor_msgs/Imu)
```
已实测：`real_frames/uzh_cam0` → 1619 帧 + 26836 IMU，读回校验通过。此 bag 用于 swift_vio 的 GS sanity（期望 t_RS≈0）。

## bag_to_frames.py
`frames_to_rosbag.py` 的**逆操作**：把 ROS1 bag 拆成"图序 + `video_ts.txt` + `imu.txt`"，喂给
`../karpenko`（纯 C++ 自研法，只吃帧序不吃 bag）。用 `rosbags` 库，**无需装 ROS**。

```bash
PY=../../.venv/bin/python   # 仓库根 .venv 已装 rosbags
$PY bag_to_frames.py ../datasets/tum_rsvi/dataset-seq1.bag ../datasets/tum_rsvi/extracted/seq1_cam1 \
    --cam-topic /cam1/image_raw --imu-topic /imu0        # RS 目（cam1）
$PY bag_to_frames.py ../datasets/tum_rsvi/dataset-seq1.bag ../datasets/tum_rsvi/extracted/seq1_cam0 \
    --cam-topic /cam0/image_raw --imu-topic /imu0        # GS 目（cam0，sanity 负样本）
```
产出 `cam0/%06d.png`（8-bit；TUM-RSVI 为 mono16 满量程，按 `>>8` 降位）+ `video_ts.txt` + `imu.txt`（7 列）。
已实测：seq1 → 808 帧 + 8119 IMU。

## download_tumrsvi.sh
下载 TUM Rolling-Shutter 数据集到 `../datasets/tum_rsvi/`（`curl -C -` 断点续传）。

```bash
bash download_tumrsvi.sh                 # 默认 seq1 + 标定段 + camchain（~9GB，够跑通）
bash download_tumrsvi.sh seq1 seq2       # 指定序列
bash download_tumrsvi.sh all             # 全 10 段（~42GB）
FORMAT=euroc bash download_tumrsvi.sh seq1   # 取 EuRoC/ASL tar
```

要点（已实地核实）：
- **cam0 = 全局快门(GS)**，**cam1 = 卷帘快门(RS)**，同序列共享 OptiTrack 真值轨迹；话题 `/cam0/image_raw`、`/cam1/image_raw`、`/imu0`。
- IMU = BMI160 六轴 @200Hz，硬件同步；相机 1280×1024 @20Hz，pinhole-equidistant。
- **真值 line delay ≈ 29.4737 µs/行 → 整帧 t_RS ≈ 0.03018 s（仅 RS 目 cam1）**；GS 目 cam0 期望 t_RS≈0。
- 标定 camchain 种子已入库：`../datasets/tum_rsvi/camchain-calibration-equidistant4_camimu_dataset-calib-imu1.yaml`。

## sim_natural_rs.py（待实现）
计划：改造 `../../kalibr_baseline/run_sim.sh` 的 GS→RS 仿真闭环——保留"B 样条拟合 GS 位姿 + 注入已知
line_delay"机制，把观测从 AprilGrid 角点换成**随机自然路标**，为 Karpenko/swift_vio/Ctrl-VIO 造带真值
t_RS、td 的自然场景测试数据。见根 README §2。
