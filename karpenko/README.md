# Karpenko 2011 复现：纯陀螺视频卷帘 + 时延联合标定

独立实现 Karpenko et al., *"Digital Video Stabilization and Rolling Shutter
Correction using Gyroscopes"* (Stanford Tech Report, 2011) 的**标定内核**：
单段视频 + 陀螺，离线、纯陀螺（无加速度计）、无 ROS，联合估计
**卷帘整帧读出时间 t_RS** 与 **相机-陀螺时延 td**（外加陀螺零偏、轴排列）。

这是 `selfcalib_baseline` 中唯一的“自研”组件。算法思想参考了公开工作
（alex-golts/Video-Stabilization，GPL-3），**但本实现为完全独立编写的 C++ 代码，
未拷贝任何源码**。见文末授权说明。

---

## 1. 算法

相机建模为纯旋转 `R(t) ∈ SO(3)`，由陀螺角速率积分 + 四元数 SLERP 插值得到。

**陀螺 → 相机旋转积分**（`gyro_integrator`）
```
Δθ_k = ( P · ω_mid,k + ω_d ) · Δt_k
q_k  = q_{k-1} · exp(Δθ_k)          # 体系右乘，累积
R(t_cam) = slerp( q at τ ),  τ = t_cam + td
```
- `P`：陀螺→相机的带符号轴排列（正交阵，det=+1，可枚举 24 个）
- `ω_d`：陀螺零偏（待估 3 维）
- `td`：相机-陀螺时延，**只在查询时刻移位**（`gyro_ts = video_ts + td`），
  不改变累积轨迹 —— 故优化 ts/td 时无需重建积分器，仅 bias/perm 改变才重建。

**卷帘模型**（`objective`）：像素行 `y` 的曝光时刻
```
t(i, y) = t_i + ts · (y / h)          # ts = 整帧读出时间 = t_RS，h = 图高
```

**帧间约束**：世界固定方向在两帧的单位方向向量 `b_i, b_j` 满足
```
b_j  ≈  R(t_j,row) · R(t_i,row)ᵀ · b_i          # 纯旋转
```
残差（归一化平面，乘像素焦距 f）：
```
J = Σ ‖ f · ( π(Rj·Riᵀ·b_i) − π(b_j) ) ‖²,   π(v) = (v_x/v_z, v_y/v_z)
```

**优化**（`optimizer`）：坐标下降 —— 逐参数试 `+step`，下降则接受；否则试 `−step`；
再否则 `step /= 3`；所有维步长低于阈值即收敛。参数向量 `x = [ts, td, ωd_x, ωd_y, ωd_z]`。
可选枚举 24 个轴排列取最小 J（`--enum-perm`）。焦距固定为已知 K 的 `fx`。
（Ceres LM 精修为可选项；未检测到 Ceres 时仅用坐标下降。）

**特征跟踪**（`feature_tracker`，rscalib 无自然特征跟踪，此处新写）：
相邻帧 `goodFeaturesToTrack` + `calcOpticalFlowPyrLK`（KLT）+ 前后向一致性校验 +
`findFundamentalMat` RANSAC 剔外点；输出原始像素对，上层按相机模型
（equidistant 鱼眼 / radtan / none）去畸变为归一化 bearing。

### 模块
| 文件 | 职责 |
|------|------|
| `src/io.{h,cpp}`            | 相机 yaml / imu.txt(7或4列) / video_ts.txt / 帧目录读取；结果写出 |
| `src/gyro_integrator.{h,cpp}` | 陀螺积分 + SLERP 查询 `R(t_cam, td)` |
| `src/feature_tracker.{h,cpp}` | KLT + FB 校验 + 基础矩阵 RANSAC |
| `src/objective.{h,cpp}`     | 卷帘逐行时刻 + 纯旋转重投影残差 |
| `src/optimizer.{h,cpp}`     | 坐标下降 |
| `src/calibrate.{h,cpp}`     | 轴排列枚举 + 坐标下降编排 |
| `src/main.cpp`              | CLI：跟踪→去畸变→组装观测→标定→写结果 |
| `test/synthetic_selfcheck.cpp` | 合成自检（注入真值→反解→断言误差） |

---

## 2. 构建

依赖：C++17、Eigen3、OpenCV（core/imgproc/video/calib3d/imgcodecs）、yaml-cpp。
Ceres 可选（未安装则仅坐标下降）。无 ROS、无 aslam。

```bash
cd karpenko
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

产物：`build/karpenko_calib`（标定主程序）、`build/karpenko_selfcheck`（自检）。

---

## 3. 运行

数据格式沿用 rscalib 约定（`rscalib/docs/02-IO层-数据格式协定.md`）：
帧序列 `cam0/%06d.png`、`video_ts.txt`（每行一个秒级时间戳）、
`imu.txt`（7 列 `t wx wy wz ax ay az` 或 4 列 `t wx wy wz`，只用陀螺）、
相机内参 kalibr camchain yaml。

```bash
./build/karpenko_calib \
  --cam    config/uzh_cam0_gs.yaml \
  --frames ../../real_frames/uzh_cam0/cam0 \
  --ts     ../../real_frames/uzh_cam0/video_ts.txt \
  --imu    ../../real_frames/uzh_cam0/imu.txt \
  --out    result_karpenko.yaml \
  [--enum-perm] [--ts-init 0.03] [--td-init 0.0]
```
`--enum-perm` 枚举 24 个陀螺-相机轴排列取最优（相机与 IMU 坐标轴不对齐时需要）。

---

## 4. 验证判据

1. **合成自检（主验证）** —— `ctest` 或 `./build/karpenko_selfcheck`。
   注入 `ts=0.030 s, td=0.008 s, bias=(0.005,−0.003,0.002)`，用积分器生成一致的
   卷帘特征观测，再反解。**实测恢复误差：ts 0 s，td 1e-6 s，bias 2e-6 rad/s（PASS）。**
   证明积分/卷帘/残差/优化四环闭合、旋转约定自洽。

2. **全局快门 (GS) 真实数据** —— 反解 `t_RS(ts)` 应 ≈ 0。

3. **卷帘 (RS) 交叉核对** —— 与已知参考量级对比：
   - TUM-RSVI（RS，1280×1024）：`t_RS ≈ 0.03018 s`
   - LiU GoPro-Gyro 数据集：`t_RS ≈ 0.0317 s`
   - 与 Ctrl-VIO / swift_vio / rscalib 的 line_delay×height 交叉核对。

> **局限（如实说明）**：残差为**纯旋转**模型（Karpenko 原始假设）。当场景近距离且
> 相机平移显著（如无人机/手持大平移）时，视差引入旋转无法解释的残差，会污染 t_RS 估计
> 并抬高 RMS。此时应选取以旋转为主、场景较远的片段，或改用带平移的 VIO 类方法
> （swift_vio / Ctrl-VIO）交叉验证。本地 UZH-FPV 为大平移无人机数据，仅用于验证管线端到端跑通。

---

## 5. 授权说明

本实现为**独立编写**的原创 C++ 代码。仅参考了 Karpenko 2011 论文的算法思想，
以及公开实现 alex-golts/Video-Stabilization（GPL-3）的**算法结构层面思路**，
未复制其任何源代码。若你的分发场景涉及 GPL 传染性顾虑，本目录代码不含被参考项目的
任何原文片段；如需进一步隔离，可按论文公式重新推导（本文件第 1 节即为公式级描述）。
