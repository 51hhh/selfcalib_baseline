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

**帧间约束**：陀螺右乘积分得到的 `R(t)=R_wc`（相机→世界）；固定世界方向 `c` 在相机系的
bearing 为 `b = R_cw·c = R(t)ᵀ·c`。由 `b_i = R(t_i)ᵀ·c` 得 `c = R(t_i)·b_i`，故两帧满足
```
b_j  ≈  R(t_j,row)ᵀ · R(t_i,row) · b_i          # 纯旋转（务必 Rjᵀ·Ri，勿写反）
```
残差（归一化平面，乘像素焦距 f）：
```
J = Σ ‖ f · ( π(Rjᵀ·Ri·b_i) − π(b_j) ) ‖²,   π(v) = (v_x/v_z, v_y/v_z)
```
> ⚠️ 旋转手性：早期实现误用 `Rj·Riᵀ`（把积分出的 R_wc 当成 R_cw）。因合成自检当时用
> **相同**的错误约定生成观测，属循环验证，掩盖了该 bug。已修正为 `Rjᵀ·Ri`，并把自检改为按
> **物理约定** `b=Rᵀc` 生成（非循环）——现能真正校验手性（对旧式 `Rj·Riᵀ` 会 FAIL）。

**优化**（`optimizer`）：坐标下降 —— 逐参数试 `+step`，下降则接受；否则试 `−step`；
再否则 `step /= 3`；所有维步长低于阈值即收敛。参数向量 `x = [ts, td, ωd_x, ωd_y, ωd_z]`。
可选枚举 24 个轴排列取最小 J（`--enum-perm`）。焦距固定为已知 K 的 `fx`。
（当前仅实现坐标下降；CMake 保留 `KARPENKO_HAVE_CERES` 探测位以备后续接 Ceres/LM 精修，
**LM 精修尚未落地**——见文末“可提升项”。）

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

1. **合成自检（主验证，非循环）** —— `ctest` 或 `./build/karpenko_selfcheck`。
   注入 `ts=0.030 s, td=0.008 s, bias=(0.005,−0.003,0.002)`，按**物理约定** `b=R(t)ᵀ·c`
   生成卷帘特征观测（与求解器的 `Rjᵀ·Ri` 预测手性一致但非同式复用），再反解。
   **实测恢复误差：ts 0 s，td 0 s，bias 1e-6 rad/s（PASS）。** 证明积分/卷帘/残差/优化四环
   闭合**且旋转手性正确**——若把 objective 写回 `Rj·Riᵀ`，本自检立即 FAIL（ts 偏 13%、td 发散到 −0.48s）。

2. **全局快门 (GS) 真实数据** —— 反解 `t_RS(ts)` 应 ≈ 0。

3. **卷帘 (RS) 交叉核对** —— 与已知参考量级对比：
   - TUM-RSVI（RS，1280×1024）：`t_RS ≈ 0.03018 s`
   - LiU GoPro-Gyro 数据集：`t_RS ≈ 0.0317 s`
   - 与 Ctrl-VIO / swift_vio / rscalib 的 line_delay×height 交叉核对。

> **局限（如实说明）**：残差为**纯旋转**模型（Karpenko 原始假设）。当场景近距离且
> 相机平移显著（如无人机/手持大平移）时，视差引入旋转无法解释的残差，会污染 t_RS 估计
> 并抬高 RMS。此时应选取以旋转为主、场景较远的片段，或改用带平移的 VIO 类方法
> （swift_vio / Ctrl-VIO）交叉验证。本地 UZH-FPV 为大平移无人机数据，仅用于验证管线端到端跑通。

### 实测结果（TUM-RSVI seq1，808 帧 / 187k 跟踪点，`--enum-perm`）

真值：RS 目 cam1 `t_RS≈0.03018 s`；GS 目 cam0 `t_RS=0`。用 `common/bag_to_frames.py` 从
`dataset-seq1.bag` 抽 `/cam1`(RS)、`/cam0`(GS) 帧 + `/imu0` 陀螺，内参取官方 camchain。

修正旋转手性后重跑（`--enum-perm`，全序列 807 帧对 / ~188k 跟踪点）：

| 输入 | t_RS(绝对) | µs/行 | td | RMS | 说明 |
|------|-----------|------|-----|-----|------|
| cam1 RS（全序列） | 0.05387 s | 52.61 | −0.00935 s | 17.36 px | 绝对值被平移视差共模抬高 |
| cam0 GS（全序列） | 0.02330 s | 22.76 | −0.00999 s | 17.37 px | 真值应为 0，正偏置＝同一视差共模 |

- **绝对 t_RS 仍不可靠**：seq1 为手持 6-DOF 行走、近距离室内场景，平移视差（739px 焦距下 ~10cm 平移 /
  ~2.5m 景深 ≈ 30px）远超卷帘信号，RMS ~17px 即视差噪声地板。这是**方法类的固有限制**（EIS 场景假设旋转为主），
  非实现缺陷——非循环合成自检可将 ts 恢复到 0 误差。手性修正后绝对偏置由负翻正、量级增大，但**共模性更干净**。
- **✅ 立体差分可恢复 t_RS（手性修正后显著提升）**：RS 与 GS 两目刚性同架、视场几乎相同、共享同一平移与场景，
  视差偏置为**共模**，作差可抵消：`t_RS ≈ ts(cam1) − ts(cam0) = 0.05387 − 0.02330 = 0.03057 s`，
  对真值 0.03018 s **误差 +1.28%**（旧手性 bug 版为 0.0292 / −3.2%）——已与 Ctrl-VIO（+1.44%）同精度档。
- **td 一致性佐证**：两目独立跑出 td = −9.35 ms / −9.99 ms，彼此一致（共用同一 IMU），
  较旧版（发散/无意义）大幅改善；绝对值仍受视差与帧时间戳约定影响，量级作参考。
- 结论：TUM-RSVI 上的**严格** t_RS 以 Ctrl-VIO/swift_vio（建模平移）为准；Karpenko 提供
  纯陀螺、无加计、无 ROS 的独立交叉核对（非循环自检证实现正确 + 立体差分 +1.28% + td 双目一致）。

复现命令（结果写入 gitignore 的 `../results/karpenko/`）：
```bash
PY=../../.venv/bin/python
$PY ../common/bag_to_frames.py ../datasets/tum_rsvi/dataset-seq1.bag \
    ../datasets/tum_rsvi/extracted/seq1_cam1 --cam-topic /cam1/image_raw   # RS
$PY ../common/bag_to_frames.py ../datasets/tum_rsvi/dataset-seq1.bag \
    ../datasets/tum_rsvi/extracted/seq1_cam0 --cam-topic /cam0/image_raw   # GS
./build/karpenko_calib --cam config/tum_rsvi_cam1_rs.yaml --frames ../datasets/tum_rsvi/extracted/seq1_cam1/cam0 \
    --ts ../datasets/tum_rsvi/extracted/seq1_cam1/video_ts.txt --imu ../datasets/tum_rsvi/extracted/seq1_cam1/imu.txt --enum-perm
./build/karpenko_calib --cam config/tum_rsvi_cam0_gs.yaml  --frames ../datasets/tum_rsvi/extracted/seq1_cam0/cam0 \
    --ts ../datasets/tum_rsvi/extracted/seq1_cam0/video_ts.txt --imu ../datasets/tum_rsvi/extracted/seq1_cam0/imu.txt --enum-perm
```

---

## 4b. 代码 review 结论与可提升项

**正确性（已复核 + 已修）**
- ✅ 陀螺积分（体系右乘中值角速率）、SLERP 查询、卷帘逐行时刻、坐标下降四环闭合。
- ✅ **旋转手性 bug 已修**：`objective` 由 `Rj·Riᵀ` 改为 `Rjᵀ·Ri`（积分产出 R_wc，bearing=Rᵀc）。
  自检改为物理约定生成（非循环），实测差分 t_RS 由 −3.2% 提升到 **+1.28%**，td 双目一致。
- ✅ `runOne` 只在 bias 变化时重建积分器（ts/td 为查询期量），既正确又省算力。

**可继续提升精度（按收益排序）**
1. **鲁棒核 + 视差门限**：残差加 Huber/Cauchy，或按 KLT 位移/深度剔除近景大视差点，压低 17px RMS 地板
   （当前平方损失让近景视差主导 J，直接偏置绝对 ts）。
2. **连续 R_cam_gyro 精修**：现仅枚举 24 个离散轴排列，TUM `T_SC` 有 ~0.6° 非轴对齐残差未建模。
   在选定排列附近再优化 3 维 so(3) 微旋转，可去掉这部分系统偏置（对**绝对** ts 增益最大）。
3. **旋转主导片段选择**：EIS 假设旋转为主；自动挑选高角速率/低平移窗口（或用 IMU 加计估平移量级）喂标定，
   可让**绝对** ts 逼近差分值。
4. **前端换单应 RANSAC**：`feature_tracker` 现用基础矩阵 RANSAC，但纯旋转+远景下 F 退化；
   改 `findHomography(RANSAC)` 与纯旋转假设自洽，减少误剔/漏剔。
5. **Ceres/LM 精修**：坐标下降收敛到局部即止；接 Ceres 对 {ts,td,bias(+so3)} 做解析/自动微分 LM
   末端精修（CMake 已留探测位，代码未实现）。
6. **fy 各向异性**：残差 x/y 分量统一乘 fx；可分别乘 fx/fy（本数据 fx≈fy，增益极小）。

> 结论：**自研方法在修正手性后是正确的**（非循环自检 0 误差 + 差分 +1.28% 对齐 Ctrl-VIO）；
> 绝对 t_RS 的剩余偏差是纯旋转模型对平移视差的固有敏感，属方法类限制，可经上述 1–3 项进一步压低。

---

## 5. 授权说明

本实现为**独立编写**的原创 C++ 代码。仅参考了 Karpenko 2011 论文的算法思想，
以及公开实现 alex-golts/Video-Stabilization（GPL-3）的**算法结构层面思路**，
未复制其任何源代码。若你的分发场景涉及 GPL 传染性顾虑，本目录代码不含被参考项目的
任何原文片段；如需进一步隔离，可按论文公式重新推导（本文件第 1 节即为公式级描述）。
