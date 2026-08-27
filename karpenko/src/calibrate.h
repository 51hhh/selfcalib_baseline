// calibrate.h —— 编排：轴排列枚举 + 坐标下降，联合估 ts(t_RS)/td/gyro bias。
#pragma once
#include <vector>
#include <Eigen/Core>
#include "io.h"
#include "objective.h"

namespace karpenko {

struct CalibConfig {
  double ts_init = 0.0, td_init = 0.0;
  double ts_step = 0.01, td_step = 0.01, bias_step = 5e-3;
  double ts_tol = 1e-6, td_tol = 1e-6, bias_tol = 1e-6;
  bool enumerate_permutations = false;  // true=枚举24个正交轴排列取最优；false=仅单位阵
  int max_iters = 300;
  // —— 精度提升选项（默认关闭以复现旧行为）——
  double huber_delta = 0.0;   // >0 : 对像素残差施加 Huber 鲁棒核（px），压制平移视差外点
  bool refine_rotation = false; // true : 在最优离散轴排列附近再优化 3 维 so(3) 微旋转
  double drot_step = 0.02, drot_tol = 1e-5;  // 微旋转步长/阈值（rad）
};

// 生成 24 个 det=+1 的带符号轴排列矩阵（含单位阵，位于首位）。
std::vector<Eigen::Matrix3d> signedPermutations();

// obs：相邻帧特征对（已去畸变为 bearing）。focal：像素焦距（固定，取 fx）。h：图高。
// imu：陀螺样本（陀螺时钟域）。返回最优结果。
CalibResult calibrate(const std::vector<ImuSample>& imu,
                      const std::vector<FrameObs>& obs,
                      double focal, int h,
                      const CalibConfig& cfg = {});

}  // namespace karpenko
