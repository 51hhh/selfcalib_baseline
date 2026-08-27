// objective.h —— 卷帘重投影残差与代价函数。
// 观测为相邻帧间的特征对，已去畸变到归一化平面并转为单位方向向量（bearing）。
// 每帧的行(row)记录用于卷帘逐行时刻 t(i,y)=t_i+ts*(y/h)。
#pragma once
#include <vector>
#include <Eigen/Geometry>
#include "gyro_integrator.h"

namespace karpenko {

struct TrackedPoint {
  double vi, vj;              // 两帧中的像素行 (y)
  Eigen::Vector3d bi, bj;     // 单位方向向量（归一化坐标 (x,y,1) 归一化）
};

struct FrameObs {
  double ti, tj;              // 两帧基准时间戳（秒，video 时钟）
  std::vector<TrackedPoint> pts;
};

struct CostResult { double J = 0; long n = 0; };  // n = 残差分量数（用于 RMS）

// 用已 build 的 integrator 计算总代价。ts 整帧读出时间, td 时延, f 焦距(像素), h 图高。
CostResult evalCost(const GyroIntegrator& gyro, const std::vector<FrameObs>& obs,
                    double ts, double td, double f, int h);

}  // namespace karpenko
