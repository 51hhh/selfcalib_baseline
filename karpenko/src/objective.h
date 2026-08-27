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

struct CostResult {
  double J = 0;     // 优化用代价（huber_delta>0 时为鲁棒代价，否则=sse）
  long   n = 0;     // 残差分量数（=2×有效点数，用于 RMS）
  double sse = 0;   // 原始平方残差和（不受鲁棒核影响；RMS=sqrt(sse/n)）
  long   n_inl = 0; // 鲁棒内点数（像素残差 <= huber_delta）；huber_delta<=0 时=有效点数
};

// 用已 build 的 integrator 计算总代价。ts 整帧读出时间, td 时延, f 焦距(像素), h 图高。
// huber_delta>0 时对每点像素残差范数施加 Huber 鲁棒核（单位 px），压制近景视差外点；
// <=0 则为普通平方损失。
CostResult evalCost(const GyroIntegrator& gyro, const std::vector<FrameObs>& obs,
                    double ts, double td, double f, int h, double huber_delta = 0.0);

}  // namespace karpenko
