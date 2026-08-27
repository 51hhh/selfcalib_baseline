// gyro_integrator.h —— 由陀螺角速率积分出连续时间旋转 R(t)，四元数 SLERP 插值查询。
// Karpenko 模型：Δθ(t) = ( P·ω(t+td) + ωd )·Δt
//   - P：陀螺->相机的轴排列/符号（正交阵，枚举确定）
//   - ωd：陀螺零偏（相机系，待估）
//   - td：相机-陀螺时延，仅影响查询时刻（gyro_ts = video_ts + td），不改变累积轨迹
// 因此 setBias/setPermutation 会重建累积四元数，而 td 只在 R(t_cam, td) 查询时移位——
// 令坐标下降在固定 bias/perm 时优化 ts/td 极其廉价。
#pragma once
#include <vector>
#include <Eigen/Geometry>
#include "io.h"

namespace karpenko {

class GyroIntegrator {
 public:
  void setSamples(const std::vector<ImuSample>& s);   // 陀螺样本（陀螺时钟域）
  void setBias(const Eigen::Vector3d& b);
  void setPermutation(const Eigen::Matrix3d& P);
  void build();                                        // 重建累积四元数

  // 查询相机时刻 t_cam 处的相机旋转（内部按 τ = t_cam + td 在陀螺时钟域插值）。
  Eigen::Matrix3d R(double t_cam, double td) const;

  bool covers(double t_cam, double td) const {         // 查询时刻是否落在陀螺时间范围内
    double tau = t_cam + td;
    return !q_.empty() && tau >= tmin_ && tau <= tmax_;
  }
  double tmin() const { return tmin_; }
  double tmax() const { return tmax_; }

 private:
  std::vector<ImuSample> s_;
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
  std::vector<double> gt_;                 // 累积四元数对应的陀螺时刻
  std::vector<Eigen::Quaterniond> q_;      // 累积旋转（gt_[0] 处为单位阵）
  double tmin_ = 0, tmax_ = 0;
};

}  // namespace karpenko
