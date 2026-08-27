#include "gyro_integrator.h"
#include <algorithm>

namespace karpenko {

void GyroIntegrator::setSamples(const std::vector<ImuSample>& s) { s_ = s; }
void GyroIntegrator::setBias(const Eigen::Vector3d& b) { bias_ = b; }
void GyroIntegrator::setPermutation(const Eigen::Matrix3d& P) { P_ = P; }

static Eigen::Quaterniond expq(const Eigen::Vector3d& rv) {
  double n = rv.norm();
  if (n < 1e-12) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(n, rv / n));
}

void GyroIntegrator::build() {
  gt_.clear(); q_.clear();
  if (s_.empty()) return;
  gt_.reserve(s_.size()); q_.reserve(s_.size());
  gt_.push_back(s_[0].t);
  q_.push_back(Eigen::Quaterniond::Identity());
  for (size_t k = 1; k < s_.size(); ++k) {
    double dt = s_[k].t - s_[k-1].t;
    if (dt <= 0) { gt_.push_back(s_[k].t); q_.push_back(q_.back()); continue; }
    // 用相邻样本中值角速度；应用轴排列与零偏（相机系）。
    Eigen::Vector3d wmid = 0.5 * (s_[k].w + s_[k-1].w);
    Eigen::Vector3d w_cam = P_ * wmid + bias_;
    Eigen::Quaterniond dq = expq(w_cam * dt);
    Eigen::Quaterniond qn = (q_.back() * dq).normalized();  // 体系右乘
    gt_.push_back(s_[k].t); q_.push_back(qn);
  }
  tmin_ = gt_.front(); tmax_ = gt_.back();
}

Eigen::Matrix3d GyroIntegrator::R(double t_cam, double td) const {
  double tau = t_cam + td;
  if (q_.empty()) return Eigen::Matrix3d::Identity();
  if (tau <= tmin_) return q_.front().toRotationMatrix();
  if (tau >= tmax_) return q_.back().toRotationMatrix();
  // 二分找 bracket
  auto it = std::upper_bound(gt_.begin(), gt_.end(), tau);
  size_t k = (size_t)(it - gt_.begin());  // gt_[k-1] <= tau < gt_[k]
  if (k == 0) return q_.front().toRotationMatrix();
  double t0 = gt_[k-1], t1 = gt_[k];
  double a = (t1 > t0) ? (tau - t0) / (t1 - t0) : 0.0;
  Eigen::Quaterniond qi = q_[k-1].slerp(a, q_[k]);
  return qi.toRotationMatrix();
}

}  // namespace karpenko
