#include "calibrate.h"
#include "gyro_integrator.h"
#include "optimizer.h"
#include <array>
#include <cmath>
#include <limits>

namespace karpenko {

std::vector<Eigen::Matrix3d> signedPermutations() {
  std::vector<Eigen::Matrix3d> out;
  std::array<int, 3> perm = {0, 1, 2};
  std::array<std::array<int, 3>, 6> perms = {{
    {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}}};
  for (auto& pr : perms) {
    for (int signs = 0; signs < 8; ++signs) {
      Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
      double s0 = (signs & 1) ? -1 : 1;
      double s1 = (signs & 2) ? -1 : 1;
      double s2 = (signs & 4) ? -1 : 1;
      M(0, pr[0]) = s0; M(1, pr[1]) = s1; M(2, pr[2]) = s2;
      if (M.determinant() > 0) out.push_back(M);  // 仅正交旋转 (det=+1)
    }
  }
  // 把单位阵放到首位（便于 enumerate=false 时直接取）。
  for (size_t i = 0; i < out.size(); ++i)
    if (out[i].isApprox(Eigen::Matrix3d::Identity())) { std::swap(out[0], out[i]); break; }
  (void)perm;
  return out;
}

// 针对固定 permutation 的一次坐标下降；返回代价与更新后的参数。
static double runOne(const std::vector<ImuSample>& imu, const std::vector<FrameObs>& obs,
                     const Eigen::Matrix3d& P, double focal, int h,
                     const CalibConfig& cfg, std::vector<double>& x) {
  GyroIntegrator gyro;
  gyro.setSamples(imu);
  gyro.setPermutation(P);
  Eigen::Vector3d last_bias(std::nan(""), 0, 0);
  gyro.setBias(Eigen::Vector3d(x[2], x[3], x[4]));
  gyro.build();
  last_bias = Eigen::Vector3d(x[2], x[3], x[4]);

  auto cost = [&](const std::vector<double>& p) -> double {
    Eigen::Vector3d b(p[2], p[3], p[4]);
    if (!b.isApprox(last_bias)) { gyro.setBias(b); gyro.build(); last_bias = b; }
    CostResult c = evalCost(gyro, obs, p[0], p[1], focal, h);
    if (c.n == 0) return std::numeric_limits<double>::max();
    return c.J;
  };

  std::vector<double> step = {cfg.ts_step, cfg.td_step, cfg.bias_step, cfg.bias_step, cfg.bias_step};
  std::vector<double> tol  = {cfg.ts_tol,  cfg.td_tol,  cfg.bias_tol, cfg.bias_tol, cfg.bias_tol};
  CoordDescentOptions opt; opt.max_iters = cfg.max_iters;
  return coordinateDescent(x, step, tol, cost, opt);
}

CalibResult calibrate(const std::vector<ImuSample>& imu,
                      const std::vector<FrameObs>& obs,
                      double focal, int h, const CalibConfig& cfg) {
  std::vector<Eigen::Matrix3d> perms;
  if (cfg.enumerate_permutations) perms = signedPermutations();
  else perms = {Eigen::Matrix3d::Identity()};

  CalibResult best; double best_J = std::numeric_limits<double>::max();
  for (const auto& P : perms) {
    std::vector<double> x = {cfg.ts_init, cfg.td_init, 0.0, 0.0, 0.0};
    double J = runOne(imu, obs, P, focal, h, cfg, x);
    if (J < best_J) {
      best_J = J;
      best.ts = x[0]; best.td = x[1];
      best.gyro_bias = Eigen::Vector3d(x[2], x[3], x[4]);
      best.R_cam_gyro = P; best.f = focal; best.final_cost = J;
    }
  }
  // RMS 像素（用最优解重算残差数）。
  {
    GyroIntegrator g; g.setSamples(imu); g.setPermutation(best.R_cam_gyro);
    g.setBias(best.gyro_bias); g.build();
    CostResult c = evalCost(g, obs, best.ts, best.td, best.f, h);
    best.rms_px = (c.n > 0) ? std::sqrt(best.final_cost / c.n) : 0.0;
  }
  return best;
}

}  // namespace karpenko
