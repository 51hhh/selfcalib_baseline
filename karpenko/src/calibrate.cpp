#include "calibrate.h"
#include "gyro_integrator.h"
#include "optimizer.h"
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Geometry>

namespace karpenko {

// so(3) 指数映射：旋转向量 -> 旋转矩阵。
static Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  double n = w.norm();
  if (n < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(n, w / n).toRotationMatrix();
}

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
    CostResult c = evalCost(gyro, obs, p[0], p[1], focal, h, cfg.huber_delta);
    if (c.n == 0) return std::numeric_limits<double>::max();
    return c.J;
  };

  std::vector<double> step = {cfg.ts_step, cfg.td_step, cfg.bias_step, cfg.bias_step, cfg.bias_step};
  std::vector<double> tol  = {cfg.ts_tol,  cfg.td_tol,  cfg.bias_tol, cfg.bias_tol, cfg.bias_tol};
  CoordDescentOptions opt; opt.max_iters = cfg.max_iters;
  return coordinateDescent(x, step, tol, cost, opt);
}

// 在给定基准轴排列 P_base 附近，联合优化 {ts,td,bias(3),δ(3)}，8 维。
// 有效轴对齐 = expSO3(δ)·P_base，故 δ 修正离散排列未覆盖的亚度级相机-陀螺失配。
// x 输入含 5 维 {ts,td,bx,by,bz}（作种子），输出扩为 8 维并写回 R_eff。
static double runRefine(const std::vector<ImuSample>& imu, const std::vector<FrameObs>& obs,
                        const Eigen::Matrix3d& P_base, double focal, int h,
                        const CalibConfig& cfg, std::vector<double>& x, Eigen::Matrix3d& R_eff) {
  GyroIntegrator gyro;
  gyro.setSamples(imu);
  x.resize(8, 0.0);  // 追加 δx,δy,δz = 0
  Eigen::Vector3d last_b(std::nan(""), 0, 0);
  Eigen::Vector3d last_d(std::nan(""), 0, 0);
  auto rebuild = [&](const Eigen::Vector3d& b, const Eigen::Vector3d& d) {
    gyro.setPermutation(expSO3(d) * P_base);
    gyro.setBias(b); gyro.build(); last_b = b; last_d = d;
  };
  rebuild(Eigen::Vector3d(x[2], x[3], x[4]), Eigen::Vector3d(x[5], x[6], x[7]));

  auto cost = [&](const std::vector<double>& p) -> double {
    Eigen::Vector3d b(p[2], p[3], p[4]), d(p[5], p[6], p[7]);
    if (!b.isApprox(last_b) || !d.isApprox(last_d)) rebuild(b, d);
    CostResult c = evalCost(gyro, obs, p[0], p[1], focal, h, cfg.huber_delta);
    if (c.n == 0) return std::numeric_limits<double>::max();
    return c.J;
  };

  std::vector<double> step = {cfg.ts_step, cfg.td_step, cfg.bias_step, cfg.bias_step, cfg.bias_step,
                              cfg.drot_step, cfg.drot_step, cfg.drot_step};
  std::vector<double> tol  = {cfg.ts_tol, cfg.td_tol, cfg.bias_tol, cfg.bias_tol, cfg.bias_tol,
                              cfg.drot_tol, cfg.drot_tol, cfg.drot_tol};
  CoordDescentOptions opt; opt.max_iters = cfg.max_iters;
  double J = coordinateDescent(x, step, tol, cost, opt);
  R_eff = expSO3(Eigen::Vector3d(x[5], x[6], x[7])) * P_base;
  return J;
}

CalibResult calibrate(const std::vector<ImuSample>& imu,
                      const std::vector<FrameObs>& obs,
                      double focal, int h, const CalibConfig& cfg) {
  std::vector<Eigen::Matrix3d> perms;
  if (cfg.enumerate_permutations) perms = signedPermutations();
  else perms = {Eigen::Matrix3d::Identity()};

  CalibResult best; double best_J = std::numeric_limits<double>::max();
  Eigen::Matrix3d best_P = Eigen::Matrix3d::Identity();
  std::vector<double> best_x;
  for (const auto& P : perms) {
    std::vector<double> x = {cfg.ts_init, cfg.td_init, 0.0, 0.0, 0.0};
    double J = runOne(imu, obs, P, focal, h, cfg, x);
    if (J < best_J) {
      best_J = J; best_P = P; best_x = x;
      best.ts = x[0]; best.td = x[1];
      best.gyro_bias = Eigen::Vector3d(x[2], x[3], x[4]);
      best.R_cam_gyro = P; best.f = focal; best.final_cost = J;
    }
  }
  // 可选：在最优离散轴排列附近做连续 so(3) 微旋转精修（去亚度级失配偏置）。
  if (cfg.refine_rotation && !best_x.empty()) {
    std::vector<double> x = best_x;  // 5 维种子
    Eigen::Matrix3d R_eff;
    double J = runRefine(imu, obs, best_P, focal, h, cfg, x, R_eff);
    if (J <= best_J) {
      best_J = J;
      best.ts = x[0]; best.td = x[1];
      best.gyro_bias = Eigen::Vector3d(x[2], x[3], x[4]);
      best.R_cam_gyro = R_eff; best.final_cost = J;
    }
  }
  // RMS 像素：始终报告原始 SSE 的 RMS（不受鲁棒核影响），另附鲁棒内点占比。
  {
    GyroIntegrator g; g.setSamples(imu); g.setPermutation(best.R_cam_gyro);
    g.setBias(best.gyro_bias); g.build();
    CostResult c = evalCost(g, obs, best.ts, best.td, best.f, h, cfg.huber_delta);
    best.rms_px = (c.n > 0) ? std::sqrt(c.sse / c.n) : 0.0;
    best.inlier_ratio = (c.n > 0) ? (double)(2 * c.n_inl) / (double)c.n : 0.0;
  }
  return best;
}

}  // namespace karpenko
