#include "objective.h"
#include <cmath>

namespace karpenko {

CostResult evalCost(const GyroIntegrator& gyro, const std::vector<FrameObs>& obs,
                    double ts, double td, double f, int h, double huber_delta) {
  CostResult out;
  const double invh = (h > 0) ? 1.0 / (double)h : 0.0;
  const bool robust = huber_delta > 0.0;
  const double d2 = huber_delta * huber_delta;
  for (const auto& fo : obs) {
    for (const auto& p : fo.pts) {
      double ti_row = fo.ti + ts * (p.vi * invh);
      double tj_row = fo.tj + ts * (p.vj * invh);
      if (!gyro.covers(ti_row, td) || !gyro.covers(tj_row, td)) continue;
      Eigen::Matrix3d Ri = gyro.R(ti_row, td);
      Eigen::Matrix3d Rj = gyro.R(tj_row, td);
      // GyroIntegrator 右乘体系角速率 -> R(t) 为「相机->世界」旋转 R_wc(t)。
      // 固定世界方向 c 在相机系的 bearing 为 b = R_cw c = R_wc^T c = R(t)^T c，
      // 故 c = R_i b_i，b_j = R_j^T c = R_j^T R_i b_i。（务必用 Rj^T*Ri，勿写反）
      Eigen::Vector3d bpred = Rj.transpose() * Ri * p.bi;   // 把 i 的方向旋到 j
      if (bpred.z() <= 1e-6 || p.bj.z() <= 1e-6) continue;
      double px = bpred.x() / bpred.z(), py = bpred.y() / bpred.z();
      double mx = p.bj.x() / p.bj.z(), my = p.bj.y() / p.bj.z();
      double rx = f * (px - mx), ry = f * (py - my);
      double e2 = rx * rx + ry * ry;
      out.sse += e2;
      out.n += 2;
      if (robust) {
        if (e2 <= d2) { out.J += e2; out.n_inl += 1; }
        else          { out.J += 2.0 * huber_delta * std::sqrt(e2) - d2; } // Huber 线性段
      } else {
        out.J += e2; out.n_inl += 1;
      }
    }
  }
  return out;
}

}  // namespace karpenko
