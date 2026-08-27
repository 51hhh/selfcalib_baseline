#include "objective.h"

namespace karpenko {

CostResult evalCost(const GyroIntegrator& gyro, const std::vector<FrameObs>& obs,
                    double ts, double td, double f, int h) {
  CostResult out;
  const double invh = (h > 0) ? 1.0 / (double)h : 0.0;
  for (const auto& fo : obs) {
    for (const auto& p : fo.pts) {
      double ti_row = fo.ti + ts * (p.vi * invh);
      double tj_row = fo.tj + ts * (p.vj * invh);
      if (!gyro.covers(ti_row, td) || !gyro.covers(tj_row, td)) continue;
      Eigen::Matrix3d Ri = gyro.R(ti_row, td);
      Eigen::Matrix3d Rj = gyro.R(tj_row, td);
      Eigen::Vector3d bpred = Rj * Ri.transpose() * p.bi;   // 把 i 的方向旋到 j
      if (bpred.z() <= 1e-6 || p.bj.z() <= 1e-6) continue;
      double px = bpred.x() / bpred.z(), py = bpred.y() / bpred.z();
      double mx = p.bj.x() / p.bj.z(), my = p.bj.y() / p.bj.z();
      double rx = f * (px - mx), ry = f * (py - my);
      out.J += rx * rx + ry * ry;
      out.n += 2;
    }
  }
  return out;
}

}  // namespace karpenko
