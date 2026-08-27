// synthetic_selfcheck.cpp —— 合成自检。
// 注入已知 ts_true / td_true / gyro_bias_true，用 GyroIntegrator 生成一致的“真值旋转”，
// 据此合成卷帘特征观测（bearing 对），再用 calibrate() 反解，断言误差足够小。
//
// ★关键（非循环自检）：观测按【物理约定】生成 —— GyroIntegrator 右乘体系角速率产出
//   R(t)=R_wc（相机->世界），固定世界方向 c 在相机系的 bearing 为 b = R(t)^T·c。
//   求解器内部预测 bpred = Rj^T·Ri·bi（objective.cpp）。二者旋转手性必须一致才能收敛回
//   注入值——故本自检能真正校验旋转约定的正确性（若把 objective 写成 Rj·Ri^T，此处会 FAIL）。
//
// 返回 0 = 通过；非 0 = 失败。无 gtest 依赖。
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include "gyro_integrator.h"
#include "objective.h"
#include "calibrate.h"

using namespace karpenko;

int main() {
  // ---- 注入的真值 ----
  const double ts_true = 0.030;      // 整帧读出时间 t_RS (s)
  const double td_true = 0.008;      // 相机-陀螺时延 (s)
  const Eigen::Vector3d bias_true(0.005, -0.003, 0.002);  // rad/s
  const int    h  = 1024;            // 图高
  const double fx = 1000.0;          // 像素焦距
  const int    nframes = 30;
  const double fps = 30.0;
  const int    npts = 40;            // 每帧对特征点数

  // ---- 合成陀螺（陀螺时钟域，200 Hz，覆盖视频时段并留裕量）----
  std::vector<ImuSample> imu;
  const double hz = 200.0, dt = 1.0 / hz;
  for (double t = -0.5; t <= 6.0; t += dt) {
    Eigen::Vector3d w(
        0.30 * std::sin(2.0 * M_PI * 0.7 * t + 0.3),
        0.25 * std::sin(2.0 * M_PI * 1.1 * t + 1.1),
        0.20 * std::sin(2.0 * M_PI * 0.5 * t + 2.0));
    imu.push_back({t, w});
  }

  // ---- 真值旋转 R_true(t_cam) = 积分(w_meas + bias_true)，P=I ----
  GyroIntegrator truth;
  truth.setSamples(imu);
  truth.setPermutation(Eigen::Matrix3d::Identity());
  truth.setBias(bias_true);
  truth.build();

  // ---- 合成观测 ----
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> urow(0.0, (double)h);
  std::uniform_real_distribution<double> uang(-0.4, 0.4);

  std::vector<FrameObs> obs;
  for (int i = 0; i < nframes - 1; ++i) {
    double ti = i / fps, tj = (i + 1) / fps;   // video 时钟
    FrameObs fo; fo.ti = ti; fo.tj = tj;
    for (int k = 0; k < npts; ++k) {
      double vi = urow(rng), vj = urow(rng);   // 两帧中的像素行
      // 世界固定方向 c，物理约定 bearing = R(t)^T·c（R=R_wc）=> bj = Rj^T·Ri·bi
      Eigen::Vector3d c(uang(rng), uang(rng), 1.0);
      c.normalize();
      double ti_row = ti + ts_true * (vi / h);
      double tj_row = tj + ts_true * (vj / h);
      Eigen::Vector3d bi = (truth.R(ti_row, td_true).transpose() * c).normalized();
      Eigen::Vector3d bj = (truth.R(tj_row, td_true).transpose() * c).normalized();
      if (bi.z() <= 1e-6 || bj.z() <= 1e-6) continue;
      TrackedPoint tp; tp.vi = vi; tp.vj = vj; tp.bi = bi; tp.bj = bj;
      fo.pts.push_back(tp);
    }
    if (!fo.pts.empty()) obs.push_back(std::move(fo));
  }
  std::printf("[selfcheck] frames=%d obs-pairs=%zu\n", nframes, obs.size());

  // ---- 反解（从 ts=0,td=0,bias=0 出发）----
  CalibConfig cfg;
  cfg.enumerate_permutations = false;   // 真值 P=I
  cfg.ts_init = 0.0; cfg.td_init = 0.0;
  CalibResult r = calibrate(imu, obs, fx, h, cfg);

  double e_ts   = std::fabs(r.ts - ts_true);
  double e_td   = std::fabs(r.td - td_true);
  double e_bias = (r.gyro_bias - bias_true).norm();

  std::printf("---- 反解 vs 真值 ----\n");
  std::printf("  ts   : %.6f  (true %.6f)  |err|=%.6f s\n", r.ts, ts_true, e_ts);
  std::printf("  td   : %.6f  (true %.6f)  |err|=%.6f s\n", r.td, td_true, e_td);
  std::printf("  bias : [%.5f %.5f %.5f] (true [%.5f %.5f %.5f]) |err|=%.6f\n",
              r.gyro_bias.x(), r.gyro_bias.y(), r.gyro_bias.z(),
              bias_true.x(), bias_true.y(), bias_true.z(), e_bias);
  std::printf("  rms  : %.4f px   final cost %.6g\n", r.rms_px, r.final_cost);

  // ---- 判据 ----
  const double TOL_TS = 1e-3;    // 1 ms（<< 30 ms 量级）
  const double TOL_TD = 1e-3;    // 1 ms
  const double TOL_BI = 2e-3;    // rad/s
  bool ok = (e_ts < TOL_TS) && (e_td < TOL_TD) && (e_bias < TOL_BI);
  std::printf("[selfcheck] %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
