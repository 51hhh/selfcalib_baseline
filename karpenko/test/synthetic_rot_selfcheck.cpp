// synthetic_rot_selfcheck.cpp —— 校验「连续 R_cam_gyro 微旋转精修」。
// 注入一个【非纯轴排列】的相机-陀螺失配 P_true = expSO3(tilt)（tilt~几度），
// 按物理约定 b=R(t)^T c 生成观测。离散枚举只能取到最近的轴排列（此处 I），残留 tilt
// 会偏置 ts；开启 refine_rotation 后应把 tilt 与 ts 一并解回真值。
// 断言：refine 的 ts 误差 << 离散-only 的 ts 误差，且 refine ts 误差 < 1ms。
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <Eigen/Geometry>
#include "gyro_integrator.h"
#include "objective.h"
#include "calibrate.h"
using namespace karpenko;

int main() {
  const double ts_true = 0.030, td_true = 0.006;
  const Eigen::Vector3d bias_true(0.004, -0.002, 0.003);
  const Eigen::Vector3d tilt(0.035, -0.025, 0.045);  // ~几度的相机-陀螺失配（非轴排列）
  const int h = 1024; const double fx = 1000.0;
  const int nframes = 30; const double fps = 30.0; const int npts = 40;

  std::vector<ImuSample> imu; const double hz = 200.0, dt = 1.0 / hz;
  for (double t = -0.5; t <= 6.0; t += dt) {
    Eigen::Vector3d w(0.30*std::sin(2*M_PI*0.7*t+0.3), 0.25*std::sin(2*M_PI*1.1*t+1.1),
                      0.20*std::sin(2*M_PI*0.5*t+2.0));
    imu.push_back({t, w});
  }
  Eigen::Matrix3d P_true = Eigen::AngleAxisd(tilt.norm(), tilt.normalized()).toRotationMatrix();
  GyroIntegrator truth; truth.setSamples(imu); truth.setPermutation(P_true);
  truth.setBias(bias_true); truth.build();

  std::mt19937 rng(999);
  std::uniform_real_distribution<double> urow(0.0,(double)h), uang(-0.4,0.4);
  std::vector<FrameObs> obs;
  for (int i = 0; i < nframes-1; ++i) {
    double ti=i/fps, tj=(i+1)/fps; FrameObs fo; fo.ti=ti; fo.tj=tj;
    for (int k=0;k<npts;++k){
      double vi=urow(rng), vj=urow(rng);
      Eigen::Vector3d c(uang(rng),uang(rng),1.0); c.normalize();
      double tir=ti+ts_true*(vi/h), tjr=tj+ts_true*(vj/h);
      Eigen::Vector3d bi=(truth.R(tir,td_true).transpose()*c).normalized();
      Eigen::Vector3d bj=(truth.R(tjr,td_true).transpose()*c).normalized();
      if(bi.z()<=1e-6||bj.z()<=1e-6) continue;
      TrackedPoint tp; tp.vi=vi; tp.vj=vj; tp.bi=bi; tp.bj=bj; fo.pts.push_back(tp);
    }
    if(!fo.pts.empty()) obs.push_back(std::move(fo));
  }

  CalibConfig base; base.enumerate_permutations = true; base.ts_init=0; base.td_init=0;
  CalibResult r_disc = calibrate(imu, obs, fx, h, base);         // 离散枚举 only
  CalibConfig ref = base; ref.refine_rotation = true;
  CalibResult r_ref  = calibrate(imu, obs, fx, h, ref);          // + 连续微旋转精修

  double e_disc = std::fabs(r_disc.ts - ts_true);
  double e_ref  = std::fabs(r_ref.ts  - ts_true);
  std::printf("注入 tilt=%.3f rad (%.2f deg)\n", tilt.norm(), tilt.norm()*180/M_PI);
  std::printf("  离散枚举 only : ts=%.6f err=%.6f rms=%.4f\n", r_disc.ts, e_disc, r_disc.rms_px);
  std::printf("  +微旋转精修   : ts=%.6f err=%.6f rms=%.4f  td=%.6f(true %.6f)\n",
              r_ref.ts, e_ref, r_ref.rms_px, r_ref.td, td_true);

  bool ok = (e_ref < 1e-3) && (e_ref < 0.5 * e_disc + 1e-9);
  std::printf("[rot_selfcheck] %s  (精修显著优于离散: %.6f -> %.6f)\n",
              ok ? "PASS" : "FAIL", e_disc, e_ref);
  return ok ? 0 : 1;
}
