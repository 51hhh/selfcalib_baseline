// main.cpp —— Karpenko 复现 CLI。
// 用法：
//   karpenko_calib --cam cam.yaml --frames <dir> --ts video_ts.txt --imu imu.txt \
//                  [--out result.yaml] [--enum-perm] [--ts-init S] [--td-init S] \
//                  [--refine-rot] [--huber <px>] [--homography]
// 提升开关（有效域=旋转主导/远景数据，平移主导数据默认关，见 README §4b）：
//   --refine-rot   在最优离散轴排列附近再优化 3 维 so(3) 微旋转（去 T_SC 非轴对齐偏置）
//   --huber <px>   残差改 Huber 鲁棒核，降权近景大视差外点
//   --homography   前端外点剔除改用单应 RANSAC（默认基础矩阵 F，适合含平移数据）
#include <cstdio>
#include <cstring>
#include <string>
#include "io.h"
#include "feature_tracker.h"
#include "calibrate.h"

using namespace karpenko;

static std::string arg(int argc, char** argv, const char* key, const std::string& def = "") {
  for (int i = 1; i < argc - 1; ++i) if (!std::strcmp(argv[i], key)) return argv[i + 1];
  return def;
}
static bool flag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], key)) return true;
  return false;
}

int main(int argc, char** argv) {
  std::string cam_yaml = arg(argc, argv, "--cam");
  std::string frames_dir = arg(argc, argv, "--frames");
  std::string ts_file = arg(argc, argv, "--ts");
  std::string imu_file = arg(argc, argv, "--imu");
  std::string out = arg(argc, argv, "--out", "result_karpenko.yaml");
  if (cam_yaml.empty() || frames_dir.empty() || ts_file.empty() || imu_file.empty()) {
    std::fprintf(stderr,
      "用法: %s --cam cam.yaml --frames <dir> --ts video_ts.txt --imu imu.txt "
      "[--out result.yaml] [--enum-perm] [--ts-init S] [--td-init S] "
      "[--refine-rot] [--huber <px>] [--homography]\n", argv[0]);
    return 2;
  }

  CameraModel cam = loadCamera(cam_yaml);
  std::vector<double> ts = loadTimestamps(ts_file);
  std::vector<ImuSample> imu = loadImu(imu_file);
  std::vector<std::string> frames = listFrames(frames_dir);
  std::printf("[karpenko] frames=%zu ts=%zu imu=%zu  %dx%d rolling=%d\n",
              frames.size(), ts.size(), imu.size(), cam.width, cam.height, (int)cam.rolling);

  // 1. 特征跟踪（原始像素对）
  TrackerOptions topt;
  topt.use_homography = flag(argc, argv, "--homography");
  std::vector<RawPair> raw = trackSequence(frames, ts, topt);
  std::printf("[karpenko] tracked frame-pairs=%zu  (geom=%s)\n",
              raw.size(), topt.use_homography ? "homography" : "fundamental");

  // 2. 去畸变为 bearing，组装 FrameObs
  std::vector<FrameObs> obs;
  long total_pts = 0;
  for (const auto& rp : raw) {
    auto ni = cam.undistortToNormalized(rp.pi);
    auto nj = cam.undistortToNormalized(rp.pj);
    FrameObs fo; fo.ti = rp.ti; fo.tj = rp.tj;
    for (size_t k = 0; k < ni.size(); ++k) {
      TrackedPoint tp;
      tp.vi = rp.pi[k].y; tp.vj = rp.pj[k].y;
      tp.bi = Eigen::Vector3d(ni[k].x, ni[k].y, 1.0).normalized();
      tp.bj = Eigen::Vector3d(nj[k].x, nj[k].y, 1.0).normalized();
      fo.pts.push_back(tp);
    }
    total_pts += (long)fo.pts.size();
    obs.push_back(std::move(fo));
  }
  std::printf("[karpenko] total tracked points=%ld\n", total_pts);
  if (obs.empty()) { std::fprintf(stderr, "无有效观测，退出\n"); return 1; }

  // 3. 标定
  CalibConfig cfg;
  cfg.enumerate_permutations = flag(argc, argv, "--enum-perm");
  cfg.refine_rotation = flag(argc, argv, "--refine-rot");
  std::string tsi = arg(argc, argv, "--ts-init"), tdi = arg(argc, argv, "--td-init");
  std::string hub = arg(argc, argv, "--huber");
  if (!tsi.empty()) cfg.ts_init = std::stod(tsi);
  if (!tdi.empty()) cfg.td_init = std::stod(tdi);
  if (!hub.empty()) cfg.huber_delta = std::stod(hub);
  std::printf("[karpenko] opts: enum_perm=%d refine_rot=%d huber=%.2fpx\n",
              (int)cfg.enumerate_permutations, (int)cfg.refine_rotation, cfg.huber_delta);
  CalibResult r = calibrate(imu, obs, cam.fx, cam.height, cfg);

  std::printf("\n===== Karpenko 结果 =====\n");
  std::printf("  t_RS(ts)     = %.6f s  (%.2f us/行)\n", r.ts,
              cam.height > 0 ? r.ts / cam.height * 1e6 : 0.0);
  std::printf("  td           = %.6f s\n", r.td);
  std::printf("  gyro_bias    = [%.5f %.5f %.5f] rad/s\n",
              r.gyro_bias.x(), r.gyro_bias.y(), r.gyro_bias.z());
  std::printf("  rms reproj   = %.4f px  (原始SSE)\n", r.rms_px);
  std::printf("  inlier ratio = %.1f%%  (残差<=huber_delta)\n", r.inlier_ratio * 100.0);
  std::printf("  final cost   = %.4f\n", r.final_cost);
  writeResult(out, cam, r);
  std::printf("  -> 写入 %s\n", out.c_str());
  return 0;
}
