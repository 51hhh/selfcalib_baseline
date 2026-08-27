#include "io.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <dirent.h>

namespace karpenko {

cv::Mat CameraModel::K() const {
  cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
  k.at<double>(0, 0) = fx; k.at<double>(1, 1) = fy;
  k.at<double>(0, 2) = cx; k.at<double>(1, 2) = cy;
  return k;
}

cv::Mat CameraModel::D() const {
  cv::Mat d(1, (int)dist.size(), CV_64F);
  for (int i = 0; i < (int)dist.size(); ++i) d.at<double>(0, i) = dist[i];
  return d;
}

std::vector<cv::Point2f> CameraModel::undistortToNormalized(
    const std::vector<cv::Point2f>& px) const {
  std::vector<cv::Point2f> out;
  if (px.empty()) return out;
  if (dist_model == EQUI) {
    cv::Mat D4(1, 4, CV_64F);
    for (int i = 0; i < 4; ++i) D4.at<double>(0, i) = (i < (int)dist.size() ? dist[i] : 0.0);
    cv::fisheye::undistortPoints(px, out, K(), D4);      // 输出归一化坐标
  } else if (dist_model == RADTAN) {
    cv::undistortPoints(px, out, K(), D());              // 输出归一化坐标
  } else {  // NONE：直接反投影
    out.reserve(px.size());
    for (const auto& p : px)
      out.emplace_back((float)((p.x - cx) / fx), (float)((p.y - cy) / fy));
  }
  return out;
}

static YAML::Node camNode(const YAML::Node& root) {
  if (root["cam0"]) return root["cam0"];
  if (root["cam1"]) return root["cam1"];  // 右目布局
  return root;  // 顶层布局
}

CameraModel loadCamera(const std::string& yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);
  YAML::Node c = camNode(root);
  CameraModel m;
  auto intr = c["intrinsics"];
  if (!intr || intr.size() < 4) throw std::runtime_error("camera yaml: intrinsics 缺失/不足4");
  m.fx = intr[0].as<double>(); m.fy = intr[1].as<double>();
  m.cx = intr[2].as<double>(); m.cy = intr[3].as<double>();
  if (c["distortion_coeffs"])
    for (auto v : c["distortion_coeffs"]) m.dist.push_back(v.as<double>());
  // 畸变模型：优先 distortion_model；否则从 camera_model 后缀推断。
  std::string dm;
  if (c["distortion_model"]) dm = c["distortion_model"].as<std::string>();
  else if (c["camera_model"]) { std::string cm = c["camera_model"].as<std::string>();
    if (cm.find("equi") != std::string::npos) dm = "equidistant";
    else if (cm.find("radtan") != std::string::npos) dm = "radtan"; }
  if (dm.find("equi") != std::string::npos) m.dist_model = CameraModel::EQUI;
  else if (dm.find("radtan") != std::string::npos) m.dist_model = CameraModel::RADTAN;
  else m.dist_model = CameraModel::NONE;
  if (c["resolution"] && c["resolution"].size() >= 2) {
    m.width = c["resolution"][0].as<int>(); m.height = c["resolution"][1].as<int>();
  }
  if (c["rolling_shutter"]) m.rolling = c["rolling_shutter"].as<bool>();
  if (c["shutter"]) m.rolling = (c["shutter"].as<std::string>() == "rolling");
  if (c["camera_model"] && c["camera_model"].as<std::string>().find("-rs") != std::string::npos)
    m.rolling = true;
  if (c["line_delay"]) m.line_delay = c["line_delay"].as<double>();
  else if (c["line_delay_nanoseconds"]) m.line_delay = c["line_delay_nanoseconds"].as<double>() * 1e-9;
  return m;
}

std::vector<ImuSample> loadImu(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("无法打开 imu: " + path);
  std::vector<ImuSample> out;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::vector<double> v; double x;
    while (ss >> x) v.push_back(x);
    if (v.size() < 4) continue;  // 至少 t wx wy wz
    ImuSample s; s.t = v[0]; s.w = Eigen::Vector3d(v[1], v[2], v[3]);
    out.push_back(s);
  }
  std::sort(out.begin(), out.end(), [](const ImuSample& a, const ImuSample& b){ return a.t < b.t; });
  return out;
}

std::vector<double> loadTimestamps(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("无法打开时间戳: " + path);
  std::vector<double> out; std::string line;
  while (std::getline(f, line)) {
    std::istringstream ss(line); double t;
    if (ss >> t) out.push_back(t);
  }
  return out;
}

std::vector<std::string> listFrames(const std::string& dir) {
  std::vector<std::string> out;
  DIR* d = opendir(dir.c_str());
  if (!d) throw std::runtime_error("无法打开帧目录: " + dir);
  struct dirent* e;
  while ((e = readdir(d))) {
    std::string n = e->d_name;
    if (n.size() < 4) continue;
    std::string ext = n.substr(n.size() - 4);
    if (ext == ".png" || ext == ".jpg" || ext == ".PNG" || ext == ".JPG")
      out.push_back(dir + "/" + n);
  }
  closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

void writeResult(const std::string& path, const CameraModel& cam, const CalibResult& r) {
  std::ofstream f(path);
  f << "# Karpenko 2011 复现结果（自然场景纯陀螺离线标定）\n";
  f << "cam0:\n";
  f << "  camera_model: pinhole" << (cam.rolling ? "-rs" : "") << "\n";
  f << "  intrinsics: [" << cam.fx << ", " << cam.fy << ", " << cam.cx << ", " << cam.cy << "]\n";
  f << "  resolution: [" << cam.width << ", " << cam.height << "]\n";
  f << "  t_RS_seconds: " << r.ts << "        # 整帧读出时间 t_RS\n";
  if (cam.height > 0)
    f << "  line_delay: " << (r.ts / cam.height) << "   # 秒/行 = t_RS / height\n";
  f << "  timeshift_cam_imu: " << r.td << "   # td: gyro_ts = video_ts + td\n";
  f << "  gyro_bias: [" << r.gyro_bias.x() << ", " << r.gyro_bias.y() << ", " << r.gyro_bias.z() << "]\n";
  f << "  focal_px: " << r.f << "\n";
  f << "  final_cost: " << r.final_cost << "\n";
  f << "  rms_reproj_px: " << r.rms_px << "\n";
  f << "  R_cam_gyro:\n";
  for (int i = 0; i < 3; ++i)
    f << "    - [" << r.R_cam_gyro(i,0) << ", " << r.R_cam_gyro(i,1) << ", " << r.R_cam_gyro(i,2) << "]\n";
}

}  // namespace karpenko
