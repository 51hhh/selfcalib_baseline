// io.h —— Karpenko 复现的输入/输出层。
// 数据格式沿用 rscalib 的约定（见 ../rscalib/docs/02-IO层-数据格式协定.md）：
//   - 帧序列 (cam0/%06d.png) 或视频 + video_ts.txt（每行一个秒级时间戳）
//   - imu.txt：7 列 `t wx wy wz ax ay az`（秒, rad/s, m/s^2）或 4 列 `t wx wy wz`；只用陀螺
//   - 相机内参 yaml：kalibr camchain 布局（cam0:）或顶层布局
// 轻量自包含：仅依赖 Eigen + OpenCV + yaml-cpp，无 ROS、无 aslam。
#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace karpenko {

// 陀螺样本（陀螺时钟域；本复现约定 gyro_ts = video_ts + td）。
struct ImuSample {
  double t;              // 秒
  Eigen::Vector3d w;     // rad/s
};

// 相机内参 + 卷帘信息。畸变模型仅支持 pinhole 下的 equidistant(鱼眼) / radtan / none。
struct CameraModel {
  enum Dist { NONE, EQUI, RADTAN };
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::vector<double> dist;    // equi:4  radtan:4/5/8
  Dist dist_model = NONE;
  int width = 0, height = 0;
  bool rolling = false;
  double line_delay = 0.0;     // 秒/行（若 yaml 提供；本方法直接估整帧 ts，可忽略）

  cv::Mat K() const;           // 3x3 CV_64F
  cv::Mat D() const;           // 畸变向量 CV_64F
  // 像素 -> 归一化平面坐标 (x,y)（已去畸变）。批量。
  std::vector<cv::Point2f> undistortToNormalized(const std::vector<cv::Point2f>& px) const;
};

// 读取相机 yaml（cam0 段或顶层）。失败抛 std::runtime_error。
CameraModel loadCamera(const std::string& yaml_path);

// 读取 imu（7 列或 4 列），只保留陀螺。以 # 开头行跳过；列数不足丢弃。按时间排序。
std::vector<ImuSample> loadImu(const std::string& path);

// 读取时间戳文件（每行一个秒级 double）。
std::vector<double> loadTimestamps(const std::string& path);

// 列出帧序列文件（目录下按名字排序的图像）；用于特征跟踪。
std::vector<std::string> listFrames(const std::string& dir);

// 结果写出（camchain 兼容片段）。
struct CalibResult {
  double ts = 0;      // 整帧读出时间 t_RS（秒）
  double td = 0;      // 相机-陀螺时间偏移（秒），gyro_ts = video_ts + td
  double f = 0;       // 焦距（像素；若固定 = fx）
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_cam_gyro = Eigen::Matrix3d::Identity();  // 选定的轴排列/符号
  double final_cost = 0;
  double rms_px = 0;
};
void writeResult(const std::string& path, const CameraModel& cam, const CalibResult& r);

}  // namespace karpenko
