// feature_tracker.h —— 相邻帧 KLT 光流跟踪 + 基础矩阵 RANSAC 剔外点。
// rscalib 无自然特征跟踪（纯标定板角点），故此处新写。输出原始像素对，
// 由上层去畸变为 bearing。
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace karpenko {

struct RawPair {
  int i, j;
  double ti, tj;                        // 两帧时间戳（秒）
  std::vector<cv::Point2f> pi, pj;      // 匹配像素对
};

struct TrackerOptions {
  int max_corners = 500;
  double quality = 0.01;
  double min_dist = 12.0;
  double fb_thresh = 1.5;               // 前后向光流一致性阈值(像素)
  double ransac_thresh = 1.0;           // 基础矩阵 RANSAC 阈值(像素)
};

// frames：按时间排序的图像路径；ts：与之一一对应的时间戳。
std::vector<RawPair> trackSequence(const std::vector<std::string>& frames,
                                   const std::vector<double>& ts,
                                   const TrackerOptions& opt = {});

}  // namespace karpenko
