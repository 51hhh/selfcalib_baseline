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
  double ransac_thresh = 1.0;           // 几何模型 RANSAC 阈值(像素)：1.0 严格门限，
                                        // 复现已提交差分 +1.28%（放松到 3px 会放进大视差点、抬高 RS 目估计）
  // 外点剔除几何模型：
  //   false(默认) = 基础矩阵 F —— 适合【含平移】数据(如 TUM-RSVI seq1，对极几何良定义)；
  //   true        = 单应 H     —— 适合【旋转主导/远景】数据(纯旋转下 F 退化，H 与假设自洽，
  //                              且天然剔除大视差点)。见 README §4b。
  bool use_homography = false;
};

// frames：按时间排序的图像路径；ts：与之一一对应的时间戳。
std::vector<RawPair> trackSequence(const std::vector<std::string>& frames,
                                   const std::vector<double>& ts,
                                   const TrackerOptions& opt = {});

}  // namespace karpenko
