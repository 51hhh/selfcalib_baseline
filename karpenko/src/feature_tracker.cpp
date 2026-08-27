#include "feature_tracker.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

namespace karpenko {

std::vector<RawPair> trackSequence(const std::vector<std::string>& frames,
                                   const std::vector<double>& ts,
                                   const TrackerOptions& opt) {
  std::vector<RawPair> out;
  int n = (int)std::min(frames.size(), ts.size());
  cv::Mat prev;
  for (int i = 0; i < n - 1; ++i) {
    cv::Mat cur = cv::imread(frames[i], cv::IMREAD_GRAYSCALE);
    cv::Mat nxt = cv::imread(frames[i + 1], cv::IMREAD_GRAYSCALE);
    if (cur.empty() || nxt.empty()) continue;

    std::vector<cv::Point2f> p0;
    cv::goodFeaturesToTrack(cur, p0, opt.max_corners, opt.quality, opt.min_dist);
    if (p0.size() < 8) continue;

    std::vector<cv::Point2f> p1, p0b;
    std::vector<uchar> st1, st2; std::vector<float> err1, err2;
    cv::calcOpticalFlowPyrLK(cur, nxt, p0, p1, st1, err1);
    cv::calcOpticalFlowPyrLK(nxt, cur, p1, p0b, st2, err2);  // 前后向校验

    std::vector<cv::Point2f> a, b;
    for (size_t k = 0; k < p0.size(); ++k) {
      if (!st1[k] || !st2[k]) continue;
      if (cv::norm(p0[k] - p0b[k]) > opt.fb_thresh) continue;
      a.push_back(p0[k]); b.push_back(p1[k]);
    }
    if (a.size() < 8) continue;

    // 外点剔除：含平移数据用基础矩阵（对极几何良定义）；旋转主导/远景数据用单应
    // （纯旋转诱导无穷单应 K·R·K⁻¹，与假设自洽且天然剔大视差点，F 在纯旋转下退化）。
    std::vector<uchar> inl;
    if (opt.use_homography) cv::findHomography(a, b, cv::RANSAC, opt.ransac_thresh, inl);
    else cv::findFundamentalMat(a, b, cv::FM_RANSAC, opt.ransac_thresh, 0.99, inl);
    RawPair rp; rp.i = i; rp.j = i + 1; rp.ti = ts[i]; rp.tj = ts[i + 1];
    for (size_t k = 0; k < a.size(); ++k)
      if (k < inl.size() && inl[k]) { rp.pi.push_back(a[k]); rp.pj.push_back(b[k]); }
    if (rp.pi.size() >= 8) out.push_back(std::move(rp));
  }
  return out;
}

}  // namespace karpenko
