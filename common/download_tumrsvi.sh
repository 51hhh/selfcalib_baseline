#!/usr/bin/env bash
# =============================================================================
# 下载 TUM Rolling-Shutter (TUM-RSVI) 数据集到 datasets/tum_rsvi/。
# 主页: https://cvg.cit.tum.de/data/datasets/rolling-shutter-dataset
# CDN : https://cdn3.vision.in.tum.de/rolling/
#
# 数据说明（已实地核实 2026-08-26）：
#   - 双目 1280×1024 @20Hz：cam0 = 全局快门(GS)，cam1 = 卷帘快门(RS)；同序列共享真值轨迹。
#   - IMU: Bosch BMI160，六轴(陀螺+加计) @200Hz，硬件同步；真值轨迹来自 OptiTrack 120Hz。
#   - 真值 line delay ≈ 29.4737 µs/行 → 整帧 t_RS = 1024 × 29.4737µs ≈ 0.03018 s（仅 RS 目 cam1）。
#   - 每段 bag 约 4.2 GB；标定段 calib-imu1.bag 约 4.9 GB。按需下载。
# 话题: /cam0/image_raw (GS), /cam1/image_raw (RS), /imu0。
#
# 用法:
#   bash download_tumrsvi.sh                 # 默认下载 seq1 + 标定段 + camchain（够跑通验证）
#   bash download_tumrsvi.sh seq1 seq2 seq3  # 指定若干序列
#   bash download_tumrsvi.sh all             # 全部 10 段（~42GB，慎用）
#   FORMAT=euroc bash download_tumrsvi.sh seq1   # 取 EuRoC/ASL tar 而非 rosbag
# curl -C - 断点续传，可重复运行。
# =============================================================================
set -euo pipefail

CDN="https://cdn3.vision.in.tum.de/rolling"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${SCRIPT_DIR}/../datasets/tum_rsvi"
FORMAT="${FORMAT:-bag}"   # bag | euroc
mkdir -p "$OUT"

dl() {  # dl <url> <dest>
  local url="$1" dest="$2"
  echo ">> $url"
  curl -f -L -C - --retry 3 -o "$dest" "$url" || { echo "下载失败: $url"; return 1; }
}

# 标定配置种子（小文件，始终获取）
dl "$CDN/calibration/camchain-calibration-equidistant4_camimu_dataset-calib-imu1.yaml" \
   "$OUT/camchain-calibration-equidistant4_camimu_dataset-calib-imu1.yaml" || true

# 解析参数
args=("$@")
if [ "${#args[@]}" -eq 0 ]; then
  args=(seq1 calib)
elif [ "${args[0]}" = "all" ]; then
  args=(seq1 seq2 seq3 seq4 seq5 seq6 seq7 seq8 seq9 seq10 calib)
fi

for name in "${args[@]}"; do
  case "$name" in
    calib)
      if [ "$FORMAT" = "euroc" ]; then
        dl "$CDN/exported/euroc/dataset-calib-cam1.tar"  "$OUT/dataset-calib-cam1.tar"
        dl "$CDN/exported/euroc/dataset-calib-imu1.tar"  "$OUT/dataset-calib-imu1.tar"
      else
        dl "$CDN/calibrated/dataset-calib-cam1.bag" "$OUT/dataset-calib-cam1.bag"
        dl "$CDN/calibrated/dataset-calib-imu1.bag" "$OUT/dataset-calib-imu1.bag"
      fi
      ;;
    seq*)
      if [ "$FORMAT" = "euroc" ]; then
        dl "$CDN/exported/euroc/dataset-${name}.tar" "$OUT/dataset-${name}.tar"
      else
        dl "$CDN/calibrated/dataset-${name}.bag" "$OUT/dataset-${name}.bag"
      fi
      ;;
    *) echo "未知参数: $name（用 seqN / calib / all）"; exit 1;;
  esac
done

echo "完成。数据在 $OUT"
echo "提示: RS 目 = /cam1/image_raw (期望 t_RS≈0.03018s)；GS 目 = /cam0/image_raw (期望 t_RS≈0)。"
