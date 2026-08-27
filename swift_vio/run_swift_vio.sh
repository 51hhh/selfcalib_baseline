#!/usr/bin/env bash
# ============================================================================
# run_swift_vio.sh —— swift_vio / KSWF 离线联合标定 (t_RS + td) 运行入口
#
# 在容器 swift_vio:melodic 内跑同步 rosbag 离线标定（swift_vio_node_synchronous），
# dump_output_option=3 把「全部标定量」(含 readout t_r / td / 内外参 / IMU 内参) 导出到
# output_dir 下的 csv。宿主机执行本脚本会自动 docker run 进容器。
#
# 用法：
#   ./run_swift_vio.sh <bag路径> [config.yaml] [cam0话题] [cam1话题] [imu话题]
# 默认：
#   config = config/config_tum_rs_calib.yaml   (已开 sigma_td/sigma_tr)
#   话题   = /cam0/image_raw /cam1/image_raw /imu0   (TUM-RSVI rosbag 约定)
# 输出：  results/<bag名>/  下的 swift_vio 状态与标定 csv
#
# 数据来源二选一：
#   (A) TUM-RSVI 官方 rosbag（真值 t_RS≈0.03018s，右目 RS / 左目 GS）—— 主测。
#   (B) GS sanity：把 ../real_frames/uzh_cam0（帧+video_ts.txt+imu.txt）用
#       common/frames_to_rosbag.py 转成单目 bag，期望标出 t_RS≈0（负样本）。
#       转 bag 时相机话题写 /cam0/image_raw、IMU 写 /imu0，config 用单目变体
#       （monocular_input: true，删掉 cam1）。
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER="${DOCKER:-sudo docker}"
IMAGE="${IMAGE:-swift_vio:melodic}"

BAG="${1:?用法: run_swift_vio.sh <bag路径> [config] [cam0] [cam1] [imu]}"
CONFIG="${2:-config/config_tum_rs_calib.yaml}"
CAM0="${3:-/cam0/image_raw}"
CAM1="${4:-/cam1/image_raw}"
IMU="${5:-/imu0}"

[ -f "$BAG" ]            || { echo "找不到 bag: $BAG" >&2; exit 1; }
[ -f "$HERE/$CONFIG" ]   || { echo "找不到 config: $HERE/$CONFIG" >&2; exit 1; }

BAGNAME="$(basename "$BAG" .bag)"
OUTDIR="results/$BAGNAME"
mkdir -p "$HERE/$OUTDIR"

# 把 swift_vio/ 挂到 /work，bag 目录挂到 /data
BAGDIR="$(cd "$(dirname "$BAG")" && pwd)"
BAGBASE="$(basename "$BAG")"

echo ">> swift_vio 离线标定"
echo "   bag    : $BAG"
echo "   config : $CONFIG"
echo "   topics : $CAM0 , $CAM1 , $IMU"
echo "   output : $OUTDIR/  (dump_output_option=3 -> 全部标定量 csv)"

$DOCKER run --rm -it \
  -v "$HERE":/work \
  -v "$BAGDIR":/data \
  "$IMAGE" bash -lc "
    source /swift_vio_ws/devel/setup.bash && cd /work &&
    rosrun swift_vio swift_vio_node_synchronous /work/$CONFIG \
      --bagname=/data/$BAGBASE \
      --camera_topics='$CAM0,$CAM1' \
      --imu_topic='$IMU' \
      --load_input_option=1 \
      --dump_output_option=3 \
      --output_dir=/work/$OUTDIR 2>&1 | tee /work/$OUTDIR/run.log
  "

echo
echo ">> 完成。查看结果："
echo "   grep -i 'readout\\|time.*offset\\|td' $OUTDIR/run.log"
echo "   ls $OUTDIR/*.csv   # 末行即收敛标定量（readout t_r、td、内外参、IMU 内参）"
