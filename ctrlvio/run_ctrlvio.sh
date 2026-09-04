#!/usr/bin/env bash
# 离线在自然场景下用 Ctrl-VIO 标定卷帘每行延时 line_delay（TUM-RSVI）。
# 无显示环境：用 rosrun 直接跑 odometry_node（绕开 launch 里 required=true 的 rviz）。
# docker 本机需 sudo。
#
# 用法：
#   cd ctrlvio
#   ./run_ctrlvio.sh <bag绝对路径或相对datasets的路径> # 默认 tum_rsvi/dataset-seq1.bag
#   CTRLVIO_CONFIG=/path/config.yaml SELFCALIB_OUTPUT_DIR=/path/output ./run_ctrlvio.sh /path/input.bag
#
# 数据放置：把 TUM-RSVI 的 dataset-seqX.bag 放到  selfcalib_baseline/datasets/tum_rsvi/
#   （下载见 ../common/download_tumrsvi.sh 或 README）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"                 # ctrlvio/
ROOT="$(cd "$HERE/.." && pwd)"                                       # selfcalib_baseline/
read -r -a DOCKER_CMD <<< "${DOCKER:-sudo docker}"
IMAGE="${IMAGE:-ctrlvio:melodic}"

BAG_ARG="${1:-tum_rsvi/dataset-seq1.bag}"
if [[ "$BAG_ARG" = /* ]]; then BAG_HOST="$BAG_ARG";
else BAG_HOST="$ROOT/datasets/$BAG_ARG"; fi
if [ ! -f "$BAG_HOST" ]; then
  echo "找不到 bag: $BAG_HOST" >&2
  echo "请先下载 TUM-RSVI 到 datasets/tum_rsvi/（见 README / ../common/download_tumrsvi.sh）。" >&2
  exit 1
fi

TAG="$(basename "$BAG_HOST" .bag)"
OUT="${SELFCALIB_OUTPUT_DIR:-$ROOT/results/ctrlvio/$TAG}"
mkdir -p "$OUT"
BAG_DIR=$(dirname "$BAG_HOST")
BAG_BASE=$(basename "$BAG_HOST")
CONFIG_HOST="${CTRLVIO_CONFIG:-$ROOT/ctrlvio/config/ct_odometry_tumrs.yaml}"
[[ -f "$CONFIG_HOST" ]] || { echo "找不到 config: $CONFIG_HOST" >&2; exit 1; }

# 容器内：起 roscore → rosrun 节点（自己离线读 bag）→ 输出 tee 到 results
"${DOCKER_CMD[@]}" run --rm \
  -e config_path=/work/config.yaml \
  -v "$CONFIG_HOST":/work/config.yaml:ro \
  -v "$BAG_DIR":/work/data:ro \
  -v "$OUT":/work/results \
  "$IMAGE" \
  bash -lc '
    source /opt/ros/melodic/setup.bash
    source /catkin_ctrlvio/devel/setup.bash
    roscore >/work/results/roscore.log 2>&1 &
    for i in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
    rosrun ctrlvio odometry_node \
      _config_path:=/work/config.yaml \
      _bag_path:=/work/data/'"$BAG_BASE"' \
      2>&1 | tee /work/results/run.log
  '

echo "==================================================================="
echo "结果目录：$OUT"
echo "控制台每行延时（收敛取末值）："
grep -a "estimated line delay" "$OUT/run.log" | tail -5 || echo "  未捕获，检查 run.log"
echo "-------------------------------------------------------------------"
echo "换算：整帧 t_RS = line_delay(每行) × image_height(1024)"
echo "TUM-RSVI RS 目真值参考：每行 ≈ 29.4737us → 整帧 ≈ 0.03018s；GS 目应 ≈ 0"
echo "glog 里的 [line_delay]（秒）见 $OUT 下 *.INFO 日志"
