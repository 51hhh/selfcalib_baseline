#!/usr/bin/env bash
# ============================================================================
# build.sh —— 构建 swift_vio ROS1 Melodic 镜像
#   本机 docker 需 sudo；可用 DOCKER 环境变量覆盖（如 DOCKER="podman"）。
#   构建上下文 = swift_vio/（含 docker/Dockerfile.melodic 与 ws/src 源码）。
#
# 前置：先还原源码到构建上下文能 COPY 到的位置
#   ./restore_swift_vio.sh docker/ws/src
#
# 可选：WITH_GTSAM=ON 装 gtsam 以启用 SlidingWindowSmoother（本标定用 HybridFilter，默认 OFF）
# 用法：  ./docker/build.sh
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # .../swift_vio/docker
CTX="$(cd "$HERE/.." && pwd)"                                  # .../swift_vio
DOCKER="${DOCKER:-sudo docker}"
IMAGE="${IMAGE:-swift_vio:melodic}"
WITH_GTSAM="${WITH_GTSAM:-OFF}"

if [ ! -d "$CTX/ws/src/swift_vio" ]; then
  echo ">> 源码尚未还原，先执行 restore_swift_vio.sh ..."
  "$CTX/restore_swift_vio.sh" "$CTX/ws/src"
fi

GTSAM_ARGS=""
[ "$WITH_GTSAM" = "ON" ] && GTSAM_ARGS="-DGTSAM_DIR=/usr/local/lib/cmake/GTSAM"

echo ">> 构建镜像 $IMAGE  (WITH_GTSAM=$WITH_GTSAM)"
$DOCKER build \
  -f "$HERE/Dockerfile.melodic" \
  -t "$IMAGE" \
  --build-arg WITH_GTSAM="$WITH_GTSAM" \
  --build-arg GTSAM_ARGS="$GTSAM_ARGS" \
  "$CTX"

echo ">> 完成：$IMAGE"
