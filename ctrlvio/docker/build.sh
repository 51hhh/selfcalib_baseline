#!/usr/bin/env bash
# 构建 Ctrl-VIO 复现镜像。docker 本机需 sudo（与 kalibr_baseline 一致）。
# 用法：cd ctrlvio && ./docker/build.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # ctrlvio/
DOCKER="${DOCKER:-sudo docker}"

if [ ! -s "$HERE/docker/bundles/ctrlvio.bundle" ]; then
  echo "缺少 docker/bundles/ctrlvio.bundle（源码 provenance）。" >&2
  exit 1
fi

cd "$HERE"
$DOCKER build -t ctrlvio:melodic -f docker/Dockerfile.melodic .
echo "OK: 镜像 ctrlvio:melodic 构建完成。下一步见 ../ctrlvio/README.md 运行章节。"
