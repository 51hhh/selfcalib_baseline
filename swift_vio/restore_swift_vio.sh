#!/usr/bin/env bash
# ============================================================================
# restore_swift_vio.sh —— 从自包含 bundle 还原 swift_vio / KSWF 源码树
#
# 原仓库 github.com/JzHuai0108/swift_vio 与镜像 wbl1997/my_swift_vio 均已 404。
# 本脚本从 Software Heritage 存档恢复并打包的 3 个 bundle 还原可编译源码树，
# 不依赖任何 upstream 存活（见 README §1 恢复溯源）。
#
# 布局（catkin 工作区 src/）：
#   src/swift_vio/                     <- swift_vio.bundle  (master d502c14)
#   src/swift_vio/thirdparty/okvis/    <- okvis.bundle      (master 65e30d6, .gitmodules 指定)
#   src/vio_common/                    <- vio_common.bundle (master 2a8e604)
#
# 用法：  ./restore_swift_vio.sh [目标目录]     默认 ./ws/src
# ============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLES="$HERE/docker/bundles"
DEST="${1:-$HERE/ws/src}"

# 期望树哈希（provenance 校验；bundle 快照 SHA 可与 upstream 不同，但树内容逐位一致）
TREE_SWIFT_VIO="db04037d69b91de822440699beac2dd11532c49c"   # swift_vio  master d502c14
TREE_VIO_COMMON="29499c1fc209fb32afc4682998c6fb4caa52c5d2"  # vio_common master 2a8e604
TREE_OKVIS="2c666ed248c7598001dba6b87b3711b2210e235b"       # okvis      master 65e30d6

for b in swift_vio vio_common okvis; do
  [ -f "$BUNDLES/$b.bundle" ] || { echo "缺少 $BUNDLES/$b.bundle" >&2; exit 1; }
  git bundle verify "$BUNDLES/$b.bundle" >/dev/null || { echo "$b.bundle 校验失败" >&2; exit 1; }
done

mkdir -p "$DEST"

restore() {  # <bundle名> <目标路径> <分支> <期望树哈希>
  local name="$1" path="$2" branch="$3" want="$4"
  echo ">> 还原 $name -> $path (branch $branch)"
  rm -rf "$path"
  git clone -q --branch "$branch" "$BUNDLES/$name.bundle" "$path"
  local got; got="$(git -C "$path" rev-parse "HEAD^{tree}")"
  if [ "$got" != "$want" ]; then
    echo "!! $name 树哈希不匹配: got=$got want=$want" >&2; exit 1
  fi
  echo "   树哈希 OK: $got"
}

restore swift_vio  "$DEST/swift_vio"  master "$TREE_SWIFT_VIO"
restore vio_common "$DEST/vio_common" master "$TREE_VIO_COMMON"
# okvis 是 swift_vio 的 .gitmodules 子模块 (thirdparty/okvis)
restore okvis      "$DEST/swift_vio/thirdparty/okvis" master "$TREE_OKVIS"

echo
echo "完成。catkin 工作区 src 已就绪：$DEST"
echo "  swift_vio/  vio_common/  swift_vio/thirdparty/okvis/"
echo "下一步：docker/build.sh 构建镜像（容器内 catkin build），或见 README §2。"
