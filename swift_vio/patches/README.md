# patches/

从 bundle 还原的 swift_vio / vio_common / okvis 源码**开箱即可编译**（Software Heritage
存档为完整快照，非残缺镜像），无需任何修补。故本目录当前为空。

若日后为适配新编译器 / 依赖版本产生改动，按 kalibr_baseline 范式导出为 `*.patch`（对
`restore_swift_vio.sh` 还原后的源码 `git diff` 生成），并在 restore 脚本末尾 `git apply`。
