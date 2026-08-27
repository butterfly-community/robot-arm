# OpenHMD 历史补丁

当前运行链不使用 OpenHMD、Monado 或 OpenXR。本目录只保留
[`patches/openhmd-nolo-cv1.patch`](patches/openhmd-nolo-cv1.patch)，用于追溯早期 NOLO
协议分析的来源；旧服务、构建脚本和测试已在迁移到 Rust USB 主线后删除。

如果需要复现实验，应把 OpenHMD 公开仓库克隆到 `~/Develop/temp/`，再手工检查补丁是否
仍适用于目标版本。这里不提供备用启动路径，也不参与当前构建或测试。
