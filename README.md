# OpenART 双车视觉系统

最近稳定版本：**v0.9.0（2026-07-10）**

当前开发版本：**v0.9.7-dev（2026-07-11）**

本仓库提供智能车主车与从车的 OpenART Plus 单文件视觉程序。v0.9.0 已补全双车候选颜色上报、RT1021 主控仲裁和两侧 `0x03` 最终颜色锁定链路；当前开发版继续调整运行性能、动态裁切和现场检测阈值。

> **硬件约束：主车和从车现在都使用 OpenART Plus，两份正式程序均固定使用 `UART12`（115200 bps）。`IS_SLAVE_CAR` 只区分软件角色，不用于选择 UART2。**

## 正式入口

| 文件 | 用途 |
| --- | --- |
| `main.py` | 主车 OpenART Plus 脱机程序 |
| `minimain.py` | 从车 OpenART Plus 脱机程序 |

部署从车时，将 `minimain.py` 作为设备启动脚本使用。主车和从车均连接 OpenART Plus 的 `UART12`，正式部署保持单文件结构。

## 文档

- [中文说明与更新日志](README_ch.md)
- [English notes and changelog](README_en.md)
