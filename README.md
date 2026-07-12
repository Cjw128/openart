# OpenART 双车视觉系统

当前正式版本：**v1.0.0（2026-07-13）**

本仓库提供智能车主车与从车的 OpenART Plus 单文件视觉程序。v1.0.0 完全重构世界坐标计算：原始 QVGA 图像不做 `lens_corr()` 或鸟瞰拉伸，网格内使用 28 点结构化三角插值，网格外使用带边界连续校正的全局单应后备补齐，标定范围覆盖 `6–164 cm`。16 字节 UART 包长和字段位置保持不变，坐标单位提升为 `0.01 cm`（`0.1 mm`）。

> **硬件约束：主车和从车现在都使用 OpenART Plus，两份正式程序均固定使用 `UART12`（115200 bps）。`main.py` 固定为主车入口，`minimain.py` 固定为从车入口，不再提供无效的运行时角色开关。**

## 正式入口

| 文件 | 用途 |
| --- | --- |
| `main.py` | 主车 OpenART Plus 脱机程序 |
| `minimain.py` | 从车 OpenART Plus 脱机程序 |

部署从车时，将 `minimain.py` 作为设备启动脚本使用。主车和从车均连接 OpenART Plus 的 `UART12`，正式部署保持单文件结构。

将匹配相机的 `camera_ground_mesh.txt` 放在 SD 卡根目录；文件缺失时程序会使用内置全图后备矩阵，精度低于 28 点网格。

## 文档

- [中文说明与更新日志](README_ch.md)
- [English notes and changelog](README_en.md)
