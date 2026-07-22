# OpenART 双车视觉系统

当前版本：**v0.11.0（2026-07-22）省赛 04 无优先级多点世界坐标版**

本版本以实车使用的 `04_备用版_无优先级_5中7` 为基线：自动搜索不按颜色 ID 排序，优先锁定世界 Y 最近的目标，并保留 `7` 个推理帧命中 `5` 帧的首次确认。模型固定为 `/sd/80lite0.5SS.tflite`，默认曝光为 `880 us`。

## 部署文件

| 仓库文件 | 板端位置 | 用途 |
| --- | --- | --- |
| `main.py` | 主车 `/sd/main.py` | 主车正式入口 |
| `minimain.py` | 从车 `/sd/main.py` | 从车正式入口 |
| `camera_ground_mesh.txt` | 两车 `/sd/camera_ground_mesh.txt` | 28 点地面坐标网格 |
| `80lite0.5SS.tflite` | 两车 `/sd/80lite0.5SS.tflite` | 最终 SS 模型，模型文件不纳入 Git |

若板端保留 `/sd/color_thr.txt`，其中的 `exposure_us=` 会覆盖默认 `880 us`。主从均为 OpenART Plus，使用 `UART12`、`115200 bps`。

当前主从程序共用 `role=master` 网格；若两块相机的安装几何不同，必须分别采点并为从车生成独立网格。

## 世界坐标

- 标定源为 `ground_mesh_24_points_template.csv`，文件名沿用旧名，实际包含 `7 x 4 = 28` 个点。
- 图像链路固定为 QVGA、`vflip=True`、软件水平镜像、不使用 `lens_corr()`。
- 目标底边中点 `x + w/2, y + h - 0.5` 作为地面接触点。
- 网格内使用 36 个三角形插值，网格外使用全局单应矩阵回退。
- 板内坐标单位为厘米，16 字节 UART 包中的 X/Y 仍发送**毫米**。

启动时应看到：

```text
[GROUND] loaded 36 triangles from /sd/camera_ground_mesh.txt
```

缺少或拒绝网格时只会使用精度较低的全局单应回退，不应作为正式部署状态。

## 生成与验证

```powershell
python calibrate_ground_camera.py --ground-csv ground_mesh_24_points_template.csv --role master --expected-points 28 --required-near-y-cm 6 --max-y-cm 164 --output camera_ground_mesh.txt --report camera_ground_mesh_report.json
python -m unittest -v test_ground_projection.py
python -m py_compile main.py minimain.py calibrate_ground_camera.py raw_ground_projection_test.py test_ground_projection.py
git diff --check
```

`raw_ground_projection_test.py` 用于 OpenART IDE 实地采点和复核坐标。完整版本说明、误差数据和历史记录见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。
