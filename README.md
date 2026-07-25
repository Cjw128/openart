# OpenART 双车视觉系统

当前版本：**v0.11.4-dev（2026-07-25）近距离接触点稳定版**

本开发版基于已归档的省赛 v0.11.0：模型负责识别、类别确认和重捕，动态色块确认后负责完整显示框，其底边接触点经独立稳定后用于世界坐标。省赛基线保存在提交 `41260c0` 以及分支 `dedicated-model`、`archive/v0.11.0-ground-mesh`；模型仍为 `/sd/80lite0.5SS.tflite`，默认曝光仍为 `880 us`。

## 部署文件

| 仓库文件 | 板端位置 | 用途 |
| --- | --- | --- |
| `main.py` | 主车 `/sd/main.py` | 主车正式入口 |
| `minimain.py` | 从车 `/sd/main.py` | 从车正式入口 |
| `camera_ground_mesh.txt` | 两车 `/sd/camera_ground_mesh.txt` | 28 点地面坐标网格 |
| `80lite0.5SS.tflite` | 两车 `/sd/80lite0.5SS.tflite` | 最终 SS 模型，模型文件不纳入 Git |

若板端保留 `/sd/color_thr.txt`，其中的 `exposure_us=` 会覆盖默认 `880 us`。主从均为 OpenART Plus，使用 `UART12`、`115200 bps`。

当前主从程序共用 `role=master` 网格；若两块相机的安装几何不同，必须分别采点并为从车生成独立网格。

## ID2 绝对优先

三份当前入口顶部均有 `ID2_ABSOLUTE_PRIORITY` 开关，默认值为 `True`，行为与省赛稳定版一致：上电或收到 `0x08` 清零后，普通搜索和主控指定只允许 ID2；ID2 完成后，ID1/ID3/ID4/ID5 恢复按世界 Y 最近目标竞争。`0x06` 全色前扫不受该门控影响。

设为 `False` 时取消颜色优先级，所有未完成 ID 从一开始就按最近目标竞争。开启绝对优先但场上没有 ID2 时，程序会持续等待 ID2，不会退而选择其他类别。

## 世界坐标

- 标定源为 `ground_mesh_24_points_template.csv`，文件名沿用旧名，实际包含 `7 x 4 = 28` 个点。
- 图像链路固定为 QVGA、`vflip=True`、软件水平镜像、不使用 `lens_corr()`。
- 目标底边中点 `x + w/2, y + h - 0.5` 作为地面接触点。
- 接触点默认使用 `2 px` 空间死区抑制近距离色块底边抖动；单次偏移达到 `24 px` 时直接重置，不产生长时间滤波拖尾。参数为 `COORDINATE_CONTACT_DEADBAND_PX` 和 `COORDINATE_CONTACT_RESET_PX`。
- 网格内使用 36 个三角形插值，网格外使用全局单应矩阵回退。
- 板内坐标单位为厘米，16 字节 UART 包中的 X/Y 仍发送**毫米**。

### 全类别坐标观察

在 OpenART IDE 中运行 `world_coordinate_test.py`。该脚本保留 `main.py` 的完整模型、五色 LAB、模型/色块融合、最近目标选择和世界坐标逻辑，只增加屏幕标记与串口终端打印。SD 卡仍需放置正式运行使用的模型、`camera_ground_mesh.txt` 和可选的 `color_thr.txt`。

主车、从车和测试脚本均默认启用 `GROUND_CENTER_X_ON_IMAGE`：以接触点所在图像行的原始 `u=160` 投影作为 X 偏置，使图像中心线在所有距离都输出 `X=0`，不修改 Y 和原有横向尺度。设为 `False` 可恢复原始坐标作 A/B 对比；测试脚本会额外画出蓝色中心线。

默认每 `200 ms` 打印一次；把脚本顶部的 `WORLD_COORD_PRINT_INTERVAL_MS` 设为 `0` 可逐帧打印。示例：

```text
[WORLD] frame=86 id=2 source=TRACK raw_pixel=(175.5,128.5) stable_pixel=(174.5,126.5) delta_px=(-1.0,-2.0) world_cm=(3.026,29.452) world_mm=(30,295) uncentered_x=1.704 x_bias=-1.322
```

`raw_pixel` 是原始色块（无色块时为显示框）的底边中点，画面用小红十字标记；`stable_pixel` 是实际送入逆透视的稳定接触点，画面用大黄十字标记。`delta_px` 是稳定点相对原始点的位移；`world_cm` 是居中修正后的逆投影结果；`world_mm` 是测试脚本 UART 包采用的四舍五入毫米值；`uncentered_x` 是旧标定 X，`x_bias` 是本行被扣除的旧中心偏置。`source=HELD/TRACK/MODEL_FRAME` 分别表示保持帧、普通跟踪帧和模型刷新帧。

该稳定层只处理接触像素，不修改 28 个标定点、36 个三角形、单应回退、Y 映射或 UART 单位；完整色块仍决定屏幕显示框。

启动时应看到：

```text
[GROUND] loaded 36 triangles from /sd/camera_ground_mesh.txt
```

缺少或拒绝网格时只会使用精度较低的全局单应回退，不应作为正式部署状态。

## 生成与验证

```powershell
python calibrate_ground_camera.py --ground-csv ground_mesh_24_points_template.csv --role master --expected-points 28 --required-near-y-cm 6 --max-y-cm 164 --output camera_ground_mesh.txt --report camera_ground_mesh_report.json
python -m unittest -v test_model_blob_fusion.py test_ground_projection.py
python -m py_compile main.py minimain.py world_coordinate_test.py calibrate_ground_camera.py raw_ground_projection_test.py test_model_blob_fusion.py test_ground_projection.py
git diff --check
```

`raw_ground_projection_test.py` 继续用于红沙包采点和标定复核；`world_coordinate_test.py` 用于按正式全类别检测流程观察最终坐标。完整版本说明、误差数据和历史记录见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。
