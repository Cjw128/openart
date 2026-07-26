# OpenART 双车视觉系统

当前正式版本：**v1.1.0（2026-07-26）内存与热路径优化版**

v1.1.0 基于 v1.0.0（提交 `d83b6d6`，2026-07-25），是一轮内存与热路径优化：地面网格逆透视改为从上次命中三角形续扫、新增 `u=160` 中心线按行懒缓存、`model_track` 状态由 14 槽缩到 11 槽、LAB 采样与三通道阈值计算消除逐次堆分配、删除一批从未被读取的死代码，并削减导入期峰值内存。除一条行为变更外，全部改动与 v1.0.0 行为逐位等价，由门禁 `test_model_blob_fusion.py` 的 13 项测试（含 `main.py` / `minimain.py` / `world_coordinate_test.py` 三方 AST 同步）保护。**唯一行为变更**：`0x06` 全色前扫命令处理现在额外调用 `reset_yellow_state()`——搬运中收到前扫请求会清空黄线拟合与进度标志，扫描后需重新捕获黄线（v1.0.0 会把黄线状态带进 `MODE_SEARCH`）。检测参数、状态机其余部分、坐标结果与 UART 协议均未改变；模型仍为 `/sd/80lite0.5SS.tflite`，默认曝光仍为 `880 us`。

## 工程概览

这是全国大学生智能汽车竞赛双车接力方案的**视觉侧**仓库。每辆车由两块板组成：NXP RT1021 底盘主控负责运动控制与任务调度（代码在独立工程维护，不在本仓库），OpenART Plus 视觉板运行本仓库脚本，负责物块识别与世界坐标解算。视觉板作为主控的 UART 从设备（两车均为 `UART12`、`115200 bps`）：接收主控的命令帧（`0xAA 0x55` 帧头 + 单字节命令码 + 可选参数字节 + 校验和，如 `0x03` 指定颜色、`0x06` 全色前扫、`0x08` 清零完成记录），持续回发 16 字节坐标包，其中目标世界 X/Y 以**毫米**发送。

双车角色：主车运行 `main.py`，负责搬运全流程，含越过黄线记完成、回库黄线引导；从车运行 `minimain.py`，为搬运从机，搬运结束后提交待完成 ID。两份入口为各自车辆的固定入口（不再保留 `IS_SLAVE_CAR` / `SLAVE_MODE` 开关），检测管线一致：TFLite 模型负责发现、类别确认和重捕，动态 LAB 色块确认后负责完整显示框，其底边接触点经独立稳定后送入 28 点地面网格逆透视得到世界坐标。

省赛基线 v0.11.0 保存在提交 `41260c0` 以及分支 `dedicated-model`、`archive/v0.11.0-ground-mesh`；完整迭代历史见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。

## 仓库文件清单

| 文件 | 运行位置 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus / 主车 | v1.1.0 主车正式入口（2650 行） |
| `minimain.py` | OpenART Plus / 从车 | v1.1.0 从车正式入口（2376 行） |
| `world_coordinate_test.py` | OpenART Plus / IDE | 与主车完整检测对齐的全类别世界坐标观察脚本 |
| `camera_ground_mesh.txt` | OpenART Plus / 主从 | 板端加载的 28 点、36 三角形地面网格 |
| `ground_mesh_24_points_template.csv` | PC | 当前 28 个像素/世界坐标标定点（文件名沿用旧名） |
| `calibrate_ground_camera.py` | PC | 网格生成、校验和报告工具 |
| `camera_ground_mesh_report.json` | PC | 当前网格质量报告 |
| `test_model_blob_fusion.py` | PC | **门禁测试**：主从/观察脚本模型-色块融合回归与三方 AST 同步 |
| `test_ground_projection.py` | PC | **门禁测试**：三方投影块逐字节一致与 28 点回代精度回归 |
| `raw_ground_projection_test.py` | OpenART Plus / IDE | 地面坐标采点和实地复核脚本 |
| `calib_ide_autocalib_competition.py` | OpenART Plus / IDE | 比赛现场自动标定与预览脚本 |
| `front_obstacle_scan_test.py` | OpenART Plus / IDE | 搬运前前方色块扫描预览脚本 |
| `openart_test_3class.py` | OpenART Plus / IDE | 三类模型测试脚本 |
| `fast_blob_backup/`、`stable_confirm/`、`stable_no_priority/`、`mainbak` | — | 历史对照存档，不是当前入口 |

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
- 显示框继续使用当前帧 `35%` 的平滑；世界坐标原始位置改用独立的当前帧 `50%` 权重，以更快跟随正常运动。参数为 `OUTPUT_SMOOTH_ALPHA_X100` 和 `COORDINATE_SMOOTH_ALPHA_X100`。
- 接触点保留 `2 px` 空间死区抑制静止抖动；单次偏移达到 `18 px` 时当帧直接跳到新位置，避免快速接近时坐标落后。参数为 `COORDINATE_CONTACT_DEADBAND_PX` 和 `COORDINATE_CONTACT_RESET_PX`。
- 网格内使用 36 个三角形插值，网格外使用全局单应矩阵回退。v1.1.0 起三角形扫描从上次命中的网格单元续扫（连续接触点几乎总在同一单元），插值结果与逐一扫描逐位一致。
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

网格生成（标定点变化时才需要重跑）：

```powershell
python calibrate_ground_camera.py --ground-csv ground_mesh_24_points_template.csv --role master --expected-points 28 --required-near-y-cm 6 --max-y-cm 164 --output camera_ground_mesh.txt --report camera_ground_mesh_report.json
```

v1.1.0 门禁测试——改动 `main.py`、`minimain.py` 或 `world_coordinate_test.py` 后必须全绿再提交：

```powershell
python -m unittest -v test_model_blob_fusion.py
python -m unittest -v test_ground_projection.py
python -m py_compile main.py minimain.py world_coordinate_test.py calibrate_ground_camera.py raw_ground_projection_test.py test_model_blob_fusion.py test_ground_projection.py
git diff --check
```

`test_model_blob_fusion.py` 共 13 项，覆盖模型/色块融合回归以及 `main.py` / `minimain.py` / `world_coordinate_test.py` 的三方 AST 同步（观察脚本剔除 `_world_coord_*` 插桩后必须与 `main.py` 一致）。`test_ground_projection.py` 共 6 项，覆盖三方地面投影代码块逐字节一致、28 个标定点回代精度、全幅 QVGA 坐标有界与毫米单位换算；其提取 harness 已在 v1.1.0 适配 `_mesh_last_triangle` 续扫全局与 `center_line_world_x_for_row` 行缓存，两套测试均为 v1.1.0 门禁，全绿方可提交。

`raw_ground_projection_test.py` 继续用于红沙包采点和标定复核；`world_coordinate_test.py` 用于按正式全类别检测流程观察最终坐标。完整版本说明、误差数据和历史记录见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。
