# OpenART 双车视觉系统

当前正式版本：**v1.1.0（2026-07-26）内存与热路径优化版**

v1.1.0 基于 v1.0.0（提交 `d83b6d6`，2026-07-25），是一轮内存与热路径优化：地面网格逆透视改为从上次命中三角形续扫、新增 `u=160` 中心线按行懒缓存、`model_track` 状态由 14 槽缩到 11 槽、LAB 采样与三通道阈值计算消除逐次堆分配、删除一批从未被读取的死代码，并削减导入期峰值内存。除一条行为变更外，全部改动与 v1.0.0 行为逐位等价，由门禁 `test_model_blob_fusion.py` 的 13 项测试（含 `main.py` / `minimain.py` / `world_coordinate_test.py` 三方 AST 同步）保护。**唯一行为变更**：`0x06` 全色前扫命令处理现在额外调用 `reset_yellow_state()`——搬运中收到前扫请求会清空黄线拟合与进度标志，扫描后需重新捕获黄线（v1.0.0 会把黄线状态带进 `MODE_SEARCH`）。检测参数、状态机其余部分、坐标结果与 UART 协议均未改变；模型仍为 `/sd/80lite0.5SS.tflite`，默认曝光仍为 `880 us`。

当前开发版将普通自由搜索的首次锁定由 `5/7` 调整为运动容错的 `3/5`，中心连续性容差由 `24 px` 放宽至 `36 px`，尺寸变化容差由 `35%` 放宽至 `50%`；首锁置信度仍为 `0.30`，曝光和颜色 IQR 不变。模型 / 色块融合恢复省赛冻结版的响应方式：模型几何作为锚点，颜色框只用中心位移带动显示框和坐标框，两者统一采用当前帧 `70%` 的单级平滑，中心移动超过 `36 px` 时当帧直接接管。锁定后的兼容模型结果首次出现即更新，近距离低分模型帧也不再冻结旧坐标。新增 `ENABLE_COMPLETED_COLOR_EXCLUSION` 独立控制已完成颜色是否排除，默认关闭，需要“第一个 ID2、之后只从未完成颜色中选最近目标”时再开启。28 点 mesh、36 个三角形、中心 X 修正和 UART 毫米协议保持不变；主车临时 ID2 坐标 watchdog 日志已完整删除，正式循环不做 SD 日志 I/O。

## 工程概览

这是全国大学生智能汽车竞赛双车接力方案的**视觉侧**仓库。每辆车由两块板组成：NXP RT1021 底盘主控负责运动控制与任务调度（代码在独立工程维护，不在本仓库），OpenART Plus 视觉板运行本仓库脚本，负责物块识别与世界坐标解算。视觉板作为主控的 UART 从设备（两车均为 `UART12`、`115200 bps`）：接收主控的命令帧（`0xAA 0x55` 帧头 + 单字节命令码 + 参数 + 校验和，如 `0x03` 指定颜色、`0x06` 全色前扫、`0x08` 清零完成记录、`0x09` 指定同 ID 目标锚点），持续回发 16 字节坐标包，其中目标世界 X/Y 以**毫米**发送。

双车角色：主车运行 `main.py`，负责搬运全流程，含越过黄线记完成、回库黄线引导；从车运行 `minimain.py`，为搬运从机，搬运结束后提交待完成 ID。两份入口为各自车辆的固定入口（不再保留 `IS_SLAVE_CAR` / `SLAVE_MODE` 开关），检测管线一致：TFLite 模型负责发现、类别确认和重捕，动态 LAB 色块负责确认目标并提供中心位移，模型拥有的显示 / 坐标几何随该位移移动，平滑后的底边接触点再送入 28 点地面网格解算世界坐标。

省赛基线 v0.11.0 保存在提交 `41260c0` 以及分支 `dedicated-model`、`archive/v0.11.0-ground-mesh`；完整迭代历史见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。

## 仓库文件清单

| 文件 | 运行位置 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus / 主车 | v1.1.0 主车正式入口（2785 行） |
| `minimain.py` | OpenART Plus / 从车 | v1.1.0 从车正式入口（2535 行） |
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

## 已完成颜色排除开关

三份入口顶部的 `ENABLE_COMPLETED_COLOR_EXCLUSION` 默认为 `False`：程序仍记录 `completed_color_mask`，但完成颜色不会因此退出候选。配合 `ID2_ABSOLUTE_PRIORITY=True`，第一个目标依旧固定为 ID2；ID2 完成后绝对优先解除，ID1~ID5（包含 ID2）全部按世界 Y 竞争，因此第二个可能再次是 ID2。

设为 `True` 后启用候选排除，已完成颜色立即按现有记录退出普通搜索和主控指定；此时第一个仍为 ID2，ID2 完成后第二个由 ID1/ID3/ID4/ID5 中世界 Y 最近者产生，后续继续从未完成颜色中选择。再次关闭不会清空完成位。若希望从第一轮起所有颜色都参与竞争，需要同时将 `ID2_ABSOLUTE_PRIORITY=False`。

## 同 ID 目标锚点开关

三份当前入口顶部均有 `ENABLE_TARGET_ANCHOR_LOCK`，默认值为 `True`。该值只表示允许使用锚点；在没有收到有效 `0x09` 帧时，候选筛选与原 `0x03` 最近目标模式完全一致。设为 `False` 后，程序仍会完整接收 `0x09` 并按其中的 `CID` 搜索，但忽略 X/Y、半径和序号，等价退回旧模式，不会破坏串口帧同步。`RADIUS_CM=0` 也可让单条 `0x09` 按旧模式处理。

`0x09` 命令共 11 字节：

```text
AA 55 09 CID X_LO X_HI Y_LO Y_HI RADIUS_CM SEQ CHECKSUM
```

- X/Y 为 little-endian 有符号 `int16`，单位毫米，坐标系必须与接收该帧的 OpenART `box_to_world()` 相同，即该车相机当前局部坐标。
- `RADIUS_CM` 为无符号单字节厘米半径。首次锁定只接纳半径内的同 ID 候选，并以锚点误差最小者优先；锁定后继续使用现有图像连续跟踪，丢失后再受锚点约束重捕。
- `SEQ` 标识物理实例。同一 `CID+SEQ` 可随车辆平移持续更新 X/Y 和半径，不清空主控指定搜索的 `3/5` 首锁证据；新 `CID` 或新 `SEQ` 会释放旧视觉锁并重新获取。
- `CHECKSUM = sum(09..SEQ) & 0xFF`，不包含 `AA 55` 和校验字节自身。
- 重复发送同 CID 的旧 `0x03` 不会取消已生效锚点；切换到其他 CID、`0x00`、首次有效 `0x02`、`0x07` 或 `0x08` 会清除锚点。

当前 RT1021 主控代码尚未发送 `0x09`，因此只烧录本次 OpenART 程序不会改变现有整车行为，也还不能保证两车锁定同一实例。后续主控接入时，应由主控根据本轮左右槽位、两车固定间距和各自实时局部里程计，分别换算出同一物体在两块相机当前局部坐标中的预期 X/Y；未看到目标而横移的车辆继续使用同一 `SEQ` 更新坐标。

## 世界坐标

- 标定源为 `ground_mesh_24_points_template.csv`，文件名沿用旧名，实际包含 `7 x 4 = 28` 个点。
- 图像链路固定为 QVGA、`vflip=True`、软件水平镜像、不使用 `lens_corr()`。
- 目标底边中点 `x + w/2, y + h - 0.5` 作为地面接触点。
- 模型刷新帧只在模型框内缩区域搜索颜色：左右各缩 `5%`、顶部缩 `5%`、底部缩 `10%`，并忽略旧颜色框引用。模型几何作为锚点；后续颜色框的尺寸变化不会改变坐标几何，只有颜色框中心位移会同步平移显示框和坐标框。
- 显示框和坐标框统一使用省赛冻结版的单级平滑：当前值 `70%`、上一值 `30%`；框中心移动超过 `36 px` 时当帧直接采用新框。唯一参数为 `OUTPUT_SMOOTH_ALPHA_X100=70` 和 `OUTPUT_SMOOTH_RESET_CENTER2=36*36`。
- 已删除独立坐标 EMA、接触点死区、近距离降权和跳变二次确认。锁定后的兼容模型候选首次推理即更新；任何距离下的新模型几何均参与同一 `70%` 跟随，不再因低分保持旧坐标。只有真实检测丢失时仍沿用既有的有限帧保持 / 重捕状态机。
- 网格内使用 36 个三角形插值，网格外使用全局单应矩阵回退。v1.1.0 起三角形扫描从上次命中的网格单元续扫（连续接触点几乎总在同一单元），插值结果与逐一扫描逐位一致。
- 板内坐标单位为厘米，16 字节 UART 包中的 X/Y 仍发送**毫米**。
- 主车继续保留 `8 s` 硬件 WDT 防死锁；它与已删除的 ID2 坐标 watchdog 日志无关。主从正式入口都不创建或写入 `id2_coordinate_watchdog.log`。

### 全类别坐标观察

在 OpenART IDE 中运行 `world_coordinate_test.py`。该脚本保留 `main.py` 的完整模型、五色 LAB、模型/色块融合、最近目标选择和世界坐标逻辑，只增加屏幕标记与串口终端打印。SD 卡仍需放置正式运行使用的模型、`camera_ground_mesh.txt` 和可选的 `color_thr.txt`。

主车、从车和测试脚本均默认启用 `GROUND_CENTER_X_ON_IMAGE`：以接触点所在图像行的原始 `u=160` 投影作为 X 偏置，使图像中心线在所有距离都输出 `X=0`，不修改 Y 和原有横向尺度。设为 `False` 可恢复原始坐标作 A/B 对比；测试脚本会额外画出蓝色中心线。

默认每 `200 ms` 打印一次；把脚本顶部的 `WORLD_COORD_PRINT_INTERVAL_MS` 设为 `0` 可逐帧打印。示例：

```text
[WORLD] frame=86 id=2 source=TRACK raw_pixel=(175.5,128.5) stable_pixel=(174.5,126.5) delta_px=(-1.0,-2.0) world_cm=(3.026,29.452) world_mm=(30,295) uncentered_x=1.704 x_bias=-1.322
```

`raw_pixel` 是原始色块（无色块时为显示框）的底边中点，画面用小红十字标记；`stable_pixel` 是实际送入逆透视的稳定接触点，画面用大黄十字标记。`delta_px` 是稳定点相对原始点的位移；`world_cm` 是居中修正后的逆投影结果；`world_mm` 是测试脚本 UART 包采用的四舍五入毫米值；`uncentered_x` 是旧标定 X，`x_bias` 是本行被扣除的旧中心偏置。`source=HELD/TRACK/MODEL_FRAME` 分别表示保持帧、普通跟踪帧和模型刷新帧。

`stable_pixel` 来自显示 / 坐标框共用的省赛式 `70%` 单级跟随，不再经过独立接触点滤波。该跟随层不修改 28 个标定点、36 个三角形、单应回退、Y 映射或 UART 单位；目标重新锁定或 ID 切换时会重新建立模型 / 色块中心锚点，不沿用上一个物体的位置。

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

`test_model_blob_fusion.py` 当前共 32 项，覆盖模型 / 色块中心锚定、省赛式单级跟随、模型首帧即时更新、近距离低分帧即时跟随、旧坐标滤波路径删除、运动首锁、已完成颜色排除开关、锚点门控、UART 边界，以及 `main.py` / `minimain.py` / `world_coordinate_test.py` 的三方 AST 同步（观察脚本剔除 `_world_coord_*` 插桩后必须与 `main.py` 一致）。`test_ground_projection.py` 共 6 项，覆盖三方地面投影代码块逐字节一致、28 个标定点回代精度、全幅 QVGA 坐标有界与毫米单位换算；两套测试合计 38 项，全绿方可提交。

`raw_ground_projection_test.py` 继续用于红沙包采点和标定复核；`world_coordinate_test.py` 用于按正式全类别检测流程观察最终坐标。完整版本说明、误差数据和历史记录见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。
