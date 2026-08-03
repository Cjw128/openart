# OpenART 双车视觉系统

当前正式版本：**v1.1.0（2026-07-26）内存与热路径优化版**

v1.1.0 基于 v1.0.0（提交 `d83b6d6`，2026-07-25），是一轮内存与热路径优化：地面网格逆透视改为从上次命中三角形续扫、新增 `u=160` 中心线按行懒缓存、`model_track` 状态由 14 槽缩到 11 槽、LAB 采样与三通道阈值计算消除逐次堆分配、删除一批从未被读取的死代码，并削减导入期峰值内存。除一条行为变更外，全部改动与 v1.0.0 行为逐位等价，由门禁 `test_model_blob_fusion.py` 的 13 项测试（含 `main.py` / `minimain.py` / `world_coordinate_test.py` 三方 AST 同步）保护。**唯一行为变更**：`0x06` 全色前扫命令处理现在额外调用 `reset_yellow_state()`——搬运中收到前扫请求会清空黄线拟合与进度标志，扫描后需重新捕获黄线（v1.0.0 会把黄线状态带进 `MODE_SEARCH`）。检测参数、状态机其余部分、坐标结果与 UART 协议均未改变；模型仍为 `/sd/80lite0.5SS.tflite`，默认曝光仍为 `880 us`。

当前开发版将普通自由搜索的首次锁定由 `5/7` 调整为运动容错的 `3/5`，中心连续性容差由 `24 px` 放宽至 `36 px`，尺寸变化容差由 `35%` 放宽至 `50%`；首锁置信度仍为 `0.30`，曝光和颜色 IQR 不变。模型 / 色块融合恢复省赛冻结版的响应方式：模型几何作为锚点，颜色框只用中心位移带动显示框和坐标框，两者统一采用当前帧 `70%` 的单级平滑，中心移动超过 `36 px` 时当帧直接接管。锁定后的兼容模型结果首次出现即更新，近距离低分模型帧也不再冻结旧坐标。新增 `ENABLE_COMPLETED_COLOR_EXCLUSION` 独立控制已完成颜色是否排除，默认关闭，需要“第一个 ID2、之后只从未完成颜色中选最近目标”时再开启。28 点 mesh、36 个三角形、中心 X 修正和 UART 毫米协议保持不变；主车临时 ID2 坐标 watchdog 日志已完整删除，正式循环不做 SD 日志 I/O。

2026-08-01 开发版已删除试验性的坐标 / 半径锚点协议，改为两阶段同色目标选择：主控用 `0x09` 请求 OpenART 枚举指定颜色的全部可用候选，OpenART 以 `0xC9` 包逐个回传候选索引与世界坐标，主控再用 `0x0A` 确认精确候选。该协议已在 `main.py` 与 `minimain.py` 同步，且与旧 11 字节 `0x09` 帧不兼容。

2026-08-02 开发版新增 orbit 近场裁切与前方障碍补检：主控以短帧 `AA 55 0B 0B` 通知 OpenART 进入 orbit 后，普通目标搜索限制在全宽 `y >= 140` 的近场区域，进入搬运、回库、重置或切换目标时解除；该状态不累积帧，不增加坐标输出延迟。`0x06` 前扫仍独立扫描 `y < 150`，新增不依赖模型框的 ID2 横向红砖色块补检，并将模型熊框内的 ID4/ID5 判色改为两套固定 LAB 阈值的像素竞争，判据接近时返回未知而不强猜颜色。

2026-08-03 开发版将同一套 ID4/ID5 固定阈值像素竞争同步到普通自动锁色、锁定后重判、主控强制目标和 `0x09` 候选枚举。熊类身份不再由整框 LAB 中位数强制二选一；整框统计仅在与像素赢家一致时生成动态跟踪阈值，否则回退对应固定阈值。现场自动标定的固定白平衡同步为正式运行值 `(92,64,101)`；已有 `/sd/color_thr.txt` 不会自动转换，必须在实际首次识别距离重新标定并复核远距离 ID4/ID5 像素数。

## 工程概览

这是全国大学生智能汽车竞赛双车接力方案的**视觉侧**仓库。每辆车由两块板组成：NXP RT1021 底盘主控负责运动控制与任务调度（代码在独立工程维护，不在本仓库），OpenART Plus 视觉板运行本仓库脚本，负责物块识别与世界坐标解算。视觉板作为主控的 UART 从设备（两车均为 `UART12`、`115200 bps`）：接收主控的命令帧（`0xAA 0x55` 帧头 + 单字节命令码 + 参数 + 校验和，如 `0x03` 指定颜色、`0x06` 全色前扫、`0x08` 清零完成记录、`0x09` 枚举同色候选、`0x0A` 确认候选、`0x0B` 启用 orbit 近场裁切），持续回发 16 字节坐标包；处理 `0x09` 时另以 12 字节 `0xC9` 包回传候选，所有世界 X/Y 均以**毫米**发送。

双车角色：主车运行 `main.py`，负责搬运全流程，含越过黄线记完成、回库黄线引导；从车运行 `minimain.py`，为搬运从机，搬运结束后提交待完成 ID。两份入口为各自车辆的固定入口（不再保留 `IS_SLAVE_CAR` / `SLAVE_MODE` 开关），检测管线一致：TFLite 模型负责发现、类别确认和重捕，动态 LAB 色块负责确认目标并提供中心位移，模型拥有的显示 / 坐标几何随该位移移动，平滑后的底边接触点再送入 28 点地面网格解算世界坐标。

省赛基线 v0.11.0 保存在提交 `41260c0` 以及分支 `dedicated-model`、`archive/v0.11.0-ground-mesh`；完整迭代历史见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。

## 仓库文件清单

| 文件 | 运行位置 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus / 主车 | v1.1.0 主车正式入口（2986 行） |
| `minimain.py` | OpenART Plus / 从车 | v1.1.0 从车正式入口（2732 行） |
| `world_coordinate_test.py` | OpenART Plus / IDE | 与主车完整检测对齐的全类别世界坐标观察脚本 |
| `camera_ground_mesh.txt` | OpenART Plus / 主从 | 板端加载的 28 点、36 三角形地面网格 |
| `ground_mesh_24_points_template.csv` | PC | 当前 28 个像素/世界坐标标定点（文件名沿用旧名） |
| `calibrate_ground_camera.py` | PC | 网格生成、校验和报告工具 |
| `camera_ground_mesh_report.json` | PC | 当前网格质量报告 |
| `test_model_blob_fusion.py` | PC | **门禁测试**：主从/观察脚本模型-色块融合回归与三方 AST 同步 |
| `test_ground_projection.py` | PC | **门禁测试**：三方投影块逐字节一致与 28 点回代精度回归 |
| `test_orbit_y_cut.py` | PC | orbit 固定 Y 裁切、状态清理与 `0x0B` UART 回归 |
| `test_front_scan_id2_blob.py` | PC | `0x06` ID2 动态阈值优先、基础阈值回退与横砖补检回归 |
| `test_front_scan_bear_color.py` | PC | `0x06` 前扫与普通锁色 ID4/ID5 像素竞争回归 |
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

## 同 ID 目标候选枚举与确认

当前 `main.py` 与 `minimain.py` 使用两阶段事务选择同色物体。旧的 `ENABLE_TARGET_ANCHOR_LOCK`、坐标 / 半径锚点状态及 11 字节 `0x09` 帧均已删除。`0x09` 只触发一次专用模型推理并保存候选快照，不会直接确认其中某个候选；主控收到完整候选列表后，再通过 `0x0A` 提交精确索引。

两种下行命令均为 6 字节：

```text
AA 55 09 CID SEQ CHECKSUM
AA 55 0A SEQ INDEX CHECKSUM
```

- `CID` 为要枚举的颜色 ID；它仍受 ID 范围、`ID2_ABSOLUTE_PRIORITY` 和已完成颜色排除开关约束。无效或当前不可搜索的 `CID` 不会启动枚举，也不会产生 `0xC9` 结果包。
- `SEQ` 为主控分配的单字节事务号。新的 `0x09` 会覆盖尚未确认的旧候选列表；`0x0A` 只有在 `SEQ` 与最近一次已完成枚举一致且 `INDEX` 有效时才生效。
- `INDEX` 从 `0` 开始。确认成功后，OpenART 以该候选的模型框和当次 LAB 颜色样本建立跟踪锁，并清空待确认列表；无效确认被静默忽略且没有专用确认回包，主控从后续常规 16 字节坐标包观察锁定结果。
- `CHECKSUM = (CMD + PARAM + VALUE) & 0xFF`，不包含 `AA 55` 和校验字节自身。

每个有效候选使用一个 12 字节上行包：

```text
AA 55 C9 SEQ INDEX TOTAL CID X_LO X_HI Y_LO Y_HI CHECKSUM
```

- `TOTAL` 是本次事务的候选总数，同一事务的所有包携带相同 `SEQ`、`TOTAL` 和 `CID`。候选按图像中的水平中心从左到右排序，因此索引在单次快照内稳定。
- X/Y 为 OpenART 当前 `box_to_world()` 坐标系下的 little-endian 有符号 `int16`，单位毫米。
- 没有候选时仍回发一个包：`INDEX=0`、`TOTAL=0`、X/Y 均为 0；主控不得对此发送 `0x0A`。
- `CHECKSUM = sum(C9..Y_HI) & 0xFF`。全部 `0xC9` 包发送完毕后，OpenART 还会按现有协议发送一帧 16 字节目标保持包或无目标包。

候选必须同时通过指定模型类别、主控指定搜索最低分数、LAB 颜色匹配、网球黄线过滤和有效世界坐标检查。普通 `0x03` 指定颜色、目标状态重置、回库或新的 `0x09` 都会使旧事务失效。RT1021 端必须按上述新长度升级；旧的 11 字节 `0x09` 锚点帧与当前解析器不兼容。

## Orbit 近场裁切与前方障碍扫描

主控在进入 orbit 时发送短帧 `AA 55 0B 0B`。OpenART 仅在已有有效选中目标时启用 `ORBIT_Y_CUT=140`：模型候选的底边必须到达 `y=140`，色块搜索 ROI 则与 `y=140..229` 的全宽区域取交集。重复 `0x0B` 幂等，不采集前五帧 ROI，也不改变坐标包时序。`0x01` 进入搬运、`0x02` 结束搬运、`0x07` 回库、目标重置、新的目标枚举或切换到其它目标都会解除裁切。

`0x06` 前方扫描不使用上述 orbit 裁切，仍扫描动态有效区域中 `y < 150` 的部分；扫描结束后，若尚未收到解除命令，普通目标跟踪继续使用 orbit 裁切。当前目标按 `IoU >= 0.20` 或中心距离 `<= 35 px` 排除，结果连续稳定 6 帧时发送，最多观察 12 帧，回包保持 `AA 55 C7 current_id mask count checksum`，其中 `count` 是不同颜色 ID 的数量。

前扫以模型结果为主，同时增加 ID2 色块补检。补检优先使用 `adaptive_color_thresholds[1]`，没有有效横砖时回退到基础 ID2 阈值；宽高比允许 `0.6..6.0`，并要求至少 70 个命中像素、`100 px²` 包围面积和 `0.40` 密度，因此即使模型不能识别横放红砖也能置位 ID2。棕熊与白熊共用模型标签，前扫和普通锁色都在模型内缩框中分别统计 ID4/ID5 固定阈值像素；获胜方至少 12 像素、领先至少 6 像素且达到另一方的 1.3 倍才确认，否则返回未知。动态熊阈值不参与身份竞争，只在整框统计与像素赢家一致时用于跟踪；不一致时使用固定阈值，避免背景样本强化错误身份。

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

检查命令（融合门禁中的旧锚点用例仍待迁移）：

```powershell
python -m unittest -v test_model_blob_fusion.py
python -m unittest -v test_ground_projection.py
python -m unittest -v test_orbit_y_cut.py test_front_scan_id2_blob.py test_front_scan_bear_color.py
python -m py_compile main.py minimain.py world_coordinate_test.py calibrate_ground_camera.py raw_ground_projection_test.py test_model_blob_fusion.py test_ground_projection.py
git diff --check
```

orbit、前扫与地面投影的 24 项定向测试当前全绿。完整 `unittest discover` 共 56 项，其中 50 项通过；剩余 4 项旧锚点协议测试和 2 项依赖尚未同步 `world_coordinate_test.py` 的三方 AST 门禁需要迁移到上述 `0x09` / `0x0A` 协议后才能重新作为全绿门禁。当前主从运行时已通过 Python 语法检查、主从新增函数 AST 一致性检查和 `git diff --check`。

`raw_ground_projection_test.py` 继续用于红沙包采点和标定复核；`world_coordinate_test.py` 用于按正式全类别检测流程观察最终坐标。完整版本说明、误差数据和历史记录见 [中文更新日志](README_ch.md) 与 [English changelog](README_en.md)。
