# OpenART 双车视觉系统

全国大学生智能汽车竞赛双车接力方案的 OpenART Plus 视觉端代码。

> 最终归档版本：`v1.2.0`，2026-09-04。该版本保留比赛结束时的主车、从车运行逻辑和现场参数，后续不再以比赛现场迭代为目标。

[中文更新日志](README_ch.md) · [English changelog](README_en.md) · [MIT License](LICENSE)

## 项目简介

系统由两辆车组成。每辆车使用一块 NXP RT1021 负责运动控制和任务调度，并使用一块 OpenART Plus 负责视觉识别。RT1021 工程不在本仓库中；本仓库只包含 OpenART 端代码、标定工具和历史版本。

两块视觉板都通过 `UART12`、`115200 bps` 与各自的 RT1021 通信。视觉端完成以下工作：

- 使用 TFLite 模型发现沙包、网球和熊类目标；
- 使用 LAB 色块确认具体颜色 ID，并在模型推理间隔内跟踪目标；
- 将目标底边接触点通过 28 点、36 三角形地面网格换算为世界坐标；
- 按主控策略选择目标、枚举同色候选并确认指定候选；
- 扫描当前目标前方的其他颜色 ID；
- 主车判断搬运黄线和过线状态，主从车均支持回库黄线检测；
- 记录每轮已经搬运的颜色，避免重复搜索。

## 最终版本配置

主车和从车的相机参数来自各自比赛现场标定，不能视为通用值。

| 配置 | 主车 `main.py` | 从车 `minimain.py` |
| --- | --- | --- |
| 板卡 | OpenART Plus | OpenART Plus |
| 图像 | QVGA RGB565、50 fps | QVGA RGB565、50 fps |
| 模型路径 | `/sd/80lite0.5SS.tflite` | `/sd/80lite0.5SS.tflite` |
| 默认白平衡 R/G/B | `(92, 64, 91)` | `(101, 64, 97)` |
| 默认曝光 | `700 us` | `880 us` |
| 串口 | `UART12`、`115200 bps` | `UART12`、`115200 bps` |
| 完成颜色排除 | 开启 | 开启 |
| 硬件 WDT | `8 s` | 无 |
| SD 调试日志 | 默认关闭 | 默认关闭 |

`/sd/color_thr.txt` 中的 `wb_gains` 和 `exposure_us` 会覆盖上述默认值。两台相机的安装位置和成像差异较大时，应分别标定，不要直接共用参数文件。

## 目标定义

| 颜色 ID | 目标 | TFLite label |
| --- | --- | --- |
| `1` | 蓝色沙包 | `2`（bag） |
| `2` | 红色沙包 / 场地红砖 | `2`（bag） |
| `3` | 绿色网球 | `1`（ball） |
| `4` | 棕熊 | `0`（bear） |
| `5` | 白熊 | `0`（bear） |

模型只区分 `bear / ball / bag` 三个大类。ID1/ID2 和 ID4/ID5 由 LAB 颜色信息进一步区分，因此模型 label 顺序和颜色标定必须与上表一致。

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| `main.py` | 主车最终运行入口，包含搬运黄线状态机和 `8 s` WDT |
| `minimain.py` | 从车最终运行入口 |
| `calib_ide_autocalib_competition.py` | 在 OpenART IDE 中运行的五目标颜色自动标定与预览脚本 |
| `calibrate_ground_camera.py` | PC 端地面网格生成和校验工具 |
| `ground_mesh_24_points_template.csv` | 当前 28 个像素 / 世界坐标标定点；文件名因历史原因仍含 `24` |
| `camera_ground_mesh.txt` | OpenART 端读取的 28 点、36 三角形地面网格 |
| `camera_ground_mesh_report.json` | 当前网格的生成与质量报告 |
| `fast_blob_backup/` | 纯 LAB 快速回退版本的历史存档 |
| `stable_confirm/` | 严格首次确认版本的历史存档 |
| `stable_no_priority/` | 无目标优先级版本的历史存档 |
| `mainbak` | 更早的主车单文件存档 |
| `README_ch.md` / `README_en.md` | 中英文完整开发日志 |

根目录的 `main.py` 和 `minimain.py` 才是最终入口。历史目录用于对照，不应与根目录文件混合部署。

## 使用前准备

### 硬件和固件

- 两块 OpenART Plus 及可用的 SD 卡；
- 两个已固定安装的 QVGA 相机；
- 分别连接两辆车 RT1021 的 UART；
- 支持 `sensor`、`tf`、`machine.UART` 的 OpenART MicroPython 固件；
- 主车固件还应支持 `machine.WDT`，不可用时程序会继续运行但没有硬件看门狗。

本仓库不包含 RT1021 主控工程。主控必须按下文协议完成命令发送、坐标解析和任务状态管理。

### 模型文件

运行时必须提供 `/sd/80lite0.5SS.tflite`。模型权重不公开，所有公开分支和标签都通过 `.gitignore` 排除 `*.tflite`；请自行训练或取得有权使用的兼容模型，其 label 顺序必须符合“目标定义”一节。

历史分支 `archive/provincial-final-2026-07-21` 只保留比赛脚本和配置，模型文件及其校验值已从可达 Git 历史中移除。MIT License 只覆盖本仓库源码和文档，不授予任何模型权重许可。

模型加载成功时串口控制台会显示：

```text
[MODEL] loaded /sd/80lite0.5SS.tflite
```

连续加载或推理失败时，程序会输出 `[MODEL ALARM]` 并停止目标坐标输出；它不会静默退化为纯色块全场搜索。

### PC 标定依赖

只有重新生成地面网格时才需要 PC 端依赖：

```powershell
python -m pip install numpy opencv-python
```

`mpy-cross` 只用于发布前编译检查，不是运行标定脚本的必需项。

## 快速部署

1. 格式化并确认两块 SD 卡可由 OpenART Plus 正常读写。
2. 将主车的 `main.py` 复制为主车 SD 卡的 `/sd/main.py`。
3. 将从车的 `minimain.py` 重命名并复制为从车 SD 卡的 `/sd/main.py`。
4. 将 `camera_ground_mesh.txt` 复制到两块卡的 `/sd/camera_ground_mesh.txt`。
5. 将兼容模型复制到两块卡的 `/sd/80lite0.5SS.tflite`。
6. 如已完成现场颜色标定，将两台相机各自的 `color_thr.txt` 放入对应 SD 卡根目录。
7. 连接 `UART12`，确认双方均使用 `115200 bps`，再上电运行。

最终 SD 卡至少应包含：

```text
/sd/
├── main.py
├── 80lite0.5SS.tflite
├── camera_ground_mesh.txt
└── color_thr.txt              # 可选，但正式现场建议提供
```

地面网格加载成功时应看到：

```text
[GROUND] loaded 36 triangles from /sd/camera_ground_mesh.txt
```

若出现 `mesh unavailable`，程序会使用内置全局单应矩阵回退。该回退精度较低，只适合诊断，不应作为重新安装相机后的正式部署状态。

首次带轮调试前应架空驱动轮，先验证急停、串口命令、坐标方向和任务状态切换，再让车辆落地运动。

## 颜色标定

在 OpenART IDE 中运行 `calib_ide_autocalib_competition.py`。保持地面采样区域为空，按脚本提示依次放置目标：

```text
两个沙包（顺序不限） -> 网球 -> 棕熊 -> 白熊
```

脚本只有在五个槽位全部采集和复检通过后才覆盖 `/sd/color_thr.txt`；不完整结果写入 `/sd/color_thr_partial.txt`。生成文件的结构为：

```text
exposure_us=<100..4500>
wb_gains=<R>,<G>,<B>
ground=<L_MIN>,<L_MAX>,<A_MIN>,<A_MAX>,<B_MIN>,<B_MAX>
ground2=<L_MIN>,<L_MAX>,<A_MIN>,<A_MAX>,<B_MIN>,<B_MAX>
1,<ID1 的六项 LAB 边界>
2,<ID2 的六项 LAB 边界>
3,<ID3 的六项 LAB 边界>
4,<ID4 的六项 LAB 边界>
5,<ID5 的六项 LAB 边界>
```

若文件缺失、行数不足或数值无效，运行时会保留内置阈值；白平衡和曝光分别回退到各车入口顶部的值。重新安装相机、改变照明或更换模型后，应重新检查五类目标和蓝色地面分割。

## 地面坐标标定

世界坐标采用右手方向约定：X 向右、Y 向前，板内计算单位为厘米，UART 输出单位为毫米。目标框底边中点作为地面接触点。

当前 CSV 实际包含 `7 x 4 = 28` 个点。修改 `ground_mesh_24_points_template.csv` 后，在 PC 上运行：

```powershell
python calibrate_ground_camera.py `
  --ground-csv ground_mesh_24_points_template.csv `
  --role master `
  --expected-points 28 `
  --required-near-y-cm 6 `
  --max-y-cm 164 `
  --output camera_ground_mesh.txt `
  --report camera_ground_mesh_report.json
```

当前两份运行入口都读取 `role=master` 的网格。若两台相机的安装几何不同，应分别采点、生成不同网格，并同步修改对应入口的 `CAMERA_GROUND_MESH_ROLE`；否则 role 校验会拒绝加载。

图像链路必须与网格元数据一致：QVGA、`sensor_vflip=1`、软件水平镜像、无 `lens_corr()`。改变这些条件后旧网格失效。

## UART 协议

所有多字节整数均为 little-endian。除特别说明外，校验和为从命令码或包类型开始，到校验字节前所有字段之和的低 8 位；帧头 `AA 55` 不参与校验。

### 主控下行命令

普通无参数命令使用 4 字节帧：

```text
AA 55 CMD CHECKSUM
```

| 命令 | 作用 |
| --- | --- |
| `0x00` | 回到搜索模式并清除当前跟踪 |
| `0x01` | 进入搬运；主车启动搬运黄线状态机，从车锁存本轮 ID |
| `0x02` | 结束搬运并回到搜索；提交待完成 ID |
| `0x05` | 历史预留，当前只消费帧不改变状态 |
| `0x06` | 请求一次前方其他颜色扫描 |
| `0x07` | 进入回库黄线模式 |
| `0x08` | 清空全部已完成颜色，完整帧为 `AA 55 08 08` |
| `0x0B` | 已有有效目标时启用 orbit 近场裁切，完整帧为 `AA 55 0B 0B` |

带单字节参数的 `0x03`、`0x04` 和 `0x0C` 使用 5 字节帧：

```text
AA 55 CMD PARAM CHECKSUM
```

| 命令 | 参数 | 作用 |
| --- | --- | --- |
| `0x03` | `CID=1..5` | 指定要搜索的颜色 ID |
| `0x04` | 任意 | 预留命令，当前忽略 |
| `0x0C` | `FLAGS` | 切换搜索优先策略，并重新开始搜索 |

上电默认策略为 `0x00`。`ENABLE_COMPLETED_COLOR_EXCLUSION=True` 时，已完成 ID 会从策略允许的集合中继续排除。

| `FLAGS` | 搜索顺序 |
| --- | --- |
| `0x00` | 所有未完成 ID 自由竞争，按世界 Y 选择最近目标 |
| `0x01` | ID2 完成前只搜索 ID2 |
| `0x02` | ID3 完成前只搜索 ID3 |
| `0x05` | ID2 -> ID3/ID4/ID5 -> ID1 |
| `0x06` | ID3 -> ID4/ID5 -> ID1/ID2 |
| `0x07` | ID3 -> ID4/ID5 -> ID1 -> ID2 |
| `0x08` | ID4/ID5 -> ID3 -> ID1 -> ID2 |

`FLAGS=0x08` 必须放在完整策略帧 `AA 55 0C 08 14` 中。它与 4 字节的清零命令 `AA 55 08 08` 含义不同。

同色多目标使用两阶段事务，两个命令都是 6 字节：

```text
AA 55 09 CID SEQ CHECKSUM
AA 55 0A SEQ INDEX CHECKSUM
```

`0x09` 对指定 CID 做一次候选枚举，`SEQ` 由主控分配。候选按画面中心从左到右排序；`0x0A` 只有在 SEQ 与最近一次枚举一致且 INDEX 有效时才确认目标。新的枚举、普通 `0x03`、目标复位或回库会使旧事务失效。

### 视觉端上行数据

常规目标帧固定为 16 字节：

```text
AA 55 CID X_LO X_HI Y_LO Y_HI WIDTH_LO WIDTH_HI YELLOW POS R0 R1 R2 R3 CHECKSUM
```

- `CID=0` 表示无目标；
- `X/Y` 为有符号 `int16` 毫米坐标；
- `WIDTH` 为目标在图像中的像素宽度；
- `YELLOW` 为搜索状态下的黄线标志；
- 主车 `POS`：`0x00` 无边界、`0x01` 位于黄线右侧、`0x02` 已过线；
- 从车不运行主车搬运黄线状态机，相关字段保持兼容值；
- `R0..R3` 为保留字节，当前为 `0`。

`0xC7` 前方扫描结果为 7 字节：

```text
AA 55 C7 CURRENT_ID MASK COUNT CHECKSUM
```

`MASK` 的 bit0..bit4 对应 ID1..ID5，`COUNT` 是检测到的其他颜色 ID 种类数。结果连续稳定 6 帧时提前发送，最多观察 12 帧。

`0xC8` 回库黄线结果为 7 字节：

```text
AA 55 C8 STATUS Y_LO Y_HI CHECKSUM
```

`STATUS.bit0` 表示 Y 有效，`STATUS.bit1` 表示达到停车阈值；Y 是 `0..239` 的图像纵坐标。

`0xC9` 候选枚举结果为 12 字节：

```text
AA 55 C9 SEQ INDEX TOTAL CID X_LO X_HI Y_LO Y_HI CHECKSUM
```

X/Y 是有符号 `int16` 毫米坐标。无候选时仍发送一帧 `INDEX=0`、`TOTAL=0`、X/Y 为 0 的结果，主控不应对此发送 `0x0A`。

## 关键运行行为

### 目标识别和跟踪

TFLite 模型负责全局发现、类别确认和周期性重捕；LAB 色块负责细分颜色 ID 和提供模型帧之间的中心位移。熊类共用一个模型 label，首次锁定需要颜色身份和几何连续性共同成立。主控指定 ID4/ID5 时仍不会绕过颜色确认。

已经完成的 ID 默认退出普通搜索和 `0x03` 指定。完成记录只保存在 RAM，重启或发送 `0x08` 后清零。`0x06` 前扫保留全色语义，不受搜索优先掩码限制。

### 主车搬运黄线

主车使用五段水平 ROI、相邻 / 隔段连接、拟合触底和横向侧扫共同判断黄线。ID3 搬运时会避开网球框，降低球体与黄线粘连造成的漏判。确认过线的同一帧立即发出 `POS=0x02`，并将本轮锁存 ID 记为完成。

这些阈值针对比赛场地和相机安装调过，修改 ROI、曝光或图像方向后必须在真车上重新回放完整的进线、遮挡、触底和离线流程。

### 调试日志

最终版本默认关闭主车运行日志、黄线逐帧日志、搬运 JPEG、从车搬运日志和熊类日志，避免 SD I/O 影响帧率。相关开关仍保留在两个入口顶部附近，仅在复现问题时临时开启，复现后应关闭并清理 SD 卡上的日志和照片。

## 验证

桌面环境可执行以下发布前检查：

```powershell
python -m py_compile main.py minimain.py `
  calib_ide_autocalib_competition.py calibrate_ground_camera.py
mpy-cross main.py -o main.mpy
mpy-cross minimain.py -o minimain.mpy
python calibrate_ground_camera.py `
  --ground-csv ground_mesh_24_points_template.csv `
  --role master --expected-points 28 `
  --required-near-y-cm 6 --max-y-cm 164 `
  --output camera_ground_mesh.txt `
  --report camera_ground_mesh_report.json
git diff --check
```

仓库没有提交板端 API 的桌面模拟测试，`.gitignore` 会排除本地 `*test*.py`、`tests/`、日志、模型和 `.mpy` 产物。Python 编译和 `mpy-cross` 只能检查语法与固件兼容性，不能替代以下真机验证：

- 两块板分别识别五种目标，并正确区分红蓝沙包和棕白熊；
- `0x00..0x0C` 帧边界、坏校验重同步和主控状态机；
- 28 点坐标方向、近场边界和网格外回退；
- 主车普通物体及网球搬运过线；
- 两车 `0x06` 前扫、`0x07/0xC8` 回库黄线；
- 关闭调试日志后的长时间脱机运行。

## 常见问题

### 上电后没有目标坐标

先检查 `[MODEL]` 和 `[MODEL ALARM]` 输出，再检查 `color_thr.txt`、当前 `0x0C` 策略和已完成颜色掩码。发送 `AA 55 08 08` 可清空 RAM 中的完成记录。

### 坐标明显不准或方向相反

确认 `camera_ground_mesh.txt` 成功加载，并核对相机安装、`vflip`、软件镜像和 CSV 坐标方向。更换相机位置后必须重新采点，不能只调整 LAB 阈值。

### 主从车识别结果差异大

两车最终默认白平衡和曝光本来就不同。应分别运行颜色标定，并确认模型文件和固件版本一致。

### 串口偶发失步

主控应按实际命令长度解析和发送，持续搜索 `AA 55` 帧头，并验证低 8 位校验和。特别注意 `0x09/0x0A` 为 6 字节、`0x03/0x04/0x0C` 为 5 字节，其余当前命令为 4 字节。

## 已知限制

- 这是比赛现场单文件部署代码，不是通用视觉 SDK；主从共有逻辑存在有意重复。
- 最终源码发布树不包含模型、训练数据和 RT1021 主控工程，单独检出 `main` 不能组成完整车辆系统。
- 内置 LAB 阈值、黄线阈值和相机参数只代表最后一套硬件与场地。
- 两份入口当前共用 `role=master` 地面网格，双相机独立安装时需要各自标定和配置。
- 历史更新日志中的部分本地测试脚本未随仓库发布。

## 贡献

提交问题时请至少说明板卡型号、OpenART 固件版本、主车或从车、模型 label 顺序、相机参数、实际 UART 帧以及完整错误输出。涉及识别问题时，建议同时提供原始无叠加画面和对应的 `color_thr.txt`，不要只提供裁剪后的单张目标图。

代码修改应保持 OpenART MicroPython 兼容，避免在逐帧热路径中增加无界内存分配或同步 SD 写入。主从共有协议发生变化时，应同时检查 `main.py`、`minimain.py` 和本文档。

## 许可证

本仓库源码和文档使用 [MIT License](LICENSE)。外部模型、训练数据、OpenART 固件及其他第三方内容不因本许可证自动获得授权，使用者需自行确认相应许可。
