# OpenART 视觉更新日志

本仓库用于记录双 OpenART Plus 智能车视觉脚本的迭代；当前主车与从车硬件均为 OpenART Plus。

## 当前文件

| 文件 | 设备 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus / 主车 | v0.11.4-dev 模型识别 / 稳定接触点融合入口 |
| `minimain.py` | OpenART Plus / 从车 | v0.11.4-dev 模型识别 / 稳定接触点融合入口 |
| `camera_ground_mesh.txt` | OpenART Plus / 主从 | 板端加载的 28 点、36 三角形地面网格 |
| `ground_mesh_24_points_template.csv` | PC | 当前 28 个像素/世界坐标标定点 |
| `calibrate_ground_camera.py` | PC | 网格生成、校验和报告工具 |
| `camera_ground_mesh_report.json` | PC | 当前网格质量报告 |
| `raw_ground_projection_test.py` | OpenART Plus / IDE | 地面坐标采点和实地复核脚本 |
| `world_coordinate_test.py` | OpenART Plus / IDE | 与主车完整检测对齐的全类别世界坐标观察脚本 |
| `test_ground_projection.py` | PC | 运行时投影回归测试 |
| `test_model_blob_fusion.py` | PC | 主从模型 / 色块融合回归测试 |
| `calib_ide_autocalib_competition.py` | OpenART Plus / IDE | 比赛现场自动标定与预览脚本 |
| `front_obstacle_scan_test.py` | OpenART Plus / IDE | 搬运前前方色块扫描预览脚本 |

## 结构说明

- 当前部署继续使用单文件结构，`main.py` 和 `minimain.py` 分别维护主车与从车的完整主逻辑；两者启动时额外加载 `/sd/camera_ground_mesh.txt`。
- 多文件运行模块已移除，不再使用 `openart_app.py`、`openart_config.py`、`openart_detectors.py`、`openart_trackers.py`、`openart_uart.py`、`openart_math.py`、`openart_camera.py`、`openart_calibration.py`。
- 颜色检测、黄线状态、UART 协议和主循环仍在单文件主程序内；地面网格生成和实地复核保留为独立工具。
- `fast_blob_backup/`、`stable_confirm/`、`stable_no_priority/` 与 `mainbak` 仅作历史对照，不是当前入口。
- 多文件版本曾导致 TFLite 检测卡死，原因和维护约束见 v0.4.0 日志；不要把 v0.3.0 的模块化结构重新作为比赛部署结构。
- 根目录 `README.md` 只保留当前部署摘要，完整迭代记录写入 `README_ch.md` / `README_en.md`。

## 更新日志

> **当前双车硬件规则：主车和从车均为 OpenART Plus，`main.py` 与 `minimain.py` 都固定使用 `UART12`、115200 bps。两份文件分别固定为主车/从车入口，不再保留无实际引用的 `IS_SLAVE_CAR` / `SLAVE_MODE` 开关。**

### 2026-07-25 - v0.11.4-dev - 近距离世界坐标接触点稳定

范围：`main.py`、`minimain.py`、`world_coordinate_test.py`、`test_model_blob_fusion.py`、三份 README。

- 对照省赛稳定版确认，原版低抖动主要来自模型接触点的空间限幅；v0.11.3 的完整色块底边会直接吸收阴影、粘地和阈值边缘变化，近距离时这些像素变化被逆透视放大。
- 保留完整色块对显示框的所有权，在逆透视之前单独稳定底边中点：默认 `2 px` 圆形空间死区，小变化保持不动，超出后立即跟随并只保留 `2 px` 空间误差。该方法没有普通时间 EMA 的长拖尾。
- 原始接触点相对上一稳定点达到 `24 px` 时直接重置，处理真实快速移动和重捕；显示框继续采用原 `35%` 当前值平滑，不恢复省赛版偏小的模型框尺寸。
- 观察脚本同时打印 `raw_pixel`、`stable_pixel` 和 `delta_px`，并以小红十字、大黄十字分别标记原始和稳定接触点。
- 28 个标定点、36 个三角形、图像中心 X 修正、单应回退、Y 坐标和 UART 毫米单位均未修改。
- 共 18 项桌面回归测试覆盖小抖动冻结、近距离底边往返噪声收窄、真实移动后的最终收敛、大跳变立即重置、显示框/坐标框分离以及三份入口一致性。

### 2026-07-25 - v0.11.3-dev - 可切换 ID2 绝对优先

范围：`main.py`、`minimain.py`、`world_coordinate_test.py`、`test_model_blob_fusion.py`、`test_ground_projection.py`、三份 README。

- 从省赛 `01_稳定版_ID2先_5中7` 恢复首目标门控，并在三份当前入口的快速配置区增加 `ID2_ABSOLUTE_PRIORITY`，默认开启。
- 开启时，上电或 `0x08` 清零后只允许 ID2 进入普通模型搜索、动态 LAB 确认、主控 `0x03` 指定和世界坐标输出；即使其他 ID 更近或置信度更高也不会锁定。ID2 完成后，其余未完成 ID 自动恢复最近目标竞争。
- 主车沿用越过黄线时写入完成 mask，从车沿用搬运结束后提交待完成 ID；`0x06` 全色前扫继续绕过优先门控。场上没有 ID2 时会持续等待，不自动降级。
- 关闭开关时恢复 v0.11.0 的无颜色优先级行为，所有未完成 ID 从启动起按世界 Y 最近优先。
- 共 14 项桌面回归测试通过，新增测试覆盖 ID2 未完成、ID2 已完成和关闭开关三种可用 ID 集合；Python 语法、`git diff --check` 通过。

### 2026-07-25 - v0.11.2-dev - 全类别世界坐标观察脚本

范围：`main.py`、`minimain.py`、`world_coordinate_test.py`、`test_model_blob_fusion.py`、`test_ground_projection.py`、三份 README。

- 新增独立 OpenART IDE 入口 `world_coordinate_test.py`。它完整保留当前 `main.py` 的三类模型、五个颜色 ID、首次 `5/7` 锁定、动态 LAB、模型/色块几何融合、最近目标选择、地面网格和单应回退，不使用红沙包专用的简化检测。
- 对任意正常锁定类别打印最终坐标框底边中点、厘米世界坐标和正式 UART 使用的毫米坐标；画面同步标出接触点。默认每 `200 ms` 打印，目标丢失时每秒输出一次 `NO_TARGET`。
- 主车、从车和测试脚本统一按接触点所在图像行扣除 `u=160` 的原始 X 投影，使画面中心线在所有距离严格对应 `X=0`，同时保留 Y 和横向尺度；共用 `GROUND_CENTER_X_ON_IMAGE` 开关可恢复旧标定作现场 A/B 对比。测试脚本额外打印 `raw_x` 与 `x_bias`。
- 输出状态 `HELD / TRACK / MODEL_FRAME` 对应保持帧、普通跟踪帧和模型刷新帧，便于区分坐标跳变发生在哪条检测路径。
- 新增完整 AST 一致性保护：剔除 `_world_coord_*` 观察代码后，测试脚本必须与 `main.py` 完全一致；它同时参与模型/色块融合和地面投影回归测试。
- 共 13 项桌面回归测试通过，包括多距离图像中心严格映射到 `X=0`；Python 语法、`git diff --check` 和 MicroPython `mpy-cross` 编译通过。

### 2026-07-23 - v0.11.1-dev - 模型识别与动态色块几何融合试验

范围：`main.py`、`minimain.py`、`test_model_blob_fusion.py`、三份 README。

版本留存：

- 改动前的省赛 v0.11.0 已保存在提交 `41260c0`；分支 `dedicated-model` 与 `archive/v0.11.0-ground-mesh` 均指向该提交并已推送到 `origin`。当前试验在 `model-blob-fusion` 分支继续，不覆盖省赛归档。
- 归档仍以实车使用的 `04_备用版_无优先级_5中7` 为基线，保留 SS 模型、`880 us` 曝光、无 ID 优先级的最近目标选择、首次 `5/7` 确认、28 点地面网格和 UART 毫米单位。

融合行为：

- 原实现由模型框决定宽高，色块只提供相对位移；模型未完整包住物块时，最终框仍会继承偏小的模型尺寸，并在每 4 帧模型刷新时出现周期性变化。新实现删除这条模型锚点几何路径。
- 模型继续负责首次发现、模型类别确认、动态 LAB 阈值建立、定期复检和丢失后的重捕。动态色块一旦确认，色块框直接负责屏幕显示框以及世界坐标所用的底边接触点。
- 第一次寻找完整色块时，将模型框按 `50%` 扩大后搜索；已有色块后使用按 `45%` 扩大的局部 ROI。模型刷新帧会合并模型门和已有色块门，但仍沿用色块局部 ROI，不会因刷新强制缩回不完整的模型框。
- 首次色块候选的中心距离容差同时参考模型框和候选色块框尺寸，允许完整色块明显大于残缺模型框。面积、重叠、颜色 ID 和动态区域约束仍用于排除邻近干扰物。
- 最终框采用旧值 `65%`、当前色块 `35%` 的中心和尺寸平滑；大幅跳转仍直接切换，避免平滑拖尾。单帧色块丢失保持上次输出，连续 3 帧未命中时清空局部色块搜索状态，输出最多保持 5 帧，随后才使用模型几何或等待重捕。

限制与验证：

- 该版本是待上车对比的开发试验，不替代归档的省赛完成版。色块现在拥有几何解释权，因此动态阈值若切碎物体、粘连地面或吸收阴影，框与世界坐标仍可能抖动；需要分别观察原模型框、原色块框和最终平滑框。
- 新增 6 项融合回归测试，覆盖首次扩框、模型刷新 ROI、色块几何所有权、平滑权重、旧锚点路径清理，以及主从全部融合常量和辅助函数的 AST 一致性。
- 融合测试 6 项和原世界坐标测试 5 项全部通过；`python -m py_compile`、`git diff --check` 通过，`mpy-cross` 成功编译 `main.py` 与 `minimain.py`。

### 2026-07-22 - v0.11.0 - 省赛 04 无优先级多点世界坐标版

范围：`main.py`、`minimain.py`、`ground_mesh_24_points_template.csv`、`calibrate_ground_camera.py`、`camera_ground_mesh.txt`、`camera_ground_mesh_report.json`、`raw_ground_projection_test.py`、`test_ground_projection.py`、三份 README。

基线与运行参数：

- 正式主从入口改以实车烧录目录 `04_备用版_无优先级_5中7` 为基线，不按颜色 ID 或模型类别设置自动搜索优先级；候选统一按世界 Y 近到远选择，并保留 `7` 个真实推理帧至少命中 `5` 帧的首次锁定。
- 主控 `0x03` 指定颜色仍是显式定向搜索，保留 04 版 `0.25` 门槛和 `3/5` 确认，不属于自动 ID 优先级。
- 主从模型统一为 `/sd/80lite0.5SS.tflite`，固定白平衡 `(92.00, 64.00, 101.00)`，缺少板端曝光配置时使用省赛场地 `880 us`。`/sd/color_thr.txt` 中的 `exposure_us=` 仍可覆盖默认值。
- 颜色识别、动态地面裁切、搬运状态、前方扫描、回库黄线以及主从 16 字节 UART 包结构保持省赛完成版行为。

多点世界坐标重构：

- 从历史提交 `fac7b92` 恢复多点坐标工具思路，以现有 `ground_mesh_24_points_template.csv` 重建；文件名虽保留 24，当前数据实际为 7 行 x 4 列共 28 个拟合点。
- PC 生成器按近到远重排结构化行，输出 36 个三角形和 18 条带方向分类的外边界。网格内部使用重心插值，能够精确复现每个标定顶点。
- 网格外部使用全部 28 点拟合的全局单应矩阵，并按最近网格边界补偿局部/全局偏差；远端限制为 `164 cm`，X 限制为 `-250..250 cm`。
- 运行端严格检查网格 schema、角色、QVGA 尺寸、软件水平镜像、传感器垂直翻转、禁用 `lens_corr()`、Y 范围、三角形方向和回退矩阵。文件缺失或不合法时使用内置全局单应矩阵，不假装仍在使用局部网格。
- 世界坐标接触点统一为目标可见底边中点 `(x + w/2, y + h - 0.5)`。内部以厘米计算，发送前四舍五入为毫米；保留省赛协议，不恢复历史 v1.0.0 的 `0.1 mm` 缩放。
- 当前主从入口有意共用 `role=master` 网格。若从车安装几何不同，必须另采从车数据、生成 `role=slave` 网格并同步修改从车角色检查。

生成结果与限制：

- 标定 Y 范围 `6..164 cm`，网格覆盖 QVGA 画面的 `61.9%`，最小三角形角 `7.82 deg`。
- 全局单应矩阵拟合 RMS / 最大误差为 `1.679 / 3.281 cm`；留一法诊断 RMS / 最大误差为 `1.994 / 4.056 cm`。
- 当前 28 个点全部为 `split=fit`，没有独立 `verify` 点。生成器通过只证明结构和内部约束成立，最终精度仍需使用 `raw_ground_projection_test.py` 在两块相机上实测。

仓库清理：

- 删除不再使用的 `calib_ide_tune.py`、`capture_field_images.py`、`color_thr.txt`、`image.png`、`main_autocalib_test.py`、`match_field_capture_to_reference.py` 和 `return_yellow_test.py`；这些文件仍可从 Git 历史恢复。

验证：

- 网格生成器成功输出 28 点 / 36 三角形 / 18 边界，报告 QA 通过并明确提示缺少独立验证点。
- `test_ground_projection.py` 共 5 项通过：主从投影代码一致、28 个顶点精确复现、全部 76,800 个 QVGA 像素均返回有界坐标、UART 毫米舍入正确、04 模型与曝光参数正确。
- `python -m py_compile`、`git diff --check` 通过；MicroPython v1.27.0 `mpy-cross` 成功编译 `main.py`、`minimain.py` 与 `raw_ground_projection_test.py`。

### 2026-07-19 - v0.10.5 - 稳定首次锁定自动识别修复

范围：`stable_confirm/main.py`, `stable_confirm/minimain.py`, `stable_confirm/README.md`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 删除稳定版必须先收到 `0x02` 并等待 `300 ms` 才运行首次模型识别的硬门控，修复 IDE 单跑或主控尚未发送 `0x02` 时始终无目标的问题。
- 保留按世界距离选择最近候选、统一 `0.30` 首次门槛、`7` 个真实推理帧至少命中 `5` 帧以及高置信度不能单帧锁定的规则。
- LAB 动态颜色确认恢复为根目录默认版的 `3` 帧；`0x02`、`0x00` 只负责清空当前目标并重新搜索，不再控制识别权限。同一阶段重复的 `0x02` 保留去抖，不会反复清空 `5/7` 计数。

验证：主从脚本语法通过；代码中不再存在首次锁定 cycle/settle 门控引用，模型路径和预处理保持与根目录默认版一致。

### 2026-07-19 - v0.10.4 - 通信清零与稳定首次锁定版

范围：`main.py`, `minimain.py`, `stable_confirm/main.py`, `stable_confirm/minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 新增下行 `0x08` 清零命令，四字节帧为 `AA 55 08 08`。四份代码收到后都清空 ID1..ID5 完成 mask、解除当前单色/模型锁并恢复全模型搜索；从车还会清空待提交搬运 ID。
- 根目录保留原有快速版，`stable_confirm/` 提供可独立部署的主车/从车版本。稳定版把 `0x02` 作为就绪触发，等待 `300 ms` 后以统一 `0.30` 候选门槛严格按世界距离从近到远选择，并用 `7` 帧中的 `5` 帧完成首次锁定。
- 稳定版取消高置信度单帧首次锁定；复检过程中出现更近候选时立即改为确认更近目标。若 `0x08` 到来时已经处于 `0x02` 周期，则重新执行停稳和复检。

验证：四份脚本语法通过；模拟串口验证完成 mask、从车待提交 ID、目标锁和前扫状态均被正确清空；近处 `0.31` 候选优先于远处 `0.99` 候选。

### 2026-07-19 - v0.10.3 - 停用红沙包专用长宽比过滤

范围：`main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 注释掉颜色 ID2 的 `宽/高 <= 1.70` 专用过滤，以及模型框颜色采样中的同款检查；代码保留为注释，便于现场需要时恢复。
- 红沙包重新使用原有通用沙袋规则 `0.60 <= 宽/高 <= 1.80` 和密度 `>= 0.40`，避免正常红沙包被专用上限误杀。
- 标定十帧复检、标定运行预览和带主控命令的测试入口同步停用专用上限；`0x06` 本来就不检查目标长宽比，行为不变。
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py` 通过，`git diff --check` 通过。

### 2026-07-19 - v0.10.2 - 每个颜色 ID 只有效搬运一次

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 新增 ID1..ID5 完成 bit mask。主车只在搬运模式首次确认黄线穿越并产生 `POS_CROSSED` 时记录当前 ID；普通识别、黄线出现但未越过以及搜索重置均不会计为完成。
- 从车没有搬运黄线状态机，也不依赖自动流程中可能不会到达的 `0x02`：收到 `0x01` 时仅锁存本轮 ID，下一次 `0x00` 或 `0x02` 搬运结束/搜索复位时才提交完成。没有待确认搬运时，普通 `0x00` 不会记录任何 ID。
- 已完成 ID 从正常模型候选、模型引导颜色采样和最终坐标输出中排除；重复的 `0x03,id` 会清空目标状态而不会再次锁定。若同一模型大类还有未完成颜色，例如 ID1 已完成但 ID2 未完成，运行时会先取 LAB 颜色再决定是否接受候选。
- `0x06` 继续返回所有有效色块 ID，不使用完成 mask 过滤；完成状态与从车待确认 ID 仅保存在 RAM，重启 OpenART 后清零。
- `python -m py_compile main.py minimain.py` 通过，`git diff --check` 通过。

### 2026-07-19 - v0.10.1 - 红沙包与场地红砖形状分离

范围：`main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `front_obstacle_scan_test.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 为颜色 ID2 红沙包增加独立横向长宽比上限：候选框必须满足 `宽/高 <= 1.70`。该值由初版 `1.50` 放宽，以容纳正常沙包的透视和色块破碎；原有 `宽/高 >= 0.60`、密度和像素门槛保持不变，ID1 蓝沙包仍使用 `0.60..1.80`。
- 主车和从车在正常目标 LAB 候选与模型引导颜色采样阶段执行该规则，防止模型检测兜底重新接受明显横向扁长的红砖。
- `0x06` 保持“全部有效色块 ID”语义，使用独立于目标形状的扫描校验：通过颜色阈值、像素/面积、密度和有效区域检查的所有其它 ID 都写入 mask，横向红砖仍作为 ID2 返回；当前跟踪目标由 `current_id` 单独返回。
- 同步更新前方全色扫描测试、现场自动标定十帧复检、标定运行预览和带主控命令的自动标定测试，保证各入口语义一致。
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py front_obstacle_scan_test.py` 通过。

### 2026-07-16 - v0.10.0 - 模型引导现场标定与运行识别稳定化

范围：`calib_ide_autocalib_competition.py`, `main.py`, `minimain.py`, `front_obstacle_scan_test.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 将比赛标定脚本升级为完整五目标连续流程，当前构建号为 `2026-07-15 red-bag-bear-core-v21`。模型只在 IDE 标定时从 `/sd/dataset_25000_exposure.tflite` 加载；主车和从车正式程序仍使用 LAB 色块，不依赖模型文件。
- 自动曝光目标由 `L=40` 下调到 `L=38`，收敛容差收紧为 `±1`，全画面亮度上四分位达到 `Luq=92` 时提前执行高光保护，降低白熊和浅色物体过曝导致色度丢失的风险。
- 自动曝光完成后立即更新正式配置的 `exposure_us` 行，保留已有地面和五色阈值。即使后续标定失败曝光也已经改变，因此重新标定前应备份已有可用配置。
- 近处 `ground` 和远处 `ground2` 改为分块采样：排除与模型框重叠的小块，剔除 LAB 中位数离群块，再分别生成六维地面盒，减少目标、手和局部光斑污染地面阈值。
- 模型框不再只按固定比例横向/纵向扩张。脚本从框内核心沿四边多条射线扫描 LAB 跳变，并从最外侧连续地面或稳定背景反向确认真实边界；边界证据不足时才回退到 bear / ball / bag 各自的保守扩框。
- 网球改用中央核心区加可选下部内缩条带取样；下部条带命中地面、IQR 过宽或与核心 A/B 中位数差异过大时不合并，兼顾球体暗边和蓝地污染。
- 沙袋使用核心区判断地面冲突，同时允许通过 LAB 一致性检查的边缘条带补入。网球、沙袋、棕熊、白熊分别使用独立的 A/B 跨度上限，当前为 `75 / 60 / 75 / 60`。
- 熊类限制模型框向下扩散并裁掉统计区底部；棕熊阈值的 L 下界不低于本体中位数减 `14`，避免把脚下大面积暗投影纳入棕熊阈值。
- 标定阶段固定为“两个沙袋任意顺序 → 网球 → 棕熊 → 白熊”。模型只输出统一 bear 类别，因此棕熊和白熊通过阶段强制分槽；上一目标必须移出画面并连续清场 `8` 帧，防止棕熊残留进入白熊槽位。
- 每个目标采集 `10` 个合格样本。棕熊、白熊和红沙袋增加模型框稳定门；红沙袋跳到新位置后，连续 `3` 帧新框稳定即可自动重建基准并从新位置重新采样，避免永久卡在旧框。
- 每个新阈值增加 `10` 帧模型/色块对照复检，检查无色块、框完全分离、中心偏移、重叠率、相对面积、宽高覆盖以及帧间中心/尺寸/面积跳变。连续异常 `3` 帧、累计异常 `4` 帧或严重跳变 `2` 次会作废当前阈值并立即重采。
- 棕熊与白熊全部完成后，按两者 LAB 中位数差异最大的通道强制分离；要求至少 `8` 的中位数间距，并在切点两侧各留 `2` 的余量。分离后再次审计阈值无重叠且各自中位数仍在自己的盒内，失败时两个熊槽位一起作废。
- 只有完整且通过审计的五个槽位覆盖 `/sd/color_thr.txt`；不完整结果写 `/sd/color_thr_partial.txt`。正式程序只在正式文件包含完整 `1..5` 五槽位时一并加载曝光、颜色和地面参数。
- `main.py` 和 `minimain.py` 现在都解析 `ground` 与 `ground2`，对六个 LAB 边界逐项取整数平均作为动态蓝地阈值；缺少一组时使用另一组。白熊与平均地面 A/B 真正重叠时，再用 L 通道把白熊下界抬离地面。
- 主车动态裁切使用横跨画面的五条窄竖带，至少要求三条有效且横向跨度达到 `180 px`，并增加地面间隙桥接、稳健位置、单帧升降限幅和 EMA。目标 ROI 从边界上方 `10 px` 开始，跨过边界的目标仍保留。
- 从车改为五条竖带至少两条有效后取顶部水平平均并做 EMA；当前实现不是 v0.9.9 曾记录的左右端点斜裁切线，文档已按现代码纠正。
- 多颜色搜索改为一次非合并 `find_blobs()`；阈值重叠产生的多 bit blob 被视为歧义，不再默认归入较小颜色 ID。锁色后继续使用局部 ROI、IoU、中心距离和面积变化保持目标连续性。
- 棕熊和白熊分别用 `12 px`、`10 px` margin 合并毛绒碎片。白熊输出框使用旧框 `2/3`、新框 `1/3` 的平滑，减少监视框忽大忽小；严重跳框时直接切换新框，避免平滑拖尾。
- 新增网球阴影关系过滤：棕色候选位于网球下半部附近、横向重叠达到 `60%` 且面积不超过网球 `55%` 时按球底阴影丢弃；锁定棕熊时也会额外取网球阈值作为参照。
- 世界坐标改用预计算单应矩阵和目标底边中点，不再在 OpenART 上求解 8×8 方程或平均四角。X 限制在 `±250 cm`、Y 限制在 `0..300 cm`，修复固件中 tuple/type 参与浮点运算及越过投影地平线的问题。
- 搬运前 `0x06` 扫描门槛调整为 `60` 像素；`current_id / mask / count` 连续一致 `6` 帧时提前返回，最多观察 `12` 帧，超时则发送第 12 帧当前结果。当前搬运目标仍按 IoU 或中心距离排除，返回协议保持 `0xC7` 七字节包。
- `front_obstacle_scan_test.py` 同步当前颜色最小像素/面积、单次多颜色扫描、水平动态 ROI 和最下方搬运目标排除逻辑，便于脱离主控观察候选框与 mask。
- 主车搬运黄线 LAB 阈值调整为 `(62, 100, -57, 13, -8, 127)`；新增与当前目标大面积重叠过滤、近竖直拟合过滤和 `±45°` 斜率限制，并在调试画面绘制实际拟合线。
- 保持现有双车 `0x07` 回库横向黄线流程，并在根 README 中补全 `0xC7`、`0xC8` 七字节包格式。主车和从车各自保留独立回库黄线阈值。
- 增加 OpenART 固件兼容保护：显式整数/标量循环替换易出问题的生成器式解析，阈值写入逐字段格式化并回读确认，LAB 中位数在运算前解包校验，`find_blobs(..., margin=...)` 不支持时自动回退，无效世界坐标和单帧颜色检测异常不会直接终止主循环。

部署与验证：

- 在 OpenART IDE 运行标定脚本，按提示完成五目标并确认 `[bear] PASS ... overlap=0`；只有正式五色文件可部署，不能把 partial 文件改名冒充正式配置。
- 主车部署 `main.py`，从车部署 `minimain.py`，两者在各自 SD 卡上均保存为 `/sd/main.py`，并分别使用本机生成的 `/sd/color_thr.txt`。
- `python -m py_compile calib_ide_autocalib_competition.py main.py minimain.py front_obstacle_scan_test.py` 已通过。
- 四个修改脚本的 OpenART `mpy-cross` 编译已通过，`git diff --check` 已通过。
- 桌面编译无法覆盖摄像头 API、现场灯光和 MicroPython 固件差异；比赛前仍需在两块 OpenART Plus 上复测五目标、棕白分离、网球阴影、动态裁切、`0x06` 六帧确认、`0x07/0xC8` 回库黄线和长时间运行。

### 2026-07-14 - v0.9.9 - 网球搬运黄线特例与从车裁切修复

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 主车新增 `carry_target_color_id` 搬运颜色快照。收到 `0x01` 时，只有主控 `target_color_id` 与当前本地 `color_track_color_id` 一致，才确认本轮搬运颜色；锁色残留、目标丢失或颜色不一致时不启用任何网球特例。
- 搬运网球时，黄线上下检测 ROI 分别放宽为 `y=70..119` 和 `y=120..169`，首次/保持像素门槛使用 `7/5`。黄线首次拟合成功后，同一帧直接发送 `POS_CROSSED` 并进入 `MODE_WAIT_TURN`，不再等待连续确认、触底或丢线。
- 非网球目标继续使用原 ROI、`70/20` 像素门槛和两阶段过线状态机，修复红色沙包误用低门槛并提前结束搬运的问题。
- 从车恢复自身蓝地 LAB 阈值、左右采样端点和斜裁切线插值；目标色块按自身横坐标与裁切线比较，修复五条采样带统一取平均后在斜视角下出现的误裁和误识别。

验证：
- 现场复测反馈搬运流程已经稳定，红色沙包不再触发网球提前退出。
- `python -m py_compile main.py minimain.py` 已通过。
- 搬运颜色命令快照、网球专用提前退出、黄线 ROI/像素参数隔离和从车斜裁切分支测试已通过。
- `git diff --check` 已通过。

### 2026-07-12 - v0.9.8-dev - 脱机热路径精简与视觉回库移除

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 主车和从车均删除视觉回库模式、回库信标阈值/跟踪状态及对应 `find_blobs()` 路径；`0x05` 仍按 4 字节命令完成校验和缓冲区消费，但不触发状态切换。
- 对照最新 RT1021 主控目录确认业务代码不再发送 `0x05`，现用命令为 `0x00/0x01/0x02/0x03/0x04/0x06`；16 字节世界坐标回传格式保持不变。
- 完整删除鸟瞰开关、反向单应矩阵、额外帧缓冲和逐像素渲染；保留 `CALIBRATION_MODE`、四点 IPM 标定采点、验证画面和 `H_pix2world` 坐标换算。
- 正常脱机路径保留按颜色显示的目标外接框，删除目标十字/文字、动态裁切线和黄线调试线；同时移除无效距离计算、启动横幅和运行期调试字符串。
- 删除零调用的亮度校准函数链、旧 14 字节像素协议、旧局部 ROI 状态、旧锁色计数、无效角色开关及只写不读的黄线边界世界坐标。固定曝光、SD 阈值加载、颜色/黄线识别参数及状态机不变。
- 缓存单颜色阈值列表和固定裁切 ROI；候选筛选改为等价单遍选择；世界坐标四角换算不再创建临时角点列表。
- 预分配有目标/无目标两个 16 字节 UART 缓冲区，并改用无切片校验和，避免每帧创建 `bytearray` 和 `data[2:15]` 临时切片。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- 初始候选选择新旧实现随机对照 `10000` 组完全一致。
- 有目标/无目标 UART 新旧编码随机对照 `4000` 帧逐字节一致。
- `0x05` 后紧跟 `0x03` 的解析测试通过，命令缓冲保持同步。
- `git diff --check` 已通过。

### 2026-07-11 - v0.9.7-dev - 主车黄线阈值与文档同步

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 将主车黄线 LAB 阈值从 `(48, 94, -27, 51, 12, 127)` 调整为 `(51, 91, -32, 36, 1, 118)`。
- 从车继续保留独立的黄线阈值和现有 IPM 数值；标定注释改为当前 OpenART Plus 硬件表述，并明确旧 Mini 安装参数不能直接沿用，必须在当前实机重新标定。
- 根 README 同时标明最近稳定版与当前开发版；补充 v0.9.5/v0.9.6 的基线差异说明，并补齐英文 v0.7.5-dev 历史记录。

验证：
- 仓库内全部 Python 文件语法检查已通过。
- `git diff --check` 已通过。

### 2026-07-11 - v0.9.6-dev - ID1/ID2 独立最小像素阈值

范围：`main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 主车和从车新增 `COLOR_ID12_MIN_PIXELS = 100`，明确让 ID1、ID2 使用 `100` 像素；相对 v0.9.5-dev 中间版本由 `150` 降为 `100`，相对稳定版 v0.9.0 的净值保持 `100` 不变。
- 默认 `COLOR_MIN_PIXELS = 150` 现在用于熊类 ID4、ID5；相对 v0.9.0，其 `find_blobs()` 初筛净值由 `100` 提高到 `150`，原有二次过滤保持不变。网球 ID3 继续使用 `45` 像素。
- `0x06` 搬运前扫描的 `FRONT_SCAN_MIN_PIXELS = 150` 保持不变。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` 已通过。

### 2026-07-11 - v0.9.5-dev - 双车目标 ROI 调整至蓝地边界上方 10 px

范围：`main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 将主车和从车动态目标检测 ROI 上界统一改为蓝地边界上方 `10 px`。
- `main.py` 与 `minimain.py` 的 `CUT_ROI_Y_OFFSET` 均调整为 `-10`；计算仍使用 `蓝线位置 + CUT_ROI_Y_OFFSET`。
- 相对 v0.9.4-dev 的边界下方 `3 px`，本次向上移动 `13 px`；相对稳定版 v0.9.0 的边界上方 `6 px`，净变化为向上 `4 px`。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` 已通过。

### 2026-07-11 - v0.9.4-dev - 从车回退至 v0.9.1 旧结构

范围：`minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 撤销 v0.9.2 中将 `minimain.py` 按 `main.py` 重建的重构，恢复 v0.9.0/v0.9.1 系列经过从车现场验证的旧文件结构、旧命令处理和旧黄线状态机。
- 恢复从车左右垂直黄线 ROI、底部向上扫描、`yellow_raw_detected`、`YELLOW_CARRY_HOLD_FRAMES = 40` 和原有搬运过线判定流程。
- 移除重构带入从车的 WDT、主车底角黄线拟合状态、主车水平黄线双 ROI 及对应辅助函数。
- 保留 v0.9.1 的性能优化：删除橘色障碍逐帧扫描和目标重叠排除，`obstacle_flag` 固定为 `0`，动态目标 ROI 使用蓝地边界下方 `3 px`，普通颜色最小像素为 `150`。
- 保留当前双 OpenART Plus 硬件规则，从车继续固定使用 `UART12`、115200 bps。
- 本次只回退 `minimain.py`；`main.py` 不随从车一起回退。

验证：
- `python -m py_compile minimain.py` 已通过。
- `minimain.py` 相对 v0.9.0 基线仅保留 v0.9.1 性能修改和 UART12/3 px 当前参数。
- `git diff --check -- minimain.py README_ch.md README_en.md` 已通过。

### 2026-07-10 - v0.9.3-dev - 双车 Plus UART12 规则固化

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 明确当前主车和从车硬件均为 OpenART Plus，两份正式程序都无条件初始化 `UART(12, baudrate=115200)`。
- 删除 `main.py` 中从车角色切换到 UART2 的旧硬件分支，并删除 `minimain.py` 中无意义的相同分支。
- `IS_SLAVE_CAR` 只保留为软件业务角色开关，用于候选颜色和主机 `0x03` 锁色流程，不再影响串口编号。
- 在根 README 和中英文详细 README 顶部写明双车统一使用 UART12，避免后续维护再次恢复 UART2。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- `rg -n "UART\\(2" main.py minimain.py` 无匹配。

### 2026-07-10 - v0.9.2-dev - 从车运行主线同步与防卡死清理

范围：`minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 以当前 `main.py` 稳定实现为基线重建 `minimain.py`，主从车现在共享相同的命令解析、目标搜索、局部跟踪、动态裁切、黄线状态机、回库识别和主循环结构。
- 删除从车旧版左右黄线 ROI、扫描条、旧搬运保持计数、旧原始黄线状态及其分叉逻辑。
- 删除从车启动横幅、命令处理和运行期调试打印，避免脱机 stdout 无读取时阻塞。
- 为从车同步 `8 s` 看门狗、主循环各提前返回路径喂狗和每 `10` 帧一次的 `gc.collect()`，降低永久卡死和长期堆碎片风险。
- 同步主车当前多条蓝地采样动态裁切、底角黄线过线确认和已淘汰橘色障碍逻辑删除结果。
- 保留从车独立的角色选择、曝光、颜色阈值、蓝地阈值、黄线阈值、回库信标阈值及 IPM 标定参数。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- 常量差异检查确认主从文件仅保留上述设备参数差异。
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` 已通过。

### 2026-07-10 - v0.9.1-dev - 帧率与搬运过线优化

范围：`main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 删除两份正式程序中的橘色 LAB 阈值、通道位置判断、橘色障碍检测函数和目标框重叠排除逻辑。
- 搜索、搬运和回库路径不再逐帧扫描 `320×160` 的橘色障碍 ROI；检测到的正常颜色目标也不再因与橘色区域重叠而被丢弃。
- 保留 16 字节回传协议中的 `obstacle_flag` 字段以兼容现有 RT1021 解包，字段固定回传 `0`。
- `0x06` 搬运前其它颜色 ID 扫描保持不变，它与已删除的橘色障碍检测相互独立。
- 深蓝场地动态上界识别完成后，将真正的目标检测 ROI 上界设为该边界向下 `3 px`；全局搜索、局部跟踪和 `0x06` 扫描统一使用收紧后的 ROI。
- 动态裁切调试线改为绘制实际生效的水平 ROI 上界，便于现场确认裁切范围。
- 主车黄线到达左下角或右下角前仍每 2 帧检测一次；到达底角并锁存后切换为逐帧检测，连续 3 个实际帧未检测到黄线即判定过线。

效果：
- 普通目标检测和回库模式每帧减少一次大区域 `find_blobs()`，目标搜索 ROI 也进一步缩小，降低视觉处理开销并改善实际帧率。
- 主车到达黄线底角后的过线响应由约 6 个主循环帧缩短为 3 个主循环帧，同时仍保留底角到达前置条件。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- `git diff --check` 已通过。

### 2026-07-10 - v0.9.0 - 双车互通稳定版

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- `main.py` 新增 `host_color_id_received` 状态，与 `minimain.py` 保持一致，用于区分主控下发的最终颜色 ID 和本地搜索到的候选颜色。
- `main.py` 和 `minimain.py` 不再在本地稳定识别后自行写入 `target_color_id`；未收到主控 `0x03` 时持续全颜色搜索，并照常向主控回传候选颜色 ID 和坐标。
- 收到主控 `0x03 SET_TARGET_COLOR` 后，两份脚本立即切换到对应单一 LAB 阈值搜索，并清空当前跟踪框，下一帧按主控颜色重新捕获目标。
- 主控锁色后如果短暂丢失目标，只清空本地跟踪框和 ROI 状态，不清除主控颜色 ID，避免退回全颜色搜索；收到重置/回库等命令时仍清除锁色状态。
- 更新根目录 `README.md` 简单入口，标明稳定版本、主从车正式脚本和中英文详细日志位置。

效果：
- 两辆车的 OpenART 地位相等：谁先发现候选颜色就先上报，由 RT1021 主控链路仲裁并同步最终颜色，再分别通过 `0x03` 下发给两侧 OpenART，实现两车跟踪同一颜色目标。
- 本版本将此前的颜色检测、SD 阈值、搬运前扫描、黄线过线、障碍状态、回库信标与本次双车锁色闭环整理为可部署的稳定版本。

验证：
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py calib_ide_tune.py front_obstacle_scan_test.py cmm_load.py` 已通过。
- `git diff --check` 已通过。

### 2026-07-09 - v0.8.0-dev - 搬运前其它色块 ID 扫描

范围：`main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `calib_ide_tune.py`, `front_obstacle_scan_test.py`, `.gitignore`, `README_ch.md`, `README_en.md`

变更：
- `main.py` 和 `minimain.py` 新增主控命令 `0x06`，用于在进入搬运前请求 OpenART 对当前画面做全颜色阈值扫描。
- 扫描使用全部 5 个 LAB 阈值，不受当前锁色目标限制，并复用现有色块形状过滤、动态裁切线过滤和 `pixels > 400` 面积过滤。
- 扫描结果会排除当前已对准/跟踪的目标框；如果画面里还有其它同色目标，仍会把该颜色 ID 计入结果。
- 新增 `0xC7` 回传帧：`AA 55 C7 current_id mask count checksum`，其中 `mask` 的 bit0-bit4 对应颜色 1-5，`count` 为检测到的其它颜色 ID 种类数；结果需连续 10 帧稳定后才发送。
- 正常寻找、锁定、搬运、回库和黄线逻辑不变；只有收到 `0x06` 后才启动搬运前扫描流程。
- 新增现场自动标定、IDE 调参和前方色块扫描预览脚本，用于生成 `/sd/color_thr.txt`、复核阈值和离线观察 `0x06` 扫描候选。
- 清理旧的独立模型、三分类、回库信标和黄线 IPM 测试脚本；`.gitignore` 新增 `*.tflite`，避免模型文件误提交。

效果：
- 主控可在发送 `0x01` 进入搬运前先发送 `0x06`，根据其它色块 ID 自行判断前方是否存在需要处理的障碍或目标。

验证：
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py calib_ide_tune.py front_obstacle_scan_test.py` 已通过。
- `git diff --check` 已通过。

### 2026-07-09 - v0.7.9-dev - SD 参数读取与从车角度矫正移除

范围：`main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

变更：
- `main.py` 和 `minimain.py` 启动时优先读取 `/sd/color_thr.txt`，支持自动标定脚本输出的 `exposure_us=`、`ground=` / `ground2=` 和 `slot,L0,L1,A0,A1,B0,B1` 格式。
- 只有读满 5 个颜色槽位时才覆盖内置阈值；文件缺失、格式不完整或解析失败时继续使用写死默认阈值。
- 读取成功或回退默认阈值时打印 `[color_thr]` 启动信息，便于现场判断当前阈值来源。
- `minimain.py` 移除 `yellow_crossline_ipm.py` 导入、黄线角度矫正实例和每帧角度处理；保留 16 字节回传协议中的角度字段，但从车固定发送 0。
- `minimain.py` 将 `0x04` 命令保留为预留忽略，避免主控误发时破坏命令解析。

效果：
- 比赛前可用 IDE 标定脚本生成 `/sd/color_thr.txt`，正式运行脚本上电后自动使用该阈值文件。
- 从车运行入口减少一个调试模块依赖，部署更轻量。

验证：
- `python -m py_compile main.py minimain.py` 已通过。
- `git diff --check -- main.py minimain.py` 已通过。

### 2026-07-09 - v0.7.8-dev - 同色目标按最左优先获取

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 同步更新两个正式运行入口的初始颜色目标选择逻辑。
- 同一颜色 ID 下存在多个合格色块时，先选择最左侧色块作为该颜色的代表目标。
- 不同颜色之间仍然只比较每种颜色的一个代表目标，并保留原来的色块框底部距离 `y=240` 最近优先规则。
- 重写根目录 `README.md`，整理为项目入口、运行脚本、测试工具和部署注意事项。

效果：
- 同色重复目标会按从左到右的顺序优先获取，同时不改变原有跨颜色目标比较方式。

验证：
- `python -m py_compile main.py minimain.py yellow_crossline_ipm.py` 已通过。
- `git diff --check` 已通过。

### 2026-07-05 - v0.7.7-dev - Plus 从车 UART12 与两阶段黄线过线

范围：`main.py`, `minimain.py`, `yellow_crossline_ipm.py`, `README_ch.md`, `README_en.md`, `README.md`

变更：
- `minimain.py` 按 OpenART Plus 硬件更新：保留从车软件角色和颜色 ID 接收逻辑，但串口固定初始化为 `UART12`。
- `main.py` 黄线检测改为扫描 `y=100`、`y=140` 附近两块横向 ROI，取黄色 blob 中心点拟合贯穿屏幕的黄线。
- 搬运模式黄线过线改为两阶段：先等待拟合黄线到达左下角或右下角，只作为过线判定解锁；之后黄线连续消失达到阈值时才发送搬运完成/`POS_CROSSED`。
- `yellow_crossline_ipm.py` 默认串口、标定点和镜像采集路径同步到 OpenART Plus。

验证：
- `python -m py_compile .\main.py .\minimain.py .\yellow_crossline_ipm.py` 已通过。

### 2026-07-05 - v0.7.6-dev - 全程色块检测运行路径
范围：`main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

变更：
- 删除 `main.py` 中 Plus 运行时白熊相关的模型配置、加载路径、推理辅助函数和主循环模型分支。
- 5 号白熊目标改为与其它目标一致，走 LAB `find_blobs()` 色块检测和跟踪路径。
- 确认 `minimain.py` 没有目标推理路径，并标注运行时目标检测只使用 LAB 色块。

效果：
- Plus / Mini 正式运行入口不再导入 `tf`、加载 `.tflite`、创建推理用临时缩放图像，也不在帧循环中调用推理。
- 降低 RAM 压力和帧循环阻塞风险；所有目标统一使用色块检测。

验证：
- `rg -n "tf|tflite|model|MODEL|USE_WHITE|WHITE_BEAR|find_white|load_white|model_net|model_tf|\.detect\(" main.py minimain.py` 无匹配。
- `python -m py_compile main.py minimain.py` 已通过。
- `git diff --check -- main.py minimain.py` 已通过。

### 2026-06-21 - v0.7.5-dev - 移除 SD 日志，消除脱机死机隐患

范围：`main.py`, `README_ch.md`, `README_en.md`

变更：
- 删除 `ENABLE_SD_LOG`、`LOG_PATH`、`LOG_INTERVAL_MS`、`LOG_FIRST_FRAMES` 常量及 `last_log_ms` 变量。
- 删除 `log_checkpoint()` 函数。
- 删除启动时、`load_white_bear_model()` 内、主循环内全部 `log_checkpoint` 调用（约 25 处）。
- 保留看门狗逻辑（`ENABLE_WATCHDOG`、`WATCHDOG_TIMEOUT_MS`、`init_watchdog()`、`feed_watchdog()`）不变。

效果：
- 消除前 10 帧强制写 SD（约 15 次/帧）导致单帧耗时超 8 s 触发 WDT 复位的风险。
- 消除正常运行时每秒一次 SD 写入可能卡住帧循环的隐患。
- 主循环无 SD I/O，帧时间更稳定。

维护约束：
- 脱机部署不要重新开启 SD 日志；如需现场诊断，上板连 IDE 用 `print()` 或单独的诊断脚本，不要在主循环里加文件写入。
- 若将来需要持久化日志，必须在 `log_checkpoint` 内加 `feed_watchdog()`，并把强制写入帧数（`LOG_FIRST_FRAMES`）降到 3 帧以内。

验证：
- `python -m py_compile main.py` 已通过。

### 2026-06-20 - v0.7.4-dev - OpenART TypeError 兼容性记录

范围：`main.py`, `README_ch.md`, `README_en.md`

变更：
- 记录 OpenART/MicroPython 上两类现场 `TypeError` 的原因和处理方法。
- `TypeError: function takes 0 positional arguments but 1 were given`：现场固件里部分内建名或被覆盖的函数不一定等同于桌面 Python。不要在运行主流程里为了规范化真假值额外写 `bool(x)`；直接使用列表/blob 对象本身做条件判断，例如 `raw_yellow_seen = yellow_blob` 或 `raw_yellow_seen = yellow_blobs_left and yellow_blobs_right`。
- `TypeError: function takes 2 positional arguments but 1 were given`：在 OpenART/MicroPython 上新增一层辅助函数调用时，报错位置可能不够直观，尤其是状态机切换、全局变量和命令处理混在一起时。处理方法是先减少调用链，把关键状态赋值放回命令处理原位置，再逐步验证。
- 桌面端 `python -m py_compile` 只能证明语法合法，不能证明 OpenART 固件运行时 API 和内建函数行为一致；涉及 `bool()`、`max(..., key=...)`、新增函数封装、摄像头/图像 API 时必须上板验证。

维护约束：
- 主循环和命令处理里的条件判断优先使用 MicroPython 兼容写法，避免不必要的类型转换包装。
- 现场报 `TypeError` 时先检查最近新增的函数调用、内建名调用和带 `key=` 的高阶调用，再看算法逻辑。
- 修复这类问题时优先做小改动，保留原状态机路径，避免把运行时兼容问题和业务逻辑调整混在一起。

验证：
- `python -m py_compile .\main.py` 已通过。

### 2026-06-18 - v0.7.3-dev - Plus 脱机死机诊断

范围：`main.py`, `yellow_crossline_ipm.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- `main.py` 新增轻量看门狗和 SD 检查点日志，诊断信息追加写入 `/sd/watchdog.log`。
- 移除 Plus 主循环运行时 `print()`，避免脱机运行时 USB/stdout 无读取导致阻塞。
- 新增 `RUNTIME_LENS_CORR = False`，关闭 `main.py` 正常运行路径每帧 `lens_corr(2)`，用于隔离帧缓冲和堆内存压力。
- 关闭 `yellow_crossline_ipm.py` 独立运行循环中的 `lens_corr(2)`，使其图像路径与未畸变校正的标定视图一致。
- 删除临时 Plus 翻转测试脚本。

效果：
- 当前是测试/诊断状态，不作为最终稳定性结论。
- 脱机死机或看门狗复位后，可以读取日志最后检查点定位卡住阶段。
- 运行时图像坐标现在与当前标定模式一致，标定模式不执行镜头畸变校正。

验证：
- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec')"` 已通过。
- `python -c "import pathlib; compile(pathlib.Path('yellow_crossline_ipm.py').read_text(encoding='utf-8'), 'yellow_crossline_ipm.py', 'exec')"` 已通过。

### 2026-06-16 - v0.7.2 - 22cm 标定说明

范围：`minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：
- 修正 `main.py` 的逆透视标定注释：Plus 相机当前同样是离地 `22cm`，不是 `12cm`。
- 更新 `minimain.py` 的逆透视标定注释，标明当前 Mini 相机为离地 `22cm` 的现场标定参数。
- 补充说明如果再次调整任一相机高度或俯仰角，需要重新运行标定模式更新 `CALIB_PIXEL`。

效果：
- Plus 与 Mini 的标定说明现在都记录为当前离地 `22cm` 情况。

验证：
- `python -c "import pathlib; compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` 已通过。

### 2026-06-15 - v0.7.1 - 22cm 标定与死机修复

范围：`main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- Plus 正式入口 `main.py` 更新离地 `22cm` 场景下的 IPM 标定点，`CALIB_PIXEL` 调整为现场采集像素点，`CALIB_WORLD` 调整为对应世界坐标。
- `main.py` 新增统一的 `snapshot_frame()` 入口，把拍照、镜头畸变校正和软件翻转集中到同一处，避免主循环、亮度标定和 IPM 标定模式各自直接调用 `sensor.snapshot()`。
- `main.py` 使用软件方式一次完成 `hmirror` / `vflip` 后处理，规避 OpenART 固件同时维护多个硬件翻转状态时可能出现的画面异常和死机风险。
- `main.py` 修复主车串口选择，主车继续使用 `UART(12)`，从车使用 `UART(2)`。
- `main.py` 恢复白熊 TFLite 模型检测路径，并改用当前固件可用的 `model_net.detect()` 结果接口，同时在临时缩放图像释放后执行 `gc.collect()`，降低模型检测后的内存压力。
- `main.py` 支持从 `/sd/params.txt` 读取 5 组 LAB 阈值；读取失败或格式不完整时继续使用内置默认阈值。
- `minimain.py` 同步统一 `snapshot_frame()` 入口，并只保留一个硬件翻转方向，减少 Mini / 从车长时间运行时的帧缓冲压力。

效果：

- Plus 逆透视坐标适配当前离地 `22cm` 的相机安装高度。
- 主车运行时的拍照和翻转路径更集中，降低因硬件翻转状态、模型检测临时图像和内存回收不一致导致的卡死概率。
- Mini / 从车的图像采集路径与 Plus 保持同一维护方式，但默认不启用额外软件翻转，优先保证长时间运行稳定性。

验证：

- `git diff --check` 已通过。
- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec'); compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` 已通过。

### 2026-06-15 - v0.7.0 - 校赛完赛版本

范围：`main.py`, `minimain.py`, `return_beacon_ipm_test.py`

变更：

- 将当前 OpenART 视觉程序保存为校赛完赛版本，提交号为 `d7f56c3`。
- Plus 正式入口 `main.py` 恢复颜色搜索顺序为 `COLOR_SEARCH_ORDER = [1, 2, 3, 4, 5]`，不再单独把网球提前为最高优先级。
- Plus 黄线检测改为每 `2` 帧检测一次，搬运模式黄线丢失确认阈值调整为 `3` 次检测，兼顾响应速度和抗抖。
- Plus 回库信标 LAB 阈值、最小像素/面积、长宽比和密度过滤同步放宽，提升现场回库信标的检出率。
- Mini / 从车入口 `minimain.py` 增加 `yellow_raw_detected`，搬运完成判定只由真实检测周期触发，避免滞回保持状态误判。
- Mini / 从车入口增加 `YELLOW_CARRY_HOLD_FRAMES = 40`，进入搬运模式并真实看到黄线后保持一段安全窗口，再开始累计黄线丢失。
- 进入搬运模式时先清空黄线状态，再切换 `MODE_CARRY`，避免沿用上一轮任务的黄线锁存。
- `return_beacon_ipm_test.py` 同步 Plus 回库信标阈值和过滤参数，便于现场单独验证回库信标识别。

效果：

- 当前版本反映校赛实际完赛参数，优先保证稳定完赛和现场检出。
- 回库信标过滤更宽，测试脚本与正式 Plus 主程序保持一致。
- Mini / 从车搬运黄线完成判定更依赖真实新检测，减少进入搬运瞬间或上一轮状态残留造成的误触发。

验证：

- `git diff --check` 已通过。

### 2026-06-13 - v0.6.0 - 调整目标锁定优先级

范围：`main.py`, `minimain.py`

变更：

- Plus 正式入口 `main.py` 和 Mini / 从车入口 `minimain.py` 同步目标锁定策略。
- 新增 `COLOR_SEARCH_ORDER = [3, 1, 2, 4, 5]`，保持颜色 ID 不变，并让网球 `Color 3` 最先搜索。
- 网球一旦被检测到，优先锁定网球；如果同画面有多个网球，选择色块框底部距离 `y=240` 最近的那个。
- 如果没有检测到网球，再搜索其它颜色；其它颜色之间不按颜色顺序提前返回，而是比较色块框底部到 `y=240` 的距离，优先锁定更靠近图像底部的物体。
- 网球单独降低 `find_blobs()` 门槛：`TENNIS_MIN_PIXELS = 45`，`TENNIS_MIN_AREA = 45`。
- 其它物体继续使用通用门槛：`COLOR_MIN_PIXELS = 100`，`COLOR_MIN_AREA = 100`。
- 已有 `last_box` 跟踪时继续使用原跟踪评分，避免锁定后频繁跳目标。

效果：

- 网球绝对优先，解决并排物体中网球没有优先被搬运的问题。
- 非网球目标按色块框底部到 `y=240` 的距离判断远近，优先搬运更靠近车的物体。

验证：

- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec'); compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` 已通过。

### 2026-06-12 - v0.5.0 - 放宽搬运黄线完成判定

范围：`main.py`, `minimain.py`

变更：

- Plus 正式入口 `main.py` 放宽搬运模式下的黄线过线完成判定。
- Mini / 从车入口 `minimain.py` 同步相同的黄线判定策略，避免两套程序行为不一致。
- 将 `YELLOW_DETECT_INTERVAL` 从 `5` 帧改为 `3` 帧，提高搬运阶段黄线状态刷新速度。
- 将 `YELLOW_LOST_THRESHOLD` 从 `5` 次检测改为 `2` 次检测，缩短黄线通过后发送 `POS_CROSSED` 的确认窗口。
- 保留“进入搬运模式后必须先看到黄线，再丢失黄线才判定过线”的防误判机制，避免掉头或未到线时误发搬运完成。
- 搬运模式黄线扫描继续使用从图像底部向上分条扫描的方式，优先捕捉靠近车身底部的黄线。

效果：

- 原逻辑大约需要 `5 * 5 = 25` 帧确认黄线丢失后才发送搬运完成。
- 新逻辑大约需要 `3 * 2 = 6` 帧确认，响应更快，但仍有连续丢失确认。

验证：

- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec')"` 已通过。
- `python -c "import pathlib; compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` 已通过。

### 2026-06-10 - v0.4.0 - 回退单文件运行结构并保留黄线角度修复

范围：`main.py`, `minimain.py`, `yellow_crossline_ipm.py`, `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, `openart_calibration.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 将 Plus 正式运行入口回退为单文件 `main.py`，来源为 Git 初始单文件版本。
- 将 Mini / 从车运行入口恢复为单文件 `minimain.py`，来源为 Mini 黄线同步后的历史版本。
- 删除多文件运行模块：`openart_app.py`、`openart_config.py`、`openart_detectors.py`、`openart_trackers.py`、`openart_uart.py`、`openart_math.py`、`openart_camera.py`、`openart_calibration.py`。
- 保留 `yellow_crossline_ipm.py`，因为单文件 `main.py` / `minimain.py` 仍会导入它用于黄线横线角度矫正。
- 黄线角度采样改为每个竖向采样条取最靠近图像底部的黄色 blob 下边缘，减少黄线有宽度时中心点跳变造成的角度抖动。

卡死排查结论：

- 单独模型测试脚本可以运行，更换 SD 卡后多文件版本仍会卡死，因此问题主要不是 SD 卡或模型路径。
- 多文件结构在 OpenART/MicroPython 上导入模块更多，增加 RAM 占用和内存碎片；TFLite 推理需要连续内存，内存不足或碎片化时可能表现为卡死而不是正常异常。
- `lens_corr()`、`img.copy()`、调试绘图、黄线检测、障碍检测和模型推理叠加会加重问题，但最终有效修复是回退到单文件运行结构。

维护约束：

- 比赛/脱机部署不要再恢复多文件运行结构。
- 如果以后必须模块化，必须先在板子上实测 `tf.load()` 和 `tf.detect()` 前后的剩余内存，并完整跑通模型检测主循环。
- 后续模块化应优先考虑 `.mpy` 预编译、延迟导入、减少调试代码和显式内存日志。

验证：

- `python -m py_compile main.py minimain.py yellow_crossline_ipm.py test_model.py return_beacon_ipm_test.py` 已通过。

### 2026-06-08 - v0.3.0 - Plus/Mini 共享模块化重构

范围：`main.py`, `minimain.py`, `openart_config.py`, `openart_app.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_calibration.py`, `openart_camera.py`, `openart_uart.py`, `openart_math.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 将 `main.py` 和 `minimain.py` 改为薄入口文件，分别选择 `PLUS_CONFIG` 和 `MINI_CONFIG`。
- 新增 `openart_config.py`，集中管理 Plus/Mini 差异配置。
- 新增 `openart_app.py`，承载共享视觉初始化、检测流水线、命令处理、标定模式和主循环。
- 新增 `openart_detectors.py`，封装动态裁剪、颜色目标、白熊模型、障碍和回库信标检测。
- 新增 `openart_trackers.py`，封装目标、黄线、回库信标运行状态。
- 新增 `openart_calibration.py`，封装逆透视标定模式。
- 新增 `openart_camera.py`，保留启动亮度校准等相机辅助工具。
- 新增 `openart_uart.py`，封装 RT1021 串口协议。
- 新增 `openart_math.py`，封装几何、单应性和坐标转换工具。
- Plus 和 Mini 的白熊检测统一使用 TFLite 模型。
- 删除单独的 `ARCHITECTURE.md`，结构说明统一维护在 README 中。

验证：

- `python3 -m py_compile main.py minimain.py openart_app.py openart_config.py openart_uart.py openart_math.py openart_trackers.py openart_detectors.py openart_calibration.py openart_camera.py` 已通过。
- `git diff --check` 已通过。

### 2026-06-06 - v0.2.0 - Mini 黄线逻辑同步

范围：`minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

变更：

- 将 Mini 的黄线检测逻辑同步到 `main.py` 中已修复的版本。
- 将黄线检测 ROI 从窄侧边条更新为延伸到底部的左右检测区域。
- 将黄线保持阈值从 `7` 像素降低到 `3` 像素。
- 增加最近黄线检测锁存，用于处理进入搬运模式瞬间的遮挡。
- 将黄线检测提前到 `pos_flag` 计算之前，确保回传位置标志使用当前帧状态。
- 将 `minimain.py` 的注释和启动打印重写为可读 UTF-8 中文。
- 将 README 拆分为中文日志 `README_ch.md` 和英文日志 `README_en.md`。

验证：

- `python3 -m py_compile openart/minimain.py` 已通过。

当前状态：

- `main.py`：Plus 黄线逻辑已修复。
- `minimain.py`：Mini 黄线逻辑已同步。

### 2026-06-06 - v0.1.0 - 初始仓库

范围：仓库初始化

变更：

- 创建 OpenART 视觉脚本初始仓库。
- 添加 Plus 主脚本：`main.py`。
- 添加 Mini 主脚本：`minimain.py`。
- 添加测试与工具脚本。
- 添加 `.gitattributes` 和 `.gitignore`，用于跨平台协作。

当前状态：

- `main.py` 已包含修复后的黄线检测逻辑。
- `minimain.py` 尚未同步该黄线更新。

## 维护说明

- 每次修改行为逻辑、阈值、通信协议字段或设备专用脚本时，都新增一条日志。
- 日志按时间倒序排列，最新版本放在最前。
- 修改 Plus/Mini 差异时，优先修改 `openart_config.py`；只有共享行为变化才修改 `openart_app.py` 或共享模块。
- 每次更新 README 时，同时更新 `README_ch.md` 和 `README_en.md`。
