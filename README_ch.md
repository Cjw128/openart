# OpenART 视觉更新日志

本仓库用于记录 OpenART 智能车视觉脚本的迭代，包括 OpenART Plus 与 OpenART Mini。

## 当前文件

| 文件 | 设备 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus 单文件正式脱机主程序 |
| `minimain.py` | OpenART Mini | Mini / 从车单文件正式脱机主程序 |
| `yellow_crossline_ipm.py` | OpenART Plus / Mini / 测试 | 黄线横线角度与逆透视工具，被 `main.py` / `minimain.py` 导入 |
| `openart_test_3class.py` | 测试 | 三分类视觉测试 |
| `return_beacon_ipm_test.py` | 测试 | 回库信标逆透视测试 |
| `test_model.py` | 测试 | 模型测试脚本 |
| `main.py.bak_20260331` | 备份 | 本地历史单文件备份，不作为正式入口 |

## 结构说明

- 当前比赛/脱机部署使用单文件结构，`main.py` 和 `minimain.py` 分别维护 Plus 与 Mini / 从车的完整主逻辑。
- 多文件运行模块已移除，不再使用 `openart_app.py`、`openart_config.py`、`openart_detectors.py`、`openart_trackers.py`、`openart_uart.py`、`openart_math.py`、`openart_camera.py`、`openart_calibration.py`。
- `yellow_crossline_ipm.py` 是当前唯一保留的运行辅助模块；部署正式程序时需要和 `main.py` / `minimain.py` 一起复制。
- 白熊检测、颜色目标检测、黄线状态、回库信标、UART 协议、IPM 和主循环都在对应的单文件主程序内。
- 多文件版本曾导致 TFLite 检测卡死，原因和维护约束见 v0.4.0 日志；不要把 v0.3.0 的模块化结构重新作为比赛部署结构。
- 以后所有结构说明和迭代记录都写入 `README_ch.md` / `README_en.md`，并保持中英文同步更新。

## 更新日志

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
