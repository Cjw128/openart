# OpenART 视觉更新日志

本仓库用于记录 OpenART 智能车视觉脚本的迭代，包括 OpenART Plus 与 OpenART Mini。

## 当前文件

| 文件 | 设备 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus 入口，选择 Plus 配置并启动共享运行逻辑 |
| `minimain.py` | OpenART Mini | Mini 入口，选择 Mini 配置并启动共享运行逻辑 |
| `openart_config.py` | OpenART Plus / Mini | Plus/Mini 配置，包括串口角色、阈值、模型路径和标定点 |
| `openart_app.py` | OpenART Plus / Mini | 共享视觉运行逻辑，包括初始化、检测流水线、主机命令、标定模式和主循环 |
| `openart_detectors.py` | OpenART Plus / Mini | 动态裁剪、颜色目标、白熊模型、障碍和回库信标检测 |
| `openart_trackers.py` | OpenART Plus / Mini | 目标、黄线过线、回库信标的运行状态类 |
| `openart_calibration.py` | OpenART Plus / Mini | 逆透视标定模式 |
| `openart_camera.py` | OpenART Plus / Mini | 启动亮度校准等相机辅助工具 |
| `openart_uart.py` | OpenART Plus / Mini | RT1021 串口协议封包与发送 |
| `openart_math.py` | OpenART Plus / Mini | 几何、单应性、IoU 和坐标转换工具 |
| `yellow_crossline_ipm.py` | OpenART Plus / 测试 | 黄线横线与逆透视工具 |
| `openart_test_3class.py` | 测试 | 三分类视觉测试 |
| `return_beacon_ipm_test.py` | 测试 | 回库信标逆透视测试 |
| `test_model.py` | 测试 | 模型测试脚本 |

## 结构说明

- `main.py` 和 `minimain.py` 只作为入口文件使用，不再维护两套重复主逻辑。
- Plus/Mini 的设备差异集中放在 `openart_config.py`；除非硬件行为确实不同，否则不要在入口文件中添加分支逻辑。
- `openart_app.py` 是共享运行模块，导入后会启动 OpenMV/OpenART 主循环；检测、状态、串口、数学和标定细节拆在独立模块中。
- 白熊检测在 Plus 和 Mini 上都使用 TFLite 模型。配置中保留 5 号颜色阈值用于协议和编号一致性；模型启用时运行时会跳过白熊 LAB 阈值匹配。
- 以后所有结构说明和迭代记录都写入 `README_ch.md` / `README_en.md`，并保持中英文同步更新。

## 更新日志

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
