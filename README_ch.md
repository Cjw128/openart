# OpenART 视觉更新日志

本仓库用于记录 OpenART 智能车视觉脚本的迭代，包括 OpenART Plus 与 OpenART Mini。

## 当前文件

| 文件 | 设备 | 用途 |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus 主视觉脚本 |
| `minimain.py` | OpenART Mini | Mini 主视觉脚本 |
| `yellow_crossline_ipm.py` | OpenART Plus / 测试 | 黄线横线与逆透视工具 |
| `openart_test_3class.py` | 测试 | 三分类视觉测试 |
| `return_beacon_ipm_test.py` | 测试 | 回库信标逆透视测试 |
| `test_model.py` | 测试 | 模型测试脚本 |

## 更新日志

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
- 修改 Mini 行为前，先对比 `minimain.py` 与 `main.py` 中对应的已修复逻辑。
