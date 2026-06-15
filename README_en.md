# OpenART Vision Change Log

This repository tracks OpenART smart car vision scripts for OpenART Plus and OpenART Mini.

## Current Files

| File | Device | Role |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus single-file official offline runtime |
| `minimain.py` | OpenART Mini | Mini / slave-car single-file official offline runtime |
| `yellow_crossline_ipm.py` | OpenART Plus / Mini / test | Yellow crossline angle and IPM helper imported by `main.py` / `minimain.py` |
| `openart_test_3class.py` | test | Three-class vision test |
| `return_beacon_ipm_test.py` | test | Return beacon IPM test |
| `test_model.py` | test | Model test script |
| `main.py.bak_20260331` | backup | Local historical single-file backup, not an official entrypoint |

## Structure Notes

- Current competition/offline deployment uses the single-file layout. `main.py` and `minimain.py` each contain the full runtime logic for Plus and Mini / slave-car.
- The multi-file runtime modules have been removed. The deployment no longer uses `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, or `openart_calibration.py`.
- `yellow_crossline_ipm.py` is the only retained runtime helper module. Copy it together with `main.py` / `minimain.py` when deploying.
- White-bear detection, color target detection, yellow-line state, return beacon, UART protocol, IPM, and the main loop live inside the corresponding single-file runtime.
- The multi-file version caused TFLite detection freezes. See the v0.4.0 log for the investigation and maintenance rules; do not restore the v0.3.0 modular structure as the competition deployment layout.
- Keep all structure notes and iteration records in `README_ch.md` / `README_en.md`, and update both languages together.

## Logs

### 2026-06-15 - v0.7.1 - 12 cm Calibration and Freeze Fix

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Updated the Plus runtime `main.py` IPM calibration for the `12 cm` camera-height setup. `CALIB_PIXEL` now uses the field-measured image points, and `CALIB_WORLD` uses the matching world coordinates.
- Added a shared `snapshot_frame()` entry in `main.py` so capture, lens correction, and software flipping are handled in one place instead of separate direct `sensor.snapshot()` calls in the main loop, brightness calibration, and IPM calibration mode.
- Changed `main.py` to apply `hmirror` / `vflip` together in software, reducing the risk of display corruption or freezes when the OpenART firmware keeps multiple hardware flip states.
- Fixed the Plus UART selection in `main.py`: the master car uses `UART(12)`, and the slave car uses `UART(2)`.
- Re-enabled the white-bear TFLite model path in `main.py`, switched to the current firmware's `model_net.detect()` result interface, and runs `gc.collect()` after releasing the temporary scaled image to reduce memory pressure after model detection.
- Added optional loading of 5 LAB threshold rows from `/sd/params.txt` in `main.py`; invalid or incomplete files fall back to the built-in defaults.
- Synced `minimain.py` to the shared `snapshot_frame()` entry and kept only one hardware flip direction by default, reducing frame-buffer pressure during long Mini / slave-car runs.

Effect:

- Plus IPM world coordinates now match the current `12 cm` camera mounting height.
- The Plus image-capture and flip path is more centralized, reducing freeze risk from inconsistent hardware flip state, temporary model images, and delayed memory collection.
- Mini / slave-car capture follows the same maintenance pattern while keeping extra software flip disabled by default for long-run stability.

Verification:

- `git diff --check` passed.
- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec'); compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` passed.

### 2026-06-15 - v0.7.0 - School-Competition Finished Version

Scope: `main.py`, `minimain.py`, `return_beacon_ipm_test.py`

Changed:

- Saved the current OpenART vision program as the school-competition finished version, commit `d7f56c3`.
- Restored the Plus runtime `main.py` color search order to `COLOR_SEARCH_ORDER = [1, 2, 3, 4, 5]`, removing the special tennis-ball-first priority.
- Changed Plus yellow-line detection to run every `2` frames and set the carry-mode yellow-line loss threshold to `3` detection ticks, balancing response time and stability.
- Relaxed the Plus return-beacon LAB threshold, minimum pixels/area, aspect-ratio range, and density filter to improve field detection.
- Added `yellow_raw_detected` to the Mini / slave-car runtime `minimain.py` so carry completion is driven by a real detection cycle instead of only the hysteresis state.
- Added `YELLOW_CARRY_HOLD_FRAMES = 40` to Mini / slave-car carry mode. After the yellow line is truly seen, the runtime keeps a short safety window before counting yellow-line loss.
- Reset yellow-line state before switching into `MODE_CARRY`, avoiding stale latch state from the previous task.
- Synced `return_beacon_ipm_test.py` with the Plus return-beacon threshold and filtering parameters so the beacon can be verified independently on field.

Effect:

- This version records the actual school-competition finish parameters, prioritizing stable completion and field detection.
- Return-beacon filtering is wider, and the test script matches the official Plus runtime.
- Mini / slave-car carry completion depends more directly on fresh yellow-line detection, reducing false triggers from mode-entry timing or stale state.

Verification:

- `git diff --check` passed.

### 2026-06-13 - v0.6.0 - Tune Target Lock Priority

Scope: `main.py`, `minimain.py`

Changed:

- Synced the target-locking policy in the Plus runtime `main.py` and the Mini / slave-car runtime `minimain.py`.
- Added `COLOR_SEARCH_ORDER = [3, 1, 2, 4, 5]` so the tennis ball `Color 3` is searched first without changing transmitted color IDs.
- If a tennis ball is detected, it is locked first. If multiple tennis balls are visible, the chosen one is the blob whose bounding-box bottom is closest to `y=240`.
- If no tennis ball is detected, the other colors are searched and then ranked by bounding-box bottom distance to `y=240`, so the nearest lower-image object is locked first.
- Lowered the tennis-ball-only `find_blobs()` thresholds: `TENNIS_MIN_PIXELS = 45`, `TENNIS_MIN_AREA = 45`.
- Kept the general thresholds for other objects: `COLOR_MIN_PIXELS = 100`, `COLOR_MIN_AREA = 100`.
- Kept the existing `last_box` tracking score after a target is locked, reducing target jumps during tracking.

Effect:

- Tennis balls now have absolute priority when visible.
- Non-tennis targets are selected by distance from the blob bottom to `y=240`, favoring the object closest to the car.

Verification:

- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec'); compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` passed.

### 2026-06-12 - v0.5.0 - Relax Carry-Mode Yellow-Line Completion

Scope: `main.py`, `minimain.py`

Changed:

- Relaxed the carry-mode yellow-line crossing completion check in the Plus runtime `main.py`.
- Synced the same yellow-line decision policy to the Mini / slave-car runtime `minimain.py` so both entrypoints behave consistently.
- Changed `YELLOW_DETECT_INTERVAL` from `5` frames to `3` frames to refresh yellow-line state faster during carry mode.
- Changed `YELLOW_LOST_THRESHOLD` from `5` detection ticks to `2` detection ticks, reducing the confirmation window before sending `POS_CROSSED`.
- Kept the false-positive guard that requires the car to see the yellow line in carry mode before a later yellow-line loss can count as crossing.
- Kept the carry-mode bottom-up strip scan so detection still prioritizes the yellow line closest to the bottom of the image.

Effect:

- The old logic needed about `5 * 5 = 25` frames after yellow-line loss before reporting carry completion.
- The new logic needs about `3 * 2 = 6` frames, giving a faster response while still requiring consecutive loss confirmation.

Verification:

- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec')"` passed.
- `python -c "import pathlib; compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` passed.

### 2026-06-10 - v0.4.0 - Revert to Single-File Runtime and Keep Yellow Angle Fix

Scope: `main.py`, `minimain.py`, `yellow_crossline_ipm.py`, `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, `openart_calibration.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Reverted the Plus runtime to the single-file `main.py` from the initial Git single-file version.
- Restored the Mini / slave-car runtime as the single-file `minimain.py` from the historical Mini yellow-line sync version.
- Removed the multi-file runtime modules: `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, and `openart_calibration.py`.
- Kept `yellow_crossline_ipm.py` because single-file `main.py` / `minimain.py` still import it for yellow crossline angle correction.
- Updated yellow angle sampling to use the bottom edge of the nearest yellow blob in each vertical scan strip, reducing angle jitter when the yellow line has visible width.

Freeze investigation conclusion:

- The isolated model test could run, and replacing the SD card did not fix the multi-file freeze. The issue was therefore not primarily caused by the SD card or model path.
- The multi-file runtime imported more modules on OpenART/MicroPython, increasing RAM usage and fragmentation. TFLite inference needs contiguous memory, and memory pressure can appear as a freeze instead of a clean exception.
- `lens_corr()`, `img.copy()`, debug drawing, yellow-line detection, obstacle detection, and model inference in the same loop made the issue worse, but the effective fix was reverting to the single-file runtime.

Maintenance rules:

- Do not restore the multi-file runtime layout for competition/offline deployment.
- If modularization is required again, first measure free memory around `tf.load()` and `tf.detect()` on the board and verify the full model-detection main loop.
- Future modular attempts should prioritize `.mpy` precompiled modules, delayed imports, reduced debug code, and explicit memory logs.

Verification:

- `python -m py_compile main.py minimain.py yellow_crossline_ipm.py test_model.py return_beacon_ipm_test.py` passed.

### 2026-06-08 - v0.3.0 - Shared Plus/Mini Modular Refactor

Scope: `main.py`, `minimain.py`, `openart_config.py`, `openart_app.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_calibration.py`, `openart_camera.py`, `openart_uart.py`, `openart_math.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Converted `main.py` and `minimain.py` into thin entrypoints that select `PLUS_CONFIG` and `MINI_CONFIG`.
- Added `openart_config.py` to centralize Plus/Mini configuration differences.
- Added `openart_app.py` for shared vision initialization, detection pipeline, command handling, calibration mode, and main loop.
- Added `openart_detectors.py` for dynamic cut, color target, white-bear model, obstacle, and return-beacon detection.
- Added `openart_trackers.py` for target, yellow-line, and return-beacon runtime state.
- Added `openart_calibration.py` for inverse-perspective calibration mode.
- Added `openart_camera.py` for camera helper utilities such as startup brightness calibration.
- Added `openart_uart.py` for RT1021 UART protocol handling.
- Added `openart_math.py` for geometry, homography, and coordinate transform helpers.
- Unified white bear detection so both Plus and Mini use the TFLite model.
- Removed the standalone `ARCHITECTURE.md`; structure notes now live in README.

Verification:

- `python3 -m py_compile main.py minimain.py openart_app.py openart_config.py openart_uart.py openart_math.py openart_trackers.py openart_detectors.py openart_calibration.py openart_camera.py` passed.
- `git diff --check` passed.

### 2026-06-06 - v0.2.0 - Mini Yellow Line Sync

Scope: `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Synced Mini yellow line detection with the fixed logic in `main.py`.
- Updated yellow detection ROIs from narrow side strips to bottom-extended side strips.
- Reduced yellow hold threshold from `7` pixels to `3` pixels.
- Added recent yellow-line detection latch for carry-mode transition.
- Moved yellow detection before `pos_flag` calculation so transmitted position flags use the current frame state.
- Rewrote `minimain.py` comments and startup prints as readable UTF-8 Chinese.
- Split README content into `README_ch.md` for Chinese logs and `README_en.md` for English logs.

Verification:

- `python3 -m py_compile openart/minimain.py` passed.

Current state:

- `main.py`: Plus yellow line logic is fixed.
- `minimain.py`: Mini yellow line logic is synchronized.

### 2026-06-06 - v0.1.0 - Initial Repository

Scope: repository setup

Changed:

- Created the initial OpenART vision script repository.
- Added Plus main script: `main.py`.
- Added Mini main script: `minimain.py`.
- Added test and utility scripts.
- Added `.gitattributes` and `.gitignore` for cross-platform collaboration.

Current state:

- `main.py` had the fixed yellow line detection logic.
- `minimain.py` had not yet synchronized that yellow line update.

## Maintenance Notes

- Add a new log entry whenever behavior, thresholds, protocol fields, or device-specific scripts change.
- Keep entries ordered newest first.
- For Plus/Mini differences, update `openart_config.py` first; change `openart_app.py` or shared modules only when shared behavior changes.
- Whenever README is updated, update both `README_ch.md` and `README_en.md`.
