# OpenART Vision Change Log

This repository tracks OpenART smart car vision scripts for OpenART Plus and OpenART Mini.

## Current Files

| File | Device | Role |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus single-file official offline runtime |
| `minimain.py` | OpenART Plus / slave | Slave-role single-file official offline runtime |
| `main_autocalib_test.py` | OpenART Plus / test | Field auto-calibration test runtime with host commands |
| `calib_ide_autocalib_competition.py` | OpenART Plus / IDE | Competition field auto-calibration and preview script |
| `calib_ide_tune.py` | OpenART Plus / IDE | Auto-calibration parameter tuning script |
| `front_obstacle_scan_test.py` | OpenART Plus / IDE | Pre-carry front color-blob scan preview script |

## Structure Notes

- Current competition/offline deployment uses the single-file layout. `main.py` and `minimain.py` keep the complete master-role and slave-role logic.
- The multi-file runtime modules have been removed. The deployment no longer uses `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, or `openart_calibration.py`.
- White-bear target handling now uses LAB color blobs like the other targets. Color target detection, yellow-line state, return beacon, UART protocol, IPM, and the main loop live inside the corresponding single-file runtime.
- Calibration, threshold tuning, and pre-carry color-scan checks remain standalone IDE / test scripts, not official offline entrypoints.
- The multi-file version caused TFLite detection freezes. See the v0.4.0 log for the investigation and maintenance rules; do not restore the v0.3.0 modular structure as the competition deployment layout.
- Keep all structure notes and iteration records in `README_ch.md` / `README_en.md`, and update both languages together.

## Logs

### 2026-07-09 - v0.8.0-dev - Pre-carry other-color ID scan

Scope: `main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `calib_ide_tune.py`, `front_obstacle_scan_test.py`, `.gitignore`, `README_ch.md`, `README_en.md`

Changed:

- Added host command `0x06` to both `main.py` and `minimain.py` so the host can request an all-color threshold scan before entering carry mode.
- The scan uses all 5 LAB thresholds, ignores the current target-color lock, and reuses the existing color-blob shape filters, dynamic cut-line filter, and `pixels > 400` area filter.
- The result excludes the currently aligned/tracked target box. If another target with the same color is also visible, that color ID is still included in the result.
- Added return packet `0xC7`: `AA 55 C7 current_id mask count checksum`. `mask` bit0-bit4 map to color IDs 1-5, and `count` is the number of other color IDs detected; the result is sent only after it remains stable for 10 consecutive frames.
- Normal search, lock, carry, return, and yellow-line behavior is unchanged; the extra scan flow starts only after command `0x06` is received.
- Added field auto-calibration, IDE tuning, and front color-scan preview scripts for generating `/sd/color_thr.txt`, checking thresholds, and observing `0x06` scan candidates offline.
- Removed the old standalone model, three-class, return-beacon, and yellow-IPM test scripts; `.gitignore` now excludes `*.tflite` so model files are not committed by accident.

Effect:

- The host can send `0x06` before `0x01` carry mode and make its own obstacle/target decision from the returned other-color IDs.

Verification:

- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py calib_ide_tune.py front_obstacle_scan_test.py` passed.
- `git diff --check` passed.

### 2026-07-09 - v0.7.9-dev - SD threshold loading and slave angle removal

Scope: `main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

Changed:

- `main.py` and `minimain.py` now try to load `/sd/color_thr.txt` at startup. The parser accepts the auto-calibration script format with `exposure_us=`, `ground=` / `ground2=`, and `slot,L0,L1,A0,A1,B0,B1` rows.
- Built-in thresholds are overridden only when all 5 color slots are present; missing, incomplete, or invalid files fall back to the compiled defaults.
- Startup `[color_thr]` prints report whether the runtime loaded the SD threshold file or is using built-in thresholds.
- Removed the `yellow_crossline_ipm.py` import, yellow-angle correction instance, and per-frame angle processing from `minimain.py`. The 16-byte return packet keeps the angle fields, but the slave runtime sends them as zero.
- `minimain.py` keeps command `0x04` reserved and ignored so accidental host commands do not break command parsing.

Effect:

- Field calibration can generate `/sd/color_thr.txt` from the IDE script, and the runtime scripts automatically use it on the next boot.
- The slave runtime has one fewer debug-module dependency and a lighter deployment surface.

Verification:

- `python -m py_compile main.py minimain.py` passed.
- `git diff --check -- main.py minimain.py` passed.

### 2026-07-09 - v0.7.8-dev - Same-color leftmost target acquisition

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Updated initial color-target selection in both runtime entrypoints.
- When several valid blobs have the same color ID, the runtime first chooses the leftmost blob as that color's representative.
- Cross-color selection still compares one representative per color and keeps the existing nearest-bottom target rule.
- Rewrote the root `README.md` as a concise project entrypoint with runtime, tool, and deployment notes.

Effect:

- Repeated targets with the same color are acquired from left to right, while existing cross-color priority behavior is preserved.

Verification:

- `python -m py_compile main.py minimain.py yellow_crossline_ipm.py` passed.
- `git diff --check` passed.

### 2026-07-05 - v0.7.7-dev - Plus slave UART12 and two-stage yellow crossing

Scope: `main.py`, `minimain.py`, `yellow_crossline_ipm.py`, `README_ch.md`, `README_en.md`, `README.md`

Changed:

- Updated `minimain.py` for OpenART Plus hardware: the script keeps slave-role color-ID control but always initializes `UART12`.
- Updated `main.py` yellow-line detection to scan two horizontal ROIs around `y=100` and `y=140`, fit a full-screen yellow line, and use the fitted line during carry mode.
- Split carry-mode yellow crossing into two stages: first arm the crossing only after the fitted yellow line reaches the left or right bottom corner, then report carry completion only after the yellow line disappears for the configured lost-frame threshold.
- Updated `yellow_crossline_ipm.py` defaults to Plus UART12, Plus calibration, and the same mirror capture path.

Verification:

- `python -m py_compile .\main.py .\minimain.py .\yellow_crossline_ipm.py` passed.

### 2026-07-05 - v0.7.6-dev - Color-blob-only runtime

Scope: `main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

Changed:

- Removed the Plus runtime white-bear model configuration, load path, inference helper, and main-loop model branches from `main.py`.
- Color 5 now follows the same LAB `find_blobs()` detection and tracking path as the other targets.
- Confirmed `minimain.py` has no target inference path and marked its runtime target detection as LAB color-blob-only.

Effect:

- The official Plus / Mini runtime no longer imports `tf`, loads `.tflite`, creates temporary scaled images for inference, or calls inference in the frame loop.
- This reduces RAM pressure and frame-loop blocking risk; all targets now use color-block detection.

Verification:

- `rg -n "tf|tflite|model|MODEL|USE_WHITE|WHITE_BEAR|find_white|load_white|model_net|model_tf|\.detect\(" main.py minimain.py` returned no matches.
- `python -m py_compile main.py minimain.py` passed.
- `git diff --check -- main.py minimain.py` passed.

### 2026-06-20 - v0.7.4-dev - OpenART TypeError Compatibility Notes

Scope: `main.py`, `README_ch.md`, `README_en.md`

Changed:

- Documented two field `TypeError` cases seen on OpenART/MicroPython and their fixes.
- `TypeError: function takes 0 positional arguments but 1 were given`: some built-ins or shadowed names on the field firmware may not behave like desktop Python. Do not add `bool(x)` in the runtime path just to normalize truth values. Use the list/blob object directly in conditions, for example `raw_yellow_seen = yellow_blob` or `raw_yellow_seen = yellow_blobs_left and yellow_blobs_right`.
- `TypeError: function takes 2 positional arguments but 1 were given`: after adding helper-function wrappers on OpenART/MicroPython, the reported call site may be indirect, especially around state-machine transitions, globals, and command handling. The fix is to shorten the call chain first, put the key state assignments back at the command-handling site, and then validate incrementally.
- Desktop `python -m py_compile` only proves syntax validity. It does not prove OpenART firmware runtime API or built-in behavior matches desktop Python. Board testing is required for changes involving `bool()`, `max(..., key=...)`, new helper wrappers, or camera/image APIs.

Maintenance rules:

- In the main loop and command handling, prefer MicroPython-compatible condition checks and avoid unnecessary type-conversion wrappers.
- When a field `TypeError` appears, first inspect recently added function calls, built-in calls, and higher-order calls using `key=`, then inspect the algorithm logic.
- Fix these runtime compatibility issues with small changes that preserve the existing state-machine path, so compatibility fixes are not mixed with behavior changes.

Verification:

- `python -m py_compile .\main.py` passed.

### 2026-06-18 - v0.7.3-dev - Plus Offline Freeze Diagnostics

Scope: `main.py`, `yellow_crossline_ipm.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Added lightweight watchdog and SD checkpoint logging to `main.py`; diagnostic checkpoints append to `/sd/watchdog.log`.
- Removed runtime `print()` calls from the Plus main loop so offline runs do not block on unread USB/stdout output.
- Added `RUNTIME_LENS_CORR = False` and disabled per-frame runtime `lens_corr(2)` in `main.py` to isolate frame-buffer and heap pressure.
- Disabled `lens_corr(2)` in the standalone `yellow_crossline_ipm.py` loop so its image path matches the uncorrected calibration view.
- Removed the temporary Plus flip test script from the workspace.

Effect:

- This is a test/diagnostic state, not a final stability claim.
- Logs can be inspected after an offline freeze or watchdog reset to locate the last completed stage.
- Runtime image coordinates now match the current calibration-mode image path, which does not apply lens correction.

Verification:

- `python -c "import pathlib; compile(pathlib.Path('main.py').read_text(encoding='utf-8'), 'main.py', 'exec')"` passed.
- `python -c "import pathlib; compile(pathlib.Path('yellow_crossline_ipm.py').read_text(encoding='utf-8'), 'yellow_crossline_ipm.py', 'exec')"` passed.

### 2026-06-16 - v0.7.2 - 22 cm Calibration Notes

Scope: `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Corrected the `main.py` IPM calibration comment: the Plus camera setup is also `22 cm` above ground, not `12 cm`.
- Updated the `minimain.py` IPM calibration comment to mark the current Mini camera setup as `22 cm` above ground.
- Clarified that IPM points should be recalibrated if either camera height or pitch changes again.

Effect:

- Plus and Mini calibration notes now both document the current `22 cm` setup.

Verification:

- `python -c "import pathlib; compile(pathlib.Path('minimain.py').read_text(encoding='utf-8'), 'minimain.py', 'exec')"` passed.

### 2026-06-15 - v0.7.1 - 22 cm Calibration and Freeze Fix

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Updated the Plus runtime `main.py` IPM calibration for the `22 cm` camera-height setup. `CALIB_PIXEL` now uses the field-measured image points, and `CALIB_WORLD` uses the matching world coordinates.
- Added a shared `snapshot_frame()` entry in `main.py` so capture, lens correction, and software flipping are handled in one place instead of separate direct `sensor.snapshot()` calls in the main loop, brightness calibration, and IPM calibration mode.
- Changed `main.py` to apply `hmirror` / `vflip` together in software, reducing the risk of display corruption or freezes when the OpenART firmware keeps multiple hardware flip states.
- Fixed the Plus UART selection in `main.py`: the master car uses `UART(12)`, and the slave car uses `UART(2)`.
- Re-enabled the white-bear TFLite model path in `main.py`, switched to the current firmware's `model_net.detect()` result interface, and runs `gc.collect()` after releasing the temporary scaled image to reduce memory pressure after model detection.
- Added optional loading of 5 LAB threshold rows from `/sd/params.txt` in `main.py`; invalid or incomplete files fall back to the built-in defaults.
- Synced `minimain.py` to the shared `snapshot_frame()` entry and kept only one hardware flip direction by default, reducing frame-buffer pressure during long Mini / slave-car runs.

Effect:

- Plus IPM world coordinates now match the current `22 cm` camera mounting height.
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
