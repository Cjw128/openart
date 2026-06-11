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
