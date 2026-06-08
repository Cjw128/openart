# OpenART Vision Change Log

This repository tracks OpenART smart car vision scripts for OpenART Plus and OpenART Mini.

## Current Files

| File | Device | Role |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus entrypoint that selects Plus config and starts the shared runtime |
| `minimain.py` | OpenART Mini | Mini entrypoint that selects Mini config and starts the shared runtime |
| `openart_config.py` | OpenART Plus / Mini | Plus/Mini configuration, including UART role, thresholds, model path, and calibration points |
| `openart_app.py` | OpenART Plus / Mini | Shared vision runtime, including initialization, detection pipeline, host commands, calibration mode, and main loop |
| `openart_detectors.py` | OpenART Plus / Mini | Dynamic cut, color target, white-bear model, obstacle, and return-beacon detection |
| `openart_trackers.py` | OpenART Plus / Mini | Runtime state classes for targets, yellow-line crossing, and return beacon tracking |
| `openart_calibration.py` | OpenART Plus / Mini | Inverse-perspective calibration mode |
| `openart_camera.py` | OpenART Plus / Mini | Camera helper utilities such as startup brightness calibration |
| `openart_uart.py` | OpenART Plus / Mini | RT1021 UART packet builders and send helpers |
| `openart_math.py` | OpenART Plus / Mini | Geometry, homography, IoU, and coordinate transform helpers |
| `yellow_crossline_ipm.py` | OpenART Plus / test | Yellow crossline and IPM utility |
| `openart_test_3class.py` | test | Three-class vision test |
| `return_beacon_ipm_test.py` | test | Return beacon IPM test |
| `test_model.py` | test | Model test script |

## Structure Notes

- `main.py` and `minimain.py` are entrypoints only; they no longer maintain two duplicated main logic copies.
- Plus/Mini device differences live in `openart_config.py`. Do not add device-specific branches to entrypoints unless the hardware behavior truly requires it.
- `openart_app.py` is the shared runtime module. Importing it starts the OpenMV/OpenART main loop; detector, state, UART, math, and calibration details are split into dedicated modules.
- White bear detection uses the TFLite model on both Plus and Mini. Color threshold ID 5 remains in config for protocol and numbering consistency; when model detection is enabled, runtime detection skips white-bear LAB threshold matching.
- Keep all structure notes and iteration records in `README_ch.md` / `README_en.md`, and update both languages together.

## Logs

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
