# OpenART Vision Change Log

This repository tracks OpenART smart car vision scripts for OpenART Plus and OpenART Mini.

## Current Files

| File | Device | Role |
| --- | --- | --- |
| `main.py` | OpenART Plus | Plus main vision script |
| `minimain.py` | OpenART Mini | Mini main vision script |
| `yellow_crossline_ipm.py` | OpenART Plus / test | Yellow crossline and IPM utility |
| `openart_test_3class.py` | test | Three-class vision test |
| `return_beacon_ipm_test.py` | test | Return beacon IPM test |
| `test_model.py` | test | Model test script |

## Logs

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
- Before changing Mini behavior, compare `minimain.py` with the corresponding fixed logic in `main.py`.
