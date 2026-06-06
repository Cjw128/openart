# OpenART Vision Scripts

OpenART smart car vision scripts for OpenART Plus and OpenART Mini.

## Script Mapping

| File | Target Device | Status |
| --- | --- | --- |
| `main.py` | OpenART Plus | Current main script. Yellow line detection has been fixed. |
| `minimain.py` | OpenART Mini | Mini script. Yellow line detection update has not been synchronized yet. |
| `yellow_crossline_ipm.py` | OpenART Plus / test utility | Yellow crossline and IPM related test script. |
| `openart_test_3class.py` | Test utility | Three-class vision test script. |
| `return_beacon_ipm_test.py` | Test utility | Return beacon IPM test script. |
| `test_model.py` | Test utility | Model test script. |

## Version History

### v0.1.0 - Initial Commit

Date: 2026-06-06

- Created the initial Git repository for OpenART vision scripts.
- Added the OpenART Plus script: `main.py`.
- Added the OpenART Mini script: `minimain.py`.
- Added related test and utility scripts.
- Added cross-platform Git settings for Windows and macOS collaboration.
- Current known state:
  - `main.py` has fixed yellow line detection logic for OpenART Plus.
  - `minimain.py` has not yet received the yellow line detection update.

## Collaboration Notes

- Use `main` as the stable collaboration branch.
- Keep line endings normalized through `.gitattributes`.
- Before modifying Mini behavior, compare `minimain.py` with the fixed yellow line logic in `main.py`.
