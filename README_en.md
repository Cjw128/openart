# OpenART Vision Change Log

This repository tracks a dual-OpenART-Plus smart-car vision system; both current camera boards are OpenART Plus.

## Current Files

| File | Device | Role |
| --- | --- | --- |
| `main.py` | OpenART Plus / master | v1.1.0 production model/blob fusion runtime |
| `minimain.py` | OpenART Plus / slave | v1.1.0 production model/blob fusion runtime |
| `camera_ground_mesh.txt` | OpenART Plus / both | Board-side 28-point, 36-triangle ground mesh |
| `ground_mesh_24_points_template.csv` | PC | Current 28 pixel/world calibration points |
| `calibrate_ground_camera.py` | PC | Mesh generation, validation, and reporting tool |
| `camera_ground_mesh_report.json` | PC | Current mesh quality report |
| `raw_ground_projection_test.py` | OpenART Plus / IDE | Ground-coordinate collection and field verification tool |
| `world_coordinate_test.py` | OpenART Plus / IDE | All-class world-coordinate observer aligned with the full master detector |
| `test_ground_projection.py` | PC | Runtime projection regression tests |
| `test_model_blob_fusion.py` | PC | Master/slave model-blob fusion regression tests |
| `calib_ide_autocalib_competition.py` | OpenART Plus / IDE | Competition field auto-calibration and preview script |
| `front_obstacle_scan_test.py` | OpenART Plus / IDE | Pre-carry front color-blob scan preview script |

## Structure Notes

- Deployment keeps the single-file layout. `main.py` and `minimain.py` contain the complete master and slave logic and additionally load `/sd/camera_ground_mesh.txt` at startup.
- The multi-file runtime modules have been removed. The deployment no longer uses `openart_app.py`, `openart_config.py`, `openart_detectors.py`, `openart_trackers.py`, `openart_uart.py`, `openart_math.py`, `openart_camera.py`, or `openart_calibration.py`.
- Color detection, yellow-line state, the UART protocol, and the main loop remain in each single-file runtime. Ground-mesh generation and field verification remain standalone tools.
- `fast_blob_backup/`, `stable_confirm/`, `stable_no_priority/`, and `mainbak` are historical references, not deployment entrypoints.
- The multi-file version caused TFLite detection freezes. See the v0.4.0 log for the investigation and maintenance rules; do not restore the v0.3.0 modular structure as the competition deployment layout.
- The root `README.md` stays as a concise deployment guide; detailed changes live in `README_ch.md` and `README_en.md`.

## Logs

> **Current dual-car hardware rule: both cameras are OpenART Plus boards, and `main.py` and `minimain.py` both use `UART12` at 115200 bps. The files are fixed master/slave entrypoints; the unreferenced `IS_SLAVE_CAR` / `SLAVE_MODE` switches have been removed.**

### 2026-07-26 - v1.1.0 - Memory and hot-path optimization release

Scope: `main.py` (2611 → 2650 lines), `minimain.py` (2407 → 2376 lines), `world_coordinate_test.py`, `test_ground_projection.py`, and the three READMEs. Baseline: v1.0.0 (commit `d83b6d6`, 2026-07-25).

This is a memory and hot-path optimization round. Except for the single behavior change below, every change is bit-for-bit behavior-equivalent. The `test_model_blob_fusion.py` gate passes all 13 tests, including three-way AST synchronization of `main.py`, `minimain.py`, and `world_coordinate_test.py`, and the `test_ground_projection.py` gate passes all 6 tests, including byte-identical ground-projection blocks across the three entries.

Behavior change (single, intentional):

- The `0x06` front-scan command handler now additionally calls `reset_yellow_state()`: a front-scan request received while carrying clears the yellow-line fit and progress flags, so the yellow line must be reacquired after the scan (v1.0.0 carried the yellow-line state into `MODE_SEARCH`).

Performance optimizations (behavior-equivalent, all in `main.py`):

- `mesh_ground_pixel_to_world`: the triangle scan resumes from the previously hit mesh cell (consecutive contact points almost always stay in the same cell, dropping ~36 triangle tests to ~1); per-triangle fields are unpacked by direct indexing, removing 3 temporary tuples per triangle; the barycentric b/c computation short-circuits early.
- Added a lazy per-row cache of the `u=160` centerline projection (240 rows): `box_to_world` reads the table directly when the pixel-centre row hits, saving one full mesh projection per frame.
- `model_track` shrinks from 14 to 11 slots: removed the never-read contact-velocity EMA and `sample_time` bookkeeping (~6 fewer float boxings, 1 fewer `ticks_diff`, and 2 fewer divisions per inference frame).
- `sample_box_lab_stats` is extracted as a shared helper reused by `sample_model_color` and `front_scan_color_id`; `model_candidate_matches_requested_color` now returns `(matches, sampled)`, so lock-acquisition frames no longer run a second `get_statistics` sample on the same box.
- `run_model_best`: `img.width()` / `img.height()` hoisted into locals; the `host_forced_target_active()` call chain runs once per frame; redundant `float()` conversions removed.
- `build_dynamic_threshold` / `build_dynamic_channel` / `threshold_center_distance` / the IQR check are unrolled across the three channels, removing the `range` / `list` / `tuple()` allocations from every color sample.
- `draw_carry_yellow_line` drops from 4-8 heap objects per frame to 4 scalars; `pick_yellow_blob` hoists loop invariants and removes one 4-tuple unpacking allocation per blob; the `model_lock` slice assignment becomes element-wise; the frame loop's `draw_rectangle` reuses the existing `output_box` tuple.
- The throttle gate of `update_dynamic_cut` moves inside the function. The outer `lab_frame` force path was already dead code in v1.0.0: the interval check inside the function meant it could never take effect.
- Import-time peak memory reduction: `load_ground_projection` releases each `triangleN` source string as it goes, no longer retains the `pointN` value strings, and builds each triangle tuple in one step (~5KB less transient import-time garbage).
- Dead code removed: `find_color_blobs_once`, `output_model_color_id`, `model_labels_compatible`, `yellow_line_y_at_x`, `model_high_score_minimum`, `_color_threshold_groups`, `MULTICOLOR_MIN_PIXELS/AREA`, `TRACK_MAX_JUMP_PX/JUMP2`, `MODEL_SCORE_HIGH_NEAR/MID/FAR`, `CONTACT_VELOCITY_ALPHA`, `YELLOW_DETECT_INTERVAL`, `first_lock_pending_sample_time`, and a dead `import time`.
- `maybe_collect` and `receive_command_from_host` keep their v1.0.0 behavior: forced GC every 30 frames with a 48KB low-water mark, and at most one host command consumed per frame.

File synchronization:

- `minimain.py` (car-B slave) receives its first full synchronization of every applicable `main.py` optimization, 29 ports in total (mesh scan resumption, row cache, `model_track` slimming, sampling reuse, dead-code removal, and the rest), plus removal of the slave's own dead `lab_frame` gate and dead `import time`.
- `world_coordinate_test.py` (IDE observer) is realigned with `main.py` (56 top-level node replacements/deletions/insertions plus 3 point edits in the frame loop). The observation instrumentation is fully preserved and now carries its own `_world_coord_time` reference, since `main.py` no longer imports `time`.
- The ground-projection block (`WORLD_X_LIMIT_CM` through `clamp_int`) is re-synchronized byte-for-byte across the three entries; the `test_ground_projection.py` extraction harness is adapted to the new v1.1.0 `_mesh_last_triangle` resume-scan global, `ground_center_x_cache`, and `center_line_world_x_for_row` row cache, and all 6 projection regressions (28-point round-trip, bounded full-QVGA coordinates, millimetre conversion, etc.) pass.

### 2026-07-25 - v1.0.0 - Production model/blob fusion baseline

Scope: the complete current repository. Runtime entry points are `main.py` and `minimain.py`; `world_coordinate_test.py` is the all-class coordinate validation entry point.

- Promoted the v0.11.5-dev runtime, which passes the complete desktop regression and all three OpenART compilation checks, to the first formal `1.x` release. This promotion changes version identity only; detection parameters, state machines, and coordinate results are unchanged.
- The production baseline includes three model classes and five color IDs, dynamic LAB blob fusion, switchable absolute ID2 priority, all-class detection, the 28-point ground mesh, image-centred X correction, stable fast-following world coordinates, and the existing master/slave UART protocol.
- Full blobs continue to own display geometry; the model continues to own discovery, class confirmation, and reacquisition. The world contact retains its `50%` current-frame position weight, `2 px` spatial deadband, and `18 px` fast-jump threshold.
- The v0.x entries remain as development and provincial-competition history. Compatible improvements use v1.x; a deployment-protocol or runtime-contract break requires the next major version.
- The release baseline is protected by 19 desktop regressions, Python syntax checks, `git diff --check`, and `mpy-cross` compilation of `main.py`, `minimain.py`, and `world_coordinate_test.py`.

### 2026-07-25 - v0.11.5-dev - Normal-motion and fast-approach response

Scope: `main.py`, `minimain.py`, `world_coordinate_test.py`, `test_model_blob_fusion.py`, and the three READMEs.

- To address world-coordinate lag during normal movement and late jumps during rapid approach, coordinate-position smoothing is now independent from display-box smoothing.
- The display box retains a `35%` current-frame weight, while the raw coordinate position uses `50%`. The `2 px` contact deadband is unchanged, preserving stationary jitter suppression while motion response becomes faster.
- The direct reset threshold is reduced from `24 px` to `18 px`. A `17 px` input remains smoothly tracked with the coordinate point advancing faster than the display box; at `18 px`, the coordinate switches to the new position in the same frame.
- The 28-point ground mesh, image-centred X correction, Y mapping, display-box dimensions, ID2 priority, and UART protocol are unchanged.
- All 19 desktop regression tests pass, including new fast-approach response and exact `18 px` jump-boundary coverage.

### 2026-07-25 - v0.11.4-dev - Stable close-range world contact point

Scope: `main.py`, `minimain.py`, `world_coordinate_test.py`, `test_model_blob_fusion.py`, and the three READMEs.

- Comparison with the provincial stable build confirmed that its low jitter mainly came from a spatially limited model contact point. In v0.11.3, the full blob bottom directly included shadow, floor merge, and threshold-edge changes, which ground projection amplified at close range.
- The full blob still owns the display box. Only its bottom-centre point is stabilized before projection: the default `2 px` circular spatial deadband freezes small changes, then follows real movement immediately while retaining at most `2 px` spatial error. This avoids the long tail of an ordinary temporal EMA.
- A raw contact displacement of `24 px` or more from the previous stable point resets immediately for true rapid movement and reacquisition. Display geometry retains the existing `35%` current-value smoothing and does not restore the provincial build's undersized model-box geometry.
- The observer now prints `raw_pixel`, `stable_pixel`, and `delta_px`, marking the raw contact with a small red cross and the stabilized contact with a larger yellow cross.
- The 28 calibration points, 36 triangles, image-centred X correction, homography fallback, Y mapping, and UART millimetre units are unchanged.
- Eighteen desktop regression tests cover small-jitter freezing, reduced close-range bottom-edge oscillation, final convergence after true motion, immediate large-jump reset, separate display/coordinate geometry, and three-entrypoint identity.

### 2026-07-25 - v0.11.3-dev - Switchable absolute ID2 priority

Scope: `main.py`, `minimain.py`, `world_coordinate_test.py`, `test_model_blob_fusion.py`, `test_ground_projection.py`, and the three READMEs.

- Restored the first-target gate from the provincial `01_稳定版_ID2先_5中7` build and added `ID2_ABSOLUTE_PRIORITY` to the quick settings in all three current entry points. It defaults to enabled.
- When enabled, startup and `0x08` reset only allow ID2 through ordinary model search, dynamic LAB confirmation, host `0x03` requests, and world-coordinate output. A nearer or higher-confidence non-ID2 target cannot lock. Once ID2 is completed, all other incomplete IDs return to nearest-world-Y competition.
- The master retains yellow-line completion and the slave retains pending-ID completion after a carry. The `0x06` all-color front scan bypasses the priority gate. If ID2 is absent, the runtime waits rather than falling back to another ID.
- Disabling the switch restores v0.11.0 behavior: every incomplete ID competes by nearest world Y from startup.
- All 14 desktop regression tests pass, including ID availability before ID2 completion, after completion, and with the switch disabled. Python syntax and `git diff --check` also pass.

### 2026-07-25 - v0.11.2-dev - All-class world-coordinate observer

Scope: `main.py`, `minimain.py`, `world_coordinate_test.py`, `test_model_blob_fusion.py`, `test_ground_projection.py`, and the three READMEs.

- Added the standalone OpenART IDE entry point `world_coordinate_test.py`. It retains the complete current `main.py` pipeline: three model classes, five color IDs, initial `5/7` lock, dynamic LAB thresholds, model/blob geometry fusion, nearest-target selection, ground mesh, and homography fallback. It does not use the red-bag-only simplified detector.
- Every normally locked class prints the final coordinate box's bottom-center pixel, centimetre world coordinates, and the rounded millimetres used by the production UART packet. The framebuffer also marks the contact point. Output defaults to once per `200 ms`; `NO_TARGET` is printed once per second while unlocked.
- The master, slave, and observer all subtract the raw X projection at `u=160` on the contact point's scanline. This makes the image centerline exactly `X=0` at every distance while preserving Y and the calibrated lateral scale. The shared `GROUND_CENTER_X_ON_IMAGE` switch restores the old mapping for field A/B tests; observer logs additionally retain `raw_x` and `x_bias`.
- `HELD / TRACK / MODEL_FRAME` identifies held output, an ordinary tracking frame, or a model refresh frame so coordinate jumps can be tied to the active detection path.
- A full AST guard verifies that removing `_world_coord_*` instrumentation leaves a script identical to `main.py`. The observer also participates in the model/blob fusion and ground-projection regression suites.
- All 13 desktop regression tests pass, including exact `X=0` projection along the image centerline at multiple distances. Python syntax checks, `git diff --check`, and MicroPython `mpy-cross` compilation also pass.

### 2026-07-23 - v0.11.1-dev - Model recognition with dynamic-blob geometry

Scope: `main.py`, `minimain.py`, `test_model_blob_fusion.py`, and the three READMEs.

Version preservation:

- The pre-change provincial v0.11.0 build is preserved at commit `41260c0`. Branches `dedicated-model` and `archive/v0.11.0-ground-mesh` both point to it and have been pushed to `origin`, while this experiment continues on `model-blob-fusion` without replacing the archive.
- The archive retains the car-tested `04_备用版_无优先级_5中7` baseline, SS model, `880 us` exposure, nearest-target acquisition without ID priority, initial `5/7` confirmation, the 28-point ground mesh, and UART coordinates in millimetres.

Fusion behavior:

- The previous path took width and height from the model box and used the blob only as a relative displacement. An incomplete model box therefore kept the final geometry undersized and could introduce a periodic size change on each four-frame model refresh. This model-anchored geometry path has been removed.
- The model remains responsible for initial discovery, model-class confirmation, dynamic LAB threshold creation, periodic validation, and reacquisition after loss. Once a dynamic blob is confirmed, its box directly owns both display geometry and the bottom contact point used for world coordinates.
- The first full-blob search expands the model box by `50%`. Once a blob exists, tracking uses a local ROI expanded by `45%`. A model-refresh frame unions the model and existing-blob gates while retaining the local blob ROI, so refresh no longer forces tracking back into an incomplete model box.
- Initial candidate centre tolerance considers both the model-box size and the candidate-blob size, allowing a complete blob to be materially larger than a clipped model box. Area, overlap, color-ID, and dynamic-field constraints still reject nearby distractors.
- Final centre and size smoothing keeps `65%` of the previous box and accepts `35%` of the current blob; a large relocation still resets immediately to avoid lag. A transient blob miss holds the previous result. Local blob search resets after three consecutive misses, output may be held for up to five frames, and only then falls back to model geometry or waits for reacquisition.

Limits and validation:

- This is a development experiment for on-car comparison, not a replacement for the archived provincial build. Because the blob now owns geometry, a dynamic threshold that fragments the object, merges floor pixels, or absorbs shadows can still jitter both the box and world coordinates. Field testing should display the raw model box, raw blob box, and final smoothed box separately.
- Added six fusion regression tests covering first-search expansion, model-refresh ROI behavior, blob geometry ownership, smoothing weight, removal of the old anchor path, and AST identity of all master/slave fusion constants and helpers.
- All six fusion tests and five existing ground-projection tests pass. `python -m py_compile` and `git diff --check` pass, and `mpy-cross` compiles both `main.py` and `minimain.py`.

### 2026-07-22 - v0.11.0 - Provincial build-04 no-priority multi-point world coordinates

Scope: `main.py`, `minimain.py`, `ground_mesh_24_points_template.csv`, `calibrate_ground_camera.py`, `camera_ground_mesh.txt`, `camera_ground_mesh_report.json`, `raw_ground_projection_test.py`, `test_ground_projection.py`, and the three READMEs.

Baseline and runtime settings:

- Rebased both official entrypoints on the car-tested `04_备用版_无优先级_5中7` burn directory. Automatic acquisition has no color-ID or model-class priority: it ranks every eligible candidate by nearest world Y and keeps the first-lock rule of at least 5 matching hits in 7 real inference frames.
- A host `0x03` color request remains explicit directed search with build 04's `0.25` threshold and `3/5` confirmation. This is not automatic ID priority.
- Both runtimes use `/sd/80lite0.5SS.tflite`, fixed white balance `(92.00, 64.00, 101.00)`, and the field fallback exposure of `880 us`. An `exposure_us=` row in `/sd/color_thr.txt` can still override that default.
- Color recognition, dynamic ground cropping, carry state, front scanning, return-line handling, and the master/slave 16-byte UART packet layout retain the completed provincial behavior.

Multi-point world-coordinate rewrite:

- Restored the multi-point tooling approach from historical commit `fac7b92` and rebuilt it around the existing `ground_mesh_24_points_template.csv`. The old filename is retained, but the current data contains 7 rows x 4 columns, or 28 fit points.
- The PC generator orders structured rows from near to far and exports 36 triangles plus 18 direction-classified hull edges. Barycentric interpolation inside the mesh reproduces every calibration vertex exactly.
- Pixels outside the mesh use a global homography fitted from all 28 points, corrected at the nearest mesh boundary for local/global continuity. Far Y is capped at `164 cm`, and X is bounded to `-250..250 cm`.
- Runtime loading strictly checks the mesh schema, role, QVGA dimensions, software horizontal mirror, sensor vertical flip, disabled `lens_corr()`, Y range, triangle orientation, and fallback matrix. Missing or invalid files use only the embedded global homography and are never reported as a local mesh.
- The visible bottom-center point `(x + w/2, y + h - 0.5)` is the shared ground contact point. Calculations use centimetres internally and round to millimetres before transmission, preserving the provincial protocol rather than restoring v1.0.0's historical `0.1 mm` scale.
- The two entrypoints intentionally expect the same `role=master` mesh for now. A slave camera with different mounting geometry requires separate samples, a `role=slave` mesh, and the matching slave runtime role check.

Generated result and limits:

- Calibrated Y range is `6..164 cm`; the mesh covers `61.9%` of QVGA, with a minimum triangle angle of `7.82 deg`.
- The global fallback homography fit has `1.679 / 3.281 cm` RMS / maximum error. Leave-one-out diagnostics report `1.994 / 4.056 cm` RMS / maximum error.
- All 28 current rows are `split=fit`; there are no independent `verify` points. Generator QA therefore proves structural and internal constraints only. Final accuracy must be measured on both cameras with `raw_ground_projection_test.py`.

Repository cleanup:

- Removed unused `calib_ide_tune.py`, `capture_field_images.py`, `color_thr.txt`, `image.png`, `main_autocalib_test.py`, `match_field_capture_to_reference.py`, and `return_yellow_test.py`. They remain recoverable from Git history.

Validation:

- The generator exported 28 points, 36 triangles, and 18 boundaries; the report passed QA while explicitly warning that no independent verification points exist.
- All five `test_ground_projection.py` tests passed: identical master/slave projection code, exact reproduction of 28 vertices, bounded coordinates for all 76,800 QVGA pixels, correct UART millimetre rounding, and correct build-04 model/exposure settings.
- `python -m py_compile` and `git diff --check` passed. MicroPython v1.27.0 `mpy-cross` compiled `main.py`, `minimain.py`, and `raw_ground_projection_test.py` successfully.

### 2026-07-19 - v0.10.5 - Stable initial-lock auto-detection fix

Scope: `stable_confirm/main.py`, `stable_confirm/minimain.py`, `stable_confirm/README.md`, `README.md`, `README_ch.md`, `README_en.md`

Changes:

- Removed the hard gate that required `0x02` followed by a `300 ms` wait before the stable runtime could run initial model detection. This fixes permanent no-target output during standalone IDE runs or before the host sends `0x02`.
- Kept nearest-candidate ranking by world distance, the uniform `0.30` first-lock score floor, at least `5` matching hits in `7` real inference frames, and removal of the high-confidence one-frame lock shortcut.
- Restored LAB dynamic-color confirmation to the root runtime's default `3` frames. `0x02` and `0x00` now only clear the current target and restart search; they no longer grant detection permission. Repeated `0x02` packets in one phase remain debounced and do not repeatedly erase the `5/7` count.

Validation: both scripts passed syntax checks; no first-lock cycle/settle gate references remain, and the model path and preprocessing still match the root runtime.

### 2026-07-19 - v0.10.4 - Communication reset and stable initial lock

Scope: `main.py`, `minimain.py`, `stable_confirm/main.py`, `stable_confirm/minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changes:

- Added the `0x08` host command with the four-byte frame `AA 55 08 08`. All four runtimes clear the ID1..ID5 completion mask, release the current color/model lock, and restore unrestricted model search. Slave runtimes also clear the pending carry ID.
- Kept the root files as the original fast variant and added independently deployable master/slave runtimes under `stable_confirm/`. The stable variant treats `0x02` as the ready trigger, waits `300 ms`, admits first-lock candidates at a uniform `0.30` score floor, ranks them strictly by world distance, and requires `5` hits in a `7`-frame window.
- The stable variant removes the high-confidence one-frame initial-lock shortcut. A nearer candidate appearing during confirmation replaces the farther pending candidate. If `0x08` arrives during an active `0x02` cycle, settling and confirmation restart.

Validation: all four scripts passed syntax checks; simulated UART tests verified clearing of the completion mask, slave pending ID, target lock, and front-scan state; a near `0.31` candidate outranked a far `0.99` candidate.

### 2026-07-19 - v0.10.3 - Disable the red-bag-specific aspect filter

Scope: `main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `README.md`, `README_ch.md`, `README_en.md`

Changes:

- Commented out the color-ID2-specific `width / height <= 1.70` filter and its equivalent model-box color-sampling check. The code remains commented for easy field restoration.
- Red bags again use the original generic bag limits, `0.60 <= width / height <= 1.80` with density `>= 0.40`, preventing valid red bags from being rejected by the dedicated ceiling.
- Synchronized the ten-frame calibration verification, calibration runtime preview, and host-command test entrypoint. `0x06` already skipped target aspect ratios and is unchanged.
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py` passed, and `git diff --check` passed.

### 2026-07-19 - v0.10.2 - One successful carry per color ID

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changes:

- Added an ID1..ID5 completion bit mask. The master records the current ID only when carry mode confirms a yellow-line crossing and emits `POS_CROSSED` for the first time; ordinary detection, merely seeing the line, and search resets do not count as completion.
- The slave has no carry yellow-line state machine and no longer depends on an `0x02` that may be skipped by the automatic flow. It only latches the current ID on `0x01`, then commits it when the next post-carry/search-reset `0x00` or `0x02` arrives. An ordinary `0x00` records nothing when no carry is pending.
- Completed IDs are excluded from normal model candidates, model-guided color sampling, and final coordinate output. A repeated `0x03,id` clears target state instead of locking again. When a model class still contains an unfinished color—for example, ID1 is complete but ID2 is not—the runtime samples LAB before accepting the candidate.
- `0x06` continues to return every valid color-blob ID and does not apply the completion mask. Completion and the slave's pending-carry ID are RAM-only and clear when OpenART restarts.
- `python -m py_compile main.py minimain.py` passed, and `git diff --check` passed.

### 2026-07-19 - v0.10.1 - Red-bag and field-brick shape separation

Scope: `main.py`, `minimain.py`, `main_autocalib_test.py`, `calib_ide_autocalib_competition.py`, `front_obstacle_scan_test.py`, `README.md`, `README_ch.md`, `README_en.md`

Changes:

- Added a dedicated horizontal aspect-ratio ceiling for color ID2 red bags: a candidate must satisfy `width / height <= 1.70`. This relaxes the initial `1.50` limit to tolerate perspective and fragmented normal-bag blobs; the existing `width / height >= 0.60`, density, and pixel limits remain unchanged, while ID1 blue bags retain `0.60..1.80`.
- Applied the rule to normal target LAB candidates and model-guided color sampling in the master and slave runtimes, preventing the model fallback from accepting an obviously wide, flat red field brick as a pickup target.
- Preserved the `0x06` “all valid color IDs” semantics with a target-shape-independent scan validator: every other ID passing color, pixel/area, density, and valid-region checks is accumulated into the mask, so a horizontal red brick is still returned as ID2. The tracked target remains reported separately through `current_id`.
- Synchronized the front all-color scan test, ten-frame field-calibration verification, calibration runtime preview, and host-command auto-calibration test so each entrypoint keeps the intended semantics.
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py front_obstacle_scan_test.py` passed.

### 2026-07-16 - v0.10.0 - Model-guided field calibration and stable runtime recognition

Scope: `calib_ide_autocalib_competition.py`, `main.py`, `minimain.py`, `front_obstacle_scan_test.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:

- Upgraded the competition calibration script into a continuous five-target workflow, currently identified as build `2026-07-15 red-bag-bear-core-v21`. The model is loaded from `/sd/dataset_25000_exposure.tflite` only by the IDE calibration script; the deployed master and slave runtimes remain LAB-blob based and do not require the model.
- Lowered the full-frame exposure target from `L=40` to `L=38`, tightened convergence to `±1`, and moved upper-quartile highlight protection to `Luq=92`, reducing chroma loss on white bears and other bright surfaces.
- The selected exposure now updates the `exposure_us` row in the official file immediately while preserving existing ground and five-color rows. Exposure therefore changes even if a later target fails calibration, so a known-good file should be backed up before recalibration.
- Reworked near `ground` and far `ground2` collection into tiled sampling. Tiles intersecting model boxes are excluded and LAB-median outliers are rejected before each six-value ground box is produced, reducing contamination from targets, hands, and isolated highlights.
- Replaced fixed model-box expansion with multi-ray LAB edge scans on all four sides. Each ray searches from an internal core and confirms the boundary backward from continuous ground or stable background; unreliable sides fall back to conservative class-specific bear / ball / bag expansion.
- The tennis ball now uses a compact central core plus an optional inset lower strip. The strip is excluded when it matches ground, has excessive IQR, or differs too much from the core A/B medians, preserving dark ball edges without absorbing blue floor.
- Bag thresholds use a core for ground-conflict decisions and merge edge strips only after LAB consistency checks. Tennis, bag, brown-bear, and white-bear A/B span limits are independently set to `75 / 60 / 75 / 60`.
- Bear expansion is limited below the object and the bottom of the statistical ROI is removed. The brown-bear L lower bound cannot fall more than `14` below its body median, preventing a large floor shadow from entering the brown threshold.
- Enforced the collection order `either bag -> the other bag -> tennis ball -> brown bear -> white bear`. Because the model exposes only a generic bear class, the two bears are assigned by phase. A completed object must leave the view for eight consecutive clear frames before the next slot can collect.
- Each target requires ten accepted samples. Brown bear, white bear, and red bag now require stable model boxes; when the red bag moves to another location, three stable frames at the new location automatically rebase and restart its samples instead of remaining locked to the old box.
- Added a ten-frame model/blob recheck for every generated threshold. It checks missing blobs, disjoint boxes, center distance, overlap, relative area, side coverage, and frame-to-frame center/size/area jumps. Three consecutive bad frames, four bad frames in total, or two severe jumps invalidate the threshold and immediately restart collection.
- After both bears finish, their thresholds are forcibly separated on the LAB channel with the greatest median difference. The medians must differ by at least `8`, with a `2`-unit margin on each side of the cut; a final audit requires zero overlap and each median to remain inside its own threshold. Both bear slots are discarded if the audit fails.
- Only five complete, audited slots replace `/sd/color_thr.txt`; incomplete output goes to `/sd/color_thr_partial.txt`. A runtime adopts file-based exposure, colors, and ground only when the official file contains all slots `1..5`.
- Both runtimes now parse `ground` and `ground2` and integer-average all six LAB bounds coordinate by coordinate for dynamic blue-ground detection, falling back to the available single row. When white-bear A/B genuinely overlaps the averaged ground box, L is used to lift the white threshold away from ground.
- The master dynamic cut uses five distributed vertical strips, requires at least three valid strips spanning `180 px`, and adds ground-gap bridging, robust vertical selection, bounded per-frame movement, and EMA. Target search begins `10 px` above the detected boundary, while targets crossing the boundary remain eligible.
- The slave now averages the valid tops of five strips when at least two are available and applies EMA. It is not the left/right interpolated sloped line documented by v0.9.9, and the current documentation now reflects the actual implementation.
- Reworked all-color acquisition into one non-merging `find_blobs()` pass. Multi-bit codes caused by overlapping thresholds are treated as ambiguous instead of defaulting to the lowest color ID; locked targets continue through local-ROI, IoU, center-distance, and area-continuity checks.
- Brown and white bear fragments use `12 px` and `10 px` merge margins. White-bear output boxes are smoothed with `2/3` old and `1/3` new coordinates to stop monitor-box size oscillation, while severe relocations switch directly to the new box.
- Added tennis-shadow relationship filtering. A brown candidate below the ball, overlapping at least `60%` horizontally and no larger than `55%` of the ball box, is rejected as a contact shadow; a brown-bear lock also scans the tennis threshold for the reference ball.
- Replaced board-side 8x8 homography solving and four-corner averaging with a precomputed matrix and the target's bottom-center contact point. X is limited to `±250 cm` and Y to `0..300 cm`, avoiding firmware tuple/type arithmetic failures and horizon-induced sign flips.
- Reduced the `0x06` pre-carry scan threshold to `60` pixels. Matching `(current_id, mask, count)` results return early after six consecutive frames; observation stops at twelve frames and sends that frame's current result if it never stabilized. The carried target remains excluded by IoU or center distance, and the reply remains the seven-byte `0xC7` packet.
- Synchronized `front_obstacle_scan_test.py` with current color pixel/area limits, one-pass multicolor scanning, horizontal dynamic ROI, and bottommost carried-target exclusion so candidates and masks can be inspected without the controller.
- Retuned the master carry-yellow LAB threshold to `(62, 100, -57, 13, -8, 127)`. Candidates substantially overlapping the tracked object, near-vertical fits, and slopes beyond `±45°` are rejected; the accepted fit is drawn in the debug view.
- Kept the existing dual-car `0x07` horizontal return-line flow and documented the full seven-byte `0xC7` and `0xC8` layouts in the root README. Master and slave retain independent return-yellow LAB thresholds.
- Added OpenART firmware guards: explicit scalar/integer loops replace generator-sensitive parsing, calibration values are formatted field by field and read back after writing, LAB medians are unwrapped and validated before arithmetic, unsupported `find_blobs(..., margin=...)` calls fall back automatically, and invalid world/display data or a single color-detection error no longer terminates the main loop.

Deployment and verification:

- Run the calibration script in the OpenART IDE, complete all five targets, and confirm `[bear] PASS ... overlap=0`. A partial file must never be renamed and deployed as the official calibration.
- Deploy `main.py` to the master and `minimain.py` to the slave, each as `/sd/main.py`, and use the `/sd/color_thr.txt` generated by that specific camera.
- `python -m py_compile calib_ide_autocalib_competition.py main.py minimain.py front_obstacle_scan_test.py` passed.
- OpenART `mpy-cross` compilation passed for all four changed scripts, and `git diff --check` passed.
- Desktop checks cannot reproduce camera APIs, field lighting, or MicroPython firmware behavior. Both OpenART Plus boards still require field validation of all five targets, bear separation, tennis-shadow rejection, dynamic cropping, the six-frame `0x06` result, `0x07/0xC8` return-line reporting, and long-running stability.

### 2026-07-14 - v0.9.9 - Tennis carry yellow-line special case and slave cut-line fix

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:
- Added the master-side `carry_target_color_id` snapshot. On command `0x01`, the runtime confirms the current carry color only when the host `target_color_id` matches the locally tracked `color_track_color_id`; stale locks, lost targets, and mismatched colors cannot enable the tennis special case.
- During a confirmed tennis-ball carry, the upper and lower yellow-line ROIs expand to `y=70..119` and `y=120..169`, with `7/5` initial/hold pixel thresholds. The first successful line fit reports `POS_CROSSED` in the same frame and enters `MODE_WAIT_TURN` without waiting for consecutive confirmation, bottom-corner arrival, or line loss.
- Non-tennis targets retain the original ROIs, `70/20` pixel thresholds, and two-stage crossing state machine. This prevents a red bag from inheriting the low tennis thresholds and ending its carry early.
- Restored the slave camera's own blue-ground LAB threshold, separate left/right samples, and interpolated sloped cut line. Each target is filtered against the cut line at its own x-coordinate, fixing false rejection and false recognition caused by a single averaged horizontal cut under an oblique view.

Verification:
- Field retesting reported a stable carry flow, with red bags no longer triggering the tennis early-exit path.
- `python -m py_compile main.py minimain.py` passed.
- Carry-color command snapshots, tennis-only early exit, yellow ROI/pixel-parameter isolation, and slave sloped-cut branch tests passed.
- `git diff --check` passed.

### 2026-07-12 - v0.9.8-dev - Offline hot-path cleanup and vision-return removal

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:
- Removed vision-return mode, return-beacon thresholds/tracking state, and the associated `find_blobs()` path from both cars. Command `0x05` is still validated and consumed as a four-byte frame but no longer changes runtime state.
- Checked the latest RT1021 controller tree and confirmed that its business logic no longer sends `0x05`; active commands are `0x00/0x01/0x02/0x03/0x04/0x06`. The 16-byte world-coordinate return packet is unchanged.
- Completely removed the bird-view switch, reverse homography, extra frame buffer, and per-pixel renderer. `CALIBRATION_MODE`, four-point IPM capture, calibration verification graphics, and `H_pix2world` coordinate conversion remain intact.
- Kept color-coded target bounding boxes in the normal offline path while removing target crosses/text, dynamic-cut lines, and yellow-line debug overlays. Unused distance calculations, startup banners, and runtime debug formatting were also removed.
- Removed the zero-call brightness-calibration chain, legacy 14-byte pixel protocol, obsolete local-ROI state, obsolete lock counters, ineffective role switches, and write-only yellow-boundary world coordinates. Fixed exposure, SD threshold loading, color/yellow detection parameters, and state machines are unchanged.
- Cached single-color threshold lists and fixed cut ROIs, changed candidate filtering to an equivalent single-pass selection, and removed the temporary corner list from world-coordinate conversion.
- Preallocated separate target/no-target 16-byte UART buffers and replaced sliced checksums to avoid per-frame `bytearray` and `data[2:15]` allocations.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- The old and new initial-candidate selectors matched across `10000` randomized cases.
- The old and new target/no-target UART encoders matched byte-for-byte across `4000` randomized frames.
- A parser test with `0x05` immediately followed by `0x03` passed without losing command-buffer synchronization.
- `git diff --check` passed.

### 2026-07-11 - v0.9.7-dev - Master yellow-line threshold and documentation sync

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:
- Adjusted the master yellow-line LAB threshold from `(48, 94, -27, 51, 12, 127)` to `(51, 91, -32, 36, 1, 118)`.
- The slave retains its independent yellow-line threshold and current IPM values. Its calibration comments now refer to the current OpenART Plus hardware and state that parameters from an old Mini installation cannot be reused without recalibration on the current board.
- The root README now identifies both the latest stable and current development versions. The v0.9.5/v0.9.6 baseline differences are explicit, and the missing English v0.7.5-dev history has been restored.

Verification:
- Syntax checks passed for every Python file in the repository.
- `git diff --check` passed.

### 2026-07-11 - v0.9.6-dev - Separate ID1/ID2 minimum-pixel threshold

Scope: `main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

Changed:
- Added `COLOR_ID12_MIN_PIXELS = 100` to both runtimes so IDs 1 and 2 explicitly use `100` pixels. This lowers them from `150` in the intermediate v0.9.5-dev state while keeping their net value unchanged from stable v0.9.0.
- The default `COLOR_MIN_PIXELS = 150` now applies to bear IDs 4 and 5. Relative to v0.9.0, their initial `find_blobs()` threshold increases from `100` to `150` while the existing secondary filters remain unchanged. Tennis-ball ID3 remains at `45` pixels.
- The `0x06` pre-carry scan keeps `FRONT_SCAN_MIN_PIXELS = 150` unchanged.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` passed.

### 2026-07-11 - v0.9.5-dev - Dual-car target ROI set 10 px above the blue-ground boundary

Scope: `main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

Changed:
- Moved both master and slave target-detection ROI tops to `10 px` above the blue-ground boundary.
- Set `CUT_ROI_Y_OFFSET` to `-10` in both `main.py` and `minimain.py`; the calculation remains `blue boundary + CUT_ROI_Y_OFFSET`.
- Relative to v0.9.4-dev's `3 px` below the boundary, this moves the ROI upward by `13 px`. Relative to stable v0.9.0's `6 px` above the boundary, the net movement is `4 px` upward.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` passed.

### 2026-07-11 - v0.9.4-dev - Slave rollback to the v0.9.1 legacy structure

Scope: `minimain.py`, `README_ch.md`, `README_en.md`

Changed:
- Reverted the v0.9.2 rebuild of `minimain.py` from `main.py`, restoring the field-tested v0.9.0/v0.9.1 slave file structure, command handling, and yellow-line state machine.
- Restored the slave left/right vertical yellow ROIs, bottom-up scan, `yellow_raw_detected`, `YELLOW_CARRY_HOLD_FRAMES = 40`, and the original carry-crossing flow.
- Removed the watchdog, master bottom-corner fitted-line state, master horizontal yellow ROIs, and related helpers introduced by the rebuild.
- Preserved the v0.9.1 performance changes: no per-frame orange-obstacle scan or target-overlap rejection, `obstacle_flag` remains `0`, the target ROI starts `3 px` below the blue-ground boundary, and the normal-color minimum is `150` pixels.
- Preserved the current dual-OpenART-Plus hardware rule; the slave still uses `UART12` at 115200 bps.
- Only `minimain.py` was rolled back; `main.py` was not rolled back with the slave.

Verification:
- `python -m py_compile minimain.py` passed.
- Compared with the v0.9.0 baseline, `minimain.py` now contains only the v0.9.1 performance changes and the current UART12/3 px parameters.
- `git diff --check -- minimain.py README_ch.md README_en.md` passed.

### 2026-07-10 - v0.9.3-dev - Dual Plus UART12 rule

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:
- Documented that both current camera boards are OpenART Plus and both official runtimes unconditionally initialize `UART(12, baudrate=115200)`.
- Removed the obsolete slave-role UART2 branch from `main.py` and the redundant identical branch from `minimain.py`.
- Kept `IS_SLAVE_CAR` only as a software-role switch for candidate reporting and host `0x03` color locking; it no longer affects the UART number.
- Added the UART12 hardware rule to the root README and both detailed READMEs to prevent UART2 from being restored during later maintenance.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- `rg -n "UART\\(2" main.py minimain.py` returned no matches.

### 2026-07-10 - v0.9.2-dev - Slave runtime alignment and freeze-risk cleanup

Scope: `minimain.py`, `README_ch.md`, `README_en.md`

Changed:
- Rebuilt `minimain.py` from the current stable `main.py` baseline so both cars now share command parsing, target search, local tracking, dynamic cropping, yellow-line state handling, return-beacon detection, and main-loop structure.
- Removed the slave's obsolete left/right yellow ROIs, scan strips, old carry-hold counters, raw yellow state, and the associated divergent logic.
- Removed slave startup banners, command-handler prints, and runtime debug prints to avoid offline blocking when stdout is not consumed.
- Added the same `8 s` watchdog, watchdog feeds on every early main-loop exit, and `gc.collect()` every `10` frames to reduce permanent hangs and long-run heap fragmentation.
- Synchronized the current multi-strip blue-ground crop, bottom-corner crossing confirmation, and removal of the retired orange-obstacle path.
- Preserved slave-specific role selection, exposure, color thresholds, blue-ground threshold, yellow threshold, return-beacon thresholds, and IPM calibration values.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- Constant-difference checking confirmed that only the documented device-specific parameters remain different.
- `git diff --check -- main.py minimain.py README_ch.md README_en.md` passed.

### 2026-07-10 - v0.9.1-dev - Frame-rate and carry-crossing optimization

Scope: `main.py`, `minimain.py`, `README_ch.md`, `README_en.md`

Changed:
- Removed the orange LAB threshold, lane-position decision, orange-obstacle detector, and target/obstacle overlap rejection from both official runtimes.
- Search, carry, and return paths no longer scan the `320x160` orange-obstacle ROI every frame. Valid color targets are no longer discarded because they overlap an orange region.
- Kept the `obstacle_flag` byte in the 16-byte return packet for RT1021 parser compatibility; it is now always transmitted as `0`.
- The `0x06` pre-carry scan for other color IDs is unchanged and remains independent of the removed orange-obstacle detector.
- After detecting the dynamic dark-blue ground boundary, the effective target-detection ROI now starts `3 px` below it. Global search, local tracking, and the `0x06` scan all use the tightened ROI.
- The dynamic-cut debug line now shows the effective horizontal ROI boundary for direct field verification.
- The master runtime keeps checking the yellow line every 2 frames until it reaches either bottom corner. Once that state is latched, detection runs every frame and 3 consecutive frame-level misses report the crossing.

Effect:
- Normal target detection and return mode each avoid one large-ROI `find_blobs()` call per frame, while the target-search ROI is also smaller, reducing vision-processing cost and improving effective frame rate.
- The master crossing response after the yellow line reaches a bottom corner is reduced from about 6 main-loop frames to 3 while preserving the bottom-corner prerequisite.

Verification:
- `python -m py_compile main.py minimain.py` passed.
- `git diff --check` passed.

### 2026-07-10 - v0.9.0 - Stable dual-car communication release

Scope: `main.py`, `minimain.py`, `README.md`, `README_ch.md`, `README_en.md`

Changed:
- Added `host_color_id_received` to `main.py`, matching `minimain.py`, so host-issued final color IDs are separated from locally detected candidate colors.
- `main.py` and `minimain.py` no longer write `target_color_id` after local stable detection. Before host command `0x03` arrives, both scripts keep searching all colors and keep reporting candidate color IDs and coordinates to the host.
- After host command `0x03 SET_TARGET_COLOR` arrives, both scripts immediately switch to the corresponding single LAB threshold and clear the current tracking box so the next frame reacquires by the host-selected color.
- After host color lock, temporary target loss clears only local tracking box and ROI state, not the host color ID, preventing fallback to all-color search. Reset/return commands still clear the lock state.
- Updated the concise root `README.md` entrypoint with the stable version, official master/slave scripts, and links to the detailed Chinese and English logs.

Effect:
- Both cars' OpenART modules now have equal discovery roles: either side may report the first candidate color, the RT1021 host link arbitrates and synchronizes the final color, and each host sends it to its OpenART module with `0x03` so both cars track the same color target.
- This release consolidates color detection, SD thresholds, pre-carry scanning, yellow-line crossing, obstacle state, return-beacon handling, and the new dual-car color-lock loop into a deployable stable version.

Verification:
- `python -m py_compile main.py minimain.py main_autocalib_test.py calib_ide_autocalib_competition.py calib_ide_tune.py front_obstacle_scan_test.py cmm_load.py` passed.
- `git diff --check` passed.

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

### 2026-06-21 - v0.7.5-dev - Remove SD logging to prevent offline freezes

Scope: `main.py`, `README_ch.md`, `README_en.md`

Changed:

- Removed the `ENABLE_SD_LOG`, `LOG_PATH`, `LOG_INTERVAL_MS`, and `LOG_FIRST_FRAMES` constants and the `last_log_ms` variable.
- Removed the `log_checkpoint()` function.
- Removed every `log_checkpoint` call at startup, inside `load_white_bear_model()`, and in the main loop (about 25 call sites).
- Kept the watchdog logic (`ENABLE_WATCHDOG`, `WATCHDOG_TIMEOUT_MS`, `init_watchdog()`, and `feed_watchdog()`) unchanged.

Effect:

- Eliminated the risk that forced SD writes during the first 10 frames (about 15 writes per frame) could push a frame beyond 8 seconds and trigger a watchdog reset.
- Removed the risk of the once-per-second runtime SD write blocking the frame loop.
- The main loop now performs no SD I/O, giving it more stable frame timing.

Maintenance rules:

- Do not re-enable SD logging in offline deployment. For field diagnostics, connect the IDE and use `print()` or a separate diagnostic script instead of adding file writes to the main loop.
- If persistent logging is required later, `log_checkpoint` must feed the watchdog and the forced-write window (`LOG_FIRST_FRAMES`) must be reduced to no more than 3 frames.

Verification:

- `python -m py_compile main.py` passed.

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
