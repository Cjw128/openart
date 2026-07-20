# Pure LAB fast fallback

This directory keeps a model-free fallback for the two OpenART Plus cameras.

- `main.py`: master-camera runtime; deploy as `/sd/main.py` on the master.
- `minimain.py`: slave-camera runtime; deploy as `/sd/main.py` on the slave.
- `source_7a29288.zip`: exact, unmodified Git archive used as the baseline.

## Provenance

The runtime baseline is commit `7a292887aaef899b254cca5fadb4333576a34c99`.
It descends from the `4a81d6a` offline hot-path optimization and contains no
`tf` import, `.tflite` model, or model inference. Target acquisition and
tracking use LAB `find_blobs()` only.

The later baseline was selected because it retains the optimized pure-blob
path while already including the tested return-to-base yellow-line and stop
detection added by commits `1983698` and `7a29288`.

## Backported competition state

The working `main.py` and `minimain.py` synchronize the current competition
control rules while retaining pure LAB detection:

- Startup exposure is read independently from the `exposure_us=` line in
  `/sd/color_thr.txt`, clamped to `100..4500 us`, with `1400 us` as fallback.
- After startup or `AA 55 08 08`, normal acquisition and `0x03,id` commands
  accept only ID2. Once ID2 is completed, IDs 1, 3, 4, and 5 open together.
- IDs 1 through 5 are excluded from normal acquisition after completion.
- The master marks an ID when carry mode first confirms `POS_CROSSED`.
- The slave remembers the ID on `0x01` and marks it on the following
  `0x00` or `0x02` completion command.
- `AA 55 08 08` clears the completion mask, pending carry state, target lock,
  front-scan state, and return state.
- Re-sending the same valid `0x03,id` preserves the current target tracking.
  A completed or priority-blocked ID clears the active target and remains
  excluded until it is eligible again.
- Repeated `0x02` packets reset target tracking only once in the same search
  cycle, so a continuously transmitted ready command cannot erase a newly
  acquired target every frame. `0x00`, `0x01`, `0x07`, and `0x08` start a new
  reset cycle.
- `0x06` remains an unrestricted all-color obstacle scan and intentionally
  ignores the ID2-first pickup gate.

Return mode remains compatible with the current protocol: `AA 55 07 07`
enters return tracking and packet ID `0xC8` reports yellow-line position and
the stop request. The single vertical ROI is searched bottom-to-top, and the
stop bit is asserted only while the selected center has `y > 200`. Both
cameras use the current competition yellow LAB threshold
`(27, 100, -55, 16, 21, 105)`.

These files are a fallback, not replacements for the model-led runtime until
they have been exercised on both boards with the competition thresholds,
UART command sequence, five-ID carry cycle, and return path.
