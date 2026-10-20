# Audio assets

## `engine.esc`

The preset for the streaming engine model.

- Source: **enginesound** by DasEtwas, `src/default.esc`:
  <https://github.com/DasEtwas/enginesound>
- License: **MIT** (the full text is in `ENGINESOUND-LICENSE`).
- Commit: `e5fcca587397c0c8ba9c9d24874b951fed74d260`.
- Used by `view/src/audio.rs`, which deserialises it with `ron` and streams a
  `Generator` at an RPM driven by the car's speed and throttle. The viewer does
  not run the enginesound GUI or CLI.

## `boost.wav`

A real thruster recording, used as the boosted-car loop.

- Source: the project owner's `boost2.wav` (44.1 kHz, 24-bit, stereo,
  27.85 s). **License/provenance not yet recorded** — fill this in before
  publishing the repository.
- Edit: the steady section from **2.0 s to 4.5 s** was cut and the last
  200 ms crossfaded into the continuation, producing a seamless **2.5 s**
  loop. DC removed and RMS-normalised to 0.22 (peak 0.59).
- The clip is embedded with `include_bytes!` in `view/src/audio.rs`, so the
  viewer does not depend on the working directory or an asset-loading step.
- The source `boost2.wav` is not included in the repo; only the processed loop
  is shipped.

The car/ball impact, ball-bounce and demolition sounds are synthesised in
`view/src/audio.rs`; there is no recording for them in the repo.
