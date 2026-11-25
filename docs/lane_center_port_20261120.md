# Lane-centering port, carrot-dev

Source: aabc88/openpilot ej b2f40a0d, LaneCenterCorrector and its three settings. Scope: carrot-dev only. Preserve this user-requested feature during later upstream syncs. Existing ClampSteerCorrector, Joongyu control/lamps, EV4 175-degree limit, model selector and C/J HUD remain unchanged.

The model-path midpoint correction is added only in the laneless branch, before existing smoothing and final clip_curvature. Uses existing lane-change/blinker/ATC guards. Compared with ej, immediately clears the new correction on disabled/zero-gain/holdoff/invalid inputs instead of slewing a stale correction. Explicitly rejects non-finite lane probabilities and vehicle speed. No physical vehicle installation, Params writes or CAN output.

Three persistent Params use ej defaults (on,20cm,gain10). KO/EN guides document actual model-path offset, bounds, suppression and validation limits. Generate branch-specific Wiki drafts; do not overwrite the public carrot-wip catalog with dev-only settings.

Existing publish test fixture was stale: it used clamp_steer_active while production publish reads clamp_steer_corrector.alert_active and latActive. Align only the test fixture with unchanged production publish logic.
