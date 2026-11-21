# Lane centering correction

Configure this in `ej > Lane Centering` on carrot-dev. In laneless mode, it adds curvature toward the midpoint of the two model lane lines when the model path deviates. It works alongside the existing boundary clamp; final lateral acceleration, jerk, curvature and steering-angle limits remain unchanged.

| Setting | Default | Range |
|---|---|---|
| `LaneCenterEnable` | On | Off / On |
| `LaneCenterThreshold` | 20 cm | 5–50 cm |
| `LaneCenterGain` | 10 = 1.0x | 0–20 = 0–2.0x |

A lower threshold starts correction for smaller path offsets. Higher gain increases correction; zero disables it. Saved changes are read by the control loop without rebooting. Correction starts above the threshold and ends below half of it. Offset means model-path displacement from model lane centre, not a direct measurement of the vehicle lateral position.

Both lane probabilities must exceed 0.5 to enter and drop below 0.3 to release. There is no correction at or below 5 km/h and a gradual fade-in to 10 km/h. Blinkers, lane-change state or automatic-turn signals stop correction, followed by a two-second holdoff. Off, zero gain, missing lanes and invalid inputs clear the centering output. Lane-full and the separate VW MEB branch do not use this corrector.

Lookahead is one second, bounded to 5–30 metres. Additional correction is capped at 1.5 m/s² and 0.02 m⁻¹, with an admitted correction slew limit of 0.01 m⁻¹/s. This does not provide steering when lane lines are missing. Physical driving performance has not been validated.

## Abrupt midpoint protection

A camera-frame history of roughly 0.32 seconds detects abrupt lane-midpoint/width changes without a corresponding model-path change. Repeated control cycles for one camera frame are not counted as new samples. Centering pauses for at least one second and resumes only after at least 0.2 seconds of stable recent width, relative centre and confidence. Suppression clears the added correction immediately; existing final target smoothing remains.

## Lane correction during obstacle avoidance

`LaneAvoidanceMode` is a K8 Hybrid trial. EV4 does not construct this avoidance detection/application path. `0` is off, default `1` logs candidates only, and `2` applies experimentally. Mode 2 additionally requires ClampEnable and reduces only laneless LaneCenter correction. Lane-mode planning, the existing clamp and final steering limits remain unchanged.

Front/corner radar must corroborate a stopped or slow roadside vehicle and the existing model path must already provide passing space inside the same lane. Only correction pulling toward that obstacle is attenuated; this does not generate an avoidance path. Detection operates at about 18–64.8km/h. Driver steering/pedals, blinkers, lane changes, automatic turns, surrounding traffic/blind spots, stale or invalid inputs restrict detection/application. Default logging does not alter steering. Physical driving is unvalidated; apply mode is for a controlled test environment.
