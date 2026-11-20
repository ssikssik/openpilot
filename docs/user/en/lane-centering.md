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
