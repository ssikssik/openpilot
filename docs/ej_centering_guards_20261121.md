# ej centering guard follow-up

Port b09d9b20, 877bb0fe and fbf994a0 from aabc88 ej through e0d38dc9 to carrot-dev only. Exclude unrelated K8 parking assets/renderer changes. Midpoint history protection applies to LaneCenter generally. Preserve d294b576 immediate-output clearing, finite-input checks and existing user changes. Adapt upstream suppression regression to require immediate zero rather than a slew-down tail.

Keep LaneAvoidanceGuard construction restricted to KIA_K8_HEV_1ST_GEN, default mode1 log-only. Do not extend the vehicle allowlist or enable mode2 for EV4. Preserve existing lane-mode planning; transfer only diagnostic candidate data through additive LateralPlan schema fields. Existing model paths, clamp and final lateral/angle/jerk limits remain unchanged. No vehicle installation, Params writes or CAN transmission.

The new guard reads radar observations but does not change shared radar detection, lead selection or radar_motion code. Do not deploy a dev-only control experiment to the carrot-wip NAS replay service. Wiki draft generation remains branch-scoped; the public Wiki source stays carrot-wip.
