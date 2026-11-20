# November 19 synchronization policy

carrot-dev merges Joongyu bafb821f first, then Carrot 34ce0aa1. User changes take precedence; otherwise Joongyu SPAS/control behavior wins. Keep this policy for future merges.

The new upstream forced HDP bit-188 write is deliberately excluded. Ioniq9ClusterTest retains only dashed-code behavior (0/1 off, 2/3 on), now available to ccNC cars. Keep removed activation/deactivation popups removed. Retain existing ej integration, standard EV4 175-degree limit, persistent CANCEL DM-off, volume minimum zero, Orin NX compatibility, Mochi screensaver, boot-frozen internal model selection and 85-percent C/J temperature row.

Settings catalog has 257 entries, including branch-specific features; update Wiki-generator expectations accordingly. Public Wiki is generated from carrot-wip only. Do not publish this dev-only menu over the public wip catalog. Generate and validate a separate draft from this branch instead.

## November 20 follow-up

Merge Joongyu 67c78a65 lamp trials, then Carrot 8300c95f manual-indicator recovery. Keep forced HDP writes excluded, preserve existing ej/control and user policies. GV80 template byte-18 adjustment is upstream-specific; no device configuration or CAN transmission is performed by synchronization. Settings Wiki draft is validated separately; public Wiki remains carrot-wip-only.

November 20 semantic resolution: preserve Joongyu Turnlight labels, Lab grouping and external-device-only/SPAS-Kick exclusion gates. Adopt compatible shared boundary observation/recovery changes; do not enable native manual-indicator behavior merely from lower-priority catalog prose. Existing ej minimum-duration behavior stays unchanged.

## November 20 afternoon

Merge Joongyu ddeb4677 first: GV80 parking-body trials, session-relative hazard rhythm, restart burst-gap enforcement. Then integrate Carrot ad0943c0 shared optional 0x4B8/0x093 navigation reception. Preserve all existing user/ej/Joongyu resolutions and no device writes.

Navigation conflict resolution: retain Joongyu full-wrapper naming and timestamp checks and optional GV80 SPAS2 registration; add lower-priority length-checked 0x4B8 reception alongside them. Keep Joongyu catalog wording and append only the new alternate-profile capability.

## November 21 upstream CD210 rollback

Merge Carrot 5c970aa5 after already-contained Joongyu ddeb4677. On carrot-wip accept the fixed CD210 model/compiler/ModelState rollback. On carrot-dev user model selection takes precedence: keep Cheddar's exact driving_supercombo.onnx, separate CD210 source/artifact, boot-frozen selection, current default and compiler/runtime dispatch. Exclude fixed CD210 labels and retain the selector/blur; generated web assets remain unchanged because resolved web sources are unchanged. Dispatch new native internal verifier by boot-selected backend. Retain user lane-centering d294b576 as well as existing controls/HDP/HUD/DM/Orin settings. Top-level diagnostic imports and HDA2 topology fixes are independent and accepted.
