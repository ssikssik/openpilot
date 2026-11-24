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

## November 21 evening

Merge Carrot 77dd3335 after already-contained Joongyu ddeb4677. Preserve Joongyu APRK/SPAS implementation and tests: upstream no-reply probe cache assumes a different output-owner lease, so exclude it on carrot-dev, including automatically merged cache references. Adopt compatible shared manual-turn peak recovery (0.3s/5deg, existing rearm gates). Do not describe or expose the lower-priority 60-second cache on dev. Retain user EV4 avoidance extension, midpoint protection, lateral-setting maxima120/80/150, model picker, HUD and DM changes. carrot-wip accepts upstream probe cache and turn recovery.

## November 22 ATC integration

Merge Carrot 839140999 after already-contained Joongyu ddeb4677. Keep Joongyu
SPAS/control and external-device-only admission. Exclude Carrot's new native TBT
permission journal, Kick/DesireHelper promotion and matching catalog wording:
this branch deliberately does not opt into that lower-priority native ownership
path. Accept generic TBT cancellation diagnostics, prior-lamp isolation guard
and distance-window support to 2000m. Keep the user's dev distance selection
valid through 2000m on all roads; do not impose Carrot's new local-road 500m
clamp. carrot-wip accepts the original highway-only extension/native admission.
Preserve model selection, EV4 centering/avoidance/ranges, navigation changes,
DM/HUD/Orin NX and existing user features. No device writes.

Validation: dev ATC/Kick/DesireHelper 432 pass/93 branch-native skips; shared
indicator/Wiki 339 pass/32 branch-native skips; web settings navigation 13 pass.
Wip corresponding groups: 560 pass/2 platform skips, 448 pass and web 13 pass.
The old source-loop 501m-rejection fixture was updated to the retained 2000m
contract, including accepted 501/2000m and rejected 2001m cases. Device build,
physical actuation and driving were not tested.

## November 23 Joongyu GV80 rollback

Merge Joongyu f12389801 (including Carrot 839140999), then verify Carrot is
already contained. Accept the explicit GV80 parking-body trial 3/4 revert:
menu remains 0/1/2, remove park-body generation, DBC byte5/6/7 fields and the
matching Panda exception/tests. Keep existing native manual/TBT admission and
APRK no-reply-cache exclusions: the newly merged cache is described upstream
as inert on Joongyu cars, and retaining isolated initialization/dispatch pieces
without its excluded implementation would be inconsistent. Preserve the user
2000m distance on all roads and the matching descriptions, instead of importing
prose promising native Kick admission or a local-road 500m cap. Preserve manual
observation refinements and alternate-profile descriptions. Navigation parser
ordering is immaterial; keep user 0x363/0x094/slot1/section queue and HUD fixes.
No vehicle writes.

Validation: Hyundai controller/SPAS/steering/cluster 409 passed; native safety,
settings schema, ATC/TBT and Wiki generator group 523 passed. Added a rejection
regression for the retired GV80 parking-active body; focused safety 12 passed.
Host libsafety was compiled from the merged sources. Settings-schema validation
used an isolated temporary HARDWARE import stub, in addition to the existing
monitoring harness, because this macOS environment lacks native hardware imports.
Generated dev-only Wiki draft validated (808 Markdown files); public wip Wiki
is unchanged. No actual device build, physical actuation or driving validation.

## November 23 PV5 cluster display integration

Merge Carrot 6a9225b54 after already-contained Joongyu f12389801. Preserve
fresh original PV5 0x161 display states with the upstream 250ms controller-clock
expiry and source/integrity checks. Keep Joongyu vision lane rendering and
unseen-line policy: do not import the conflicting lower-priority lane_display
gate or its alternate fallback. Place PV5 stock rendering after existing vision
color selection, keeping warning/availability priorities. Retain both CI test
entries. Non-PV5 output remains unchanged, including EV4 ccNC dashed lines.
No changes to navigation, steering limits, DM, models, HUD or Orin NX support.

Validation: 523 controller/SPAS/cluster/steering/lane-crossing tests plus 29
vision-marking tests passed. No device installation, build or driving validation.

## November 25 SPAS counter integration after test repair

Start from current origin d55839234, which aligns the legacy CAN-FD suite
with current Carrot DBC names and existing hook behavior. Merge Joongyu
b8d8c0467: SPAS2 hazard values 1/2 trail stock by one, retain that offset
through zero frames for 200ms, and use +1 for Kick/menu 3..7. Preserve all
previous exclusions and user d50f93145 EV4 low-speed torque/smoothing changes.
No test-repair-driven runtime safety policy changes were made by this sync.

Fresh host libsafety build: CAN-FD native safety and piggyback 692 passed,
245 base/inapplicable skips. Controller/SPAS/EV4 steering 151 passed. Previous
325 failures no longer occur with the user-provided test correction. Passing
this branch-specific suite does not establish stock openpilot safety parity or
physical vehicle validation. No vehicle writes.
