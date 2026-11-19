# November 19 synchronization policy

carrot-dev merges Joongyu bafb821f first, then Carrot 34ce0aa1. User changes take precedence; otherwise Joongyu SPAS/control behavior wins. Keep this policy for future merges.

The new upstream forced HDP bit-188 write is deliberately excluded. Ioniq9ClusterTest retains only dashed-code behavior (0/1 off, 2/3 on), now available to ccNC cars. Keep removed activation/deactivation popups removed. Retain existing ej integration, standard EV4 175-degree limit, persistent CANCEL DM-off, volume minimum zero, Orin NX compatibility, Mochi screensaver, boot-frozen internal model selection and 85-percent C/J temperature row.

Settings catalog has 257 entries, including branch-specific features; update Wiki-generator expectations accordingly. Public Wiki is generated from carrot-wip only. Do not publish this dev-only menu over the public wip catalog. Generate and validate a separate draft from this branch instead.
