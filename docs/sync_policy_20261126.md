# November 26 LFA forwarding sync

Merge Carrot 47b1d349c into current origin wip 2e8908ef8. The target was
previously reset remotely to stock upstream; do not resurrect removed custom
commits. No Joongyu or ej integration. Trim only persistent request-ON 0x12A
backlog after 100 qualifying pops without amplifying torque steps; preserve
OFF pulses, other IDs, reserve/prefill/reuse and existing overflow behavior.
Reject disallowed DLC/relay frames before they can mutate the control FIFO.

Validation: native FIFO/timer/cluster hooks 374 pass; fresh host libsafety
piggyback and native ALT2 buttons 54 pass; controller/handover 60 pass and
steering mode 12 pass. The latter required an isolated in-memory Params
put_int/get_int harness because the stock monitoring stub is read-only. No
production harness or behavior changed to pass tests. No physical vehicle
build/flash/driving validation or claim of a complete oscillation cure.

## November 27 CarrotAssist and Mapbox integration

Merge Carrot 5914f0a45 (including Mapbox notice 993b7b2c3) onto current
wip 750db1bbe. Add localized Mapbox usage-charge notices in search/key setup;
repair consecutive assistant driving requests and add one-shot fresh speed
reads with existing permission, expiry, duplicate and blindspot guards. No
Joongyu or ej integration and no change to steering/navigation implementations.

Validation: relay/MCP 35 pass, device permission/HTTP 31 pass, web assistant/
navigation 16 pass; regenerated web assets match the incoming build. Tests use
local loopback/mocks only. No vehicle operations, real CAN or deployment.
