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
