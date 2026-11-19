The internal Cheddar policy uses the unmodified tinygrad source archive at
`ec91b74b5b6b0fc924b72d3f27689679ca2438b9`, the revision pinned by upstream
openpilot's Cheddar merge. `cheddar_runtime.py` verifies its SHA-256 before
extracting it into the ignored model build directory. The archive includes
tinygrad's MIT license and upstream sources.

The compiler and internal policy worker select this runtime before importing
tinygrad. Driver monitoring and legacy external-model compilation continue to
use `tinygrad_repo`; downloaded external models retain their own runtime pins.
This keeps serialized artifacts and runtime APIs paired without replacing the
runtime used by unrelated inference processes.

The boot-frozen internal selection can instead use the historical CD210 ONNX
(`models/driving_cd210.onnx`) with the original `tinygrad_repo` compiler and
ModelState. Its artifact is `driving_cd210_tinygrad.pkl`; Cheddar keeps
`driving_tinygrad.pkl` and the isolated runtime above. Saved choices apply at
the next real boot and do not change recovery within an active boot.
