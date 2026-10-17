# JetPack 7.2.1 host experiments from carrot-nav

This is source compatibility support, not an OS upgrade or a flashed NX image.
Install JetPack 7.2.1 separately on the Jetson. The checked runtime is aarch64,
L4T 39.2.1 and TensorRT 10.16.2. Install requirements-jetson-jp721.txt in a
Python 3.12 virtual environment with system site packages so NVIDIA's TensorRT
is visible. Existing JetPack 6 requirements and the signed official release
selection remain the default.

For a reviewed local experiment, run from this repository:

```sh
python3 tools/jetlink/configure_runtime_profile.py --runtime-profile jetpack-7.2.1 --profile nx8-jp721
```

Commit the generated runtime_compatibility.json and local_host_release.json
before exporting a host bundle using the repository's existing bundle tooling.
Keep the selected carrot-nav ONNX; a Cinque v2 model from another branch is not
an interchangeable replacement. Pass the exported bundle and verified ONNX to
prepare_local_experiment.py, then register that package with
install_local_experiment.py against the stopped, protected DATA runtime.
This registration requires the existing protected storage/bootstrap layout;
it is not a fresh OS installer. Every installation/activation and boot selection
checks the installed ABI. A local profile never downloads executable code from
the USB peer. Official NAS releases still require the existing signing key.

TensorRT engines must be rebuilt on the target Jetson. Before vehicle use,
validate model loading, synthetic inference, USB reconnect, navigation state,
thermal behavior and cold boot on the NX. No JP7.2.1 hardware validation has
been performed as part of this source change.
