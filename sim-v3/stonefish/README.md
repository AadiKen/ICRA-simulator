# Stonefish Vehicle A bridge

This directory contains the Gate A, process-isolated bridge for Stonefish
1.6.0. It intentionally exposes only GPS, IMU, and Compass observations.

On CodeNimbus, build after Gate 0 with:

```sh
bash "$HOME/ICRA-simulator/sim-v3/stonefish/build_on_codenimbus.sh" \
  "$HOME/ICRA-simulator/sim-v3/stonefish"
```

Run the end-to-end verification on an allocated node with:

```sh
cd "$HOME/ICRA-simulator/sim-v3/stonefish/python"
python3 test_bridge.py \
  --executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --deps-lib "$HOME/stonefish-deps/lib"
```

Gate B contract mapping and diagnostic:

```bash
python3 test_gate_b.py \
  --executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --deps-lib "$HOME/stonefish-deps/lib" \
  --contract ../../artifacts/rl-campaign/surveyor/task-contract-frozen.json \
  --output ../gate_b_results.json
```

Gate C dynamics, determinism, and termination diagnostic:

```bash
python3 test_gate_c.py \
  --executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --deps-lib "$HOME/stonefish-deps/lib" \
  --output ../gate_c_results.json
```

Gate D uses NumPy because the required shared reward implementation uses it:

```bash
python3 -m venv "$HOME/stonefish-python-env"
"$HOME/stonefish-python-env/bin/pip" install -r ../requirements.txt
```

Run throughput and pre-training validation from this `python` directory:

```bash
python3 benchmark_gate_d.py \
  --executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --deps-lib "$HOME/stonefish-deps/lib" \
  --output ../gate_d_throughput.json \
  --parallel 1 2 4 8 16 32 64 --aggregate-steps 4096

"$HOME/stonefish-python-env/bin/python" validate_gate_d.py \
  --executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --deps-lib "$HOME/stonefish-deps/lib" \
  --contract ../../artifacts/rl-campaign/surveyor/task-contract-frozen.json \
  --output ../gate_d_validation.json
```

`StonefishBridge` starts one headless simulator subprocess. `reset(seed)`
rebuilds the scenario and seeds Stonefish's process-global sensor RNG. `step`
accepts normalized port/starboard commands and an explicit physics-step count.
The returned `observation` has exactly `gps`, `fix_valid`, `imu`, and `compass`;
actuator values are kept in a separate diagnostics object.
