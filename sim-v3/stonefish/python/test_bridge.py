from __future__ import annotations

import argparse
import math
from pathlib import Path

from stonefish_bridge import StonefishBridge


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--stonefish-lib", type=Path, required=True)
    parser.add_argument("--deps-lib", type=Path, required=True)
    args = parser.parse_args()

    with StonefishBridge(
        args.executable,
        args.data_dir,
        library_dirs=(args.stonefish_lib, args.deps_lib),
    ) as bridge:
        initial = bridge.reset(12345)
        assert set(initial["observation"]) == {
            "gps",
            "fix_valid",
            "imu",
            "compass",
        }
        assert initial["observation"]["fix_valid"]
        assert len(initial["observation"]["gps"]) == 4
        assert len(initial["observation"]["imu"]) == 9
        assert len(initial["observation"]["compass"]) == 1

        straight = bridge.step(0.5, 0.5, physics_steps=250)
        assert straight["observation"]["fix_valid"]
        assert math.isclose(straight["simulation_time"], 0.5, abs_tol=1e-9)
        differential = bridge.step(-1.0, 1.0, physics_steps=500)
        assert abs(differential["observation"]["compass"][0]) > 1e-4
        assert differential["diagnostics"]["port_thrust"] < 0
        assert differential["diagnostics"]["starboard_thrust"] > 0

        submerged = bridge.reset(12345, gps_z_ned=0.5)
        submerged = bridge.step(0.0, 0.0, physics_steps=1)
        assert submerged["observation"]["fix_valid"] is False
        assert submerged["observation"]["gps"][2:] == [0, 0]
        assert submerged["observation"]["gps"][0] > 90
        assert submerged["observation"]["gps"][1] > 180

        print("Gate A bridge verification passed")


if __name__ == "__main__":
    main()
