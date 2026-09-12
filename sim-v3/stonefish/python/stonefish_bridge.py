"""Process-isolated Python client for the Stonefish Vehicle A bridge."""

from __future__ import annotations

import json
import math
import os
import subprocess
from pathlib import Path
from typing import Any


class StonefishBridge:
    """Own one headless Stonefish process and expose reset/step primitives."""

    def __init__(
        self,
        executable: str | os.PathLike[str],
        data_dir: str | os.PathLike[str],
        *,
        library_dirs: tuple[str | os.PathLike[str], ...] = (),
        physics_threads: int | None = 1,
        sensor_noise: bool = False,
    ) -> None:
        env = os.environ.copy()
        env.pop("DISPLAY", None)
        env.pop("WAYLAND_DISPLAY", None)
        env["CUDA_VISIBLE_DEVICES"] = ""
        paths = [str(Path(path)) for path in library_dirs]
        if env.get("LD_LIBRARY_PATH"):
            paths.append(env["LD_LIBRARY_PATH"])
        if paths:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(paths)
        thread_argument = 0 if physics_threads is None else physics_threads
        if thread_argument < 0:
            raise ValueError("physics_threads must be positive or None")
        self._process = subprocess.Popen(
            [str(Path(executable)), str(Path(data_dir)), str(thread_argument),
             "1" if sensor_noise else "0"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env,
        )

    def _request(self, command: str) -> dict[str, Any]:
        if self._process.poll() is not None:
            stderr = self._process.stderr.read() if self._process.stderr else ""
            raise RuntimeError(
                f"Stonefish bridge exited with {self._process.returncode}: {stderr}"
            )
        assert self._process.stdin is not None
        assert self._process.stdout is not None
        self._process.stdin.write(command + "\n")
        self._process.stdin.flush()
        while True:
            line = self._process.stdout.readline()
            if not line:
                raise RuntimeError("Stonefish bridge closed stdout")
            if line.startswith("{"):
                response = json.loads(line)
                if not response.get("ok"):
                    raise RuntimeError(response.get("error", "bridge request failed"))
                return response

    def reset(
        self,
        seed: int,
        *,
        gps_z_ned: float = -0.5,
        scenario: str = "normal",
        initial_north_m: float = 0.0,
        initial_east_m: float = 0.0,
        initial_yaw_rad: float = 0.0,
        current_ned_mps: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> dict[str, Any]:
        if not 0 <= seed <= 0xFFFFFFFF:
            raise ValueError("seed must fit in uint32")
        if scenario not in {"normal", "grounding", "object_collision"}:
            raise ValueError("unknown scenario")
        pose_and_current = (
            initial_north_m, initial_east_m, initial_yaw_rad, *current_ned_mps
        )
        if len(current_ned_mps) != 3 or not all(
            math.isfinite(float(value)) for value in pose_and_current
        ):
            raise ValueError("initial pose and current must be finite 3-vectors")
        return self._request(
            f"RESET {seed} {gps_z_ned:.17g} {scenario} "
            f"{initial_north_m:.17g} {initial_east_m:.17g} {initial_yaw_rad:.17g} "
            f"{current_ned_mps[0]:.17g} {current_ned_mps[1]:.17g} "
            f"{current_ned_mps[2]:.17g}"
        )

    def step(
        self, port: float, starboard: float, *, physics_steps: int = 1
    ) -> dict[str, Any]:
        if physics_steps <= 0:
            raise ValueError("physics_steps must be positive")
        return self._request(
            f"STEP {port:.17g} {starboard:.17g} {physics_steps}"
        )

    def close(self) -> None:
        if self._process.poll() is not None:
            return
        try:
            self._request("QUIT")
        finally:
            self._process.terminate()
            self._process.wait(timeout=5)

    def __enter__(self) -> "StonefishBridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
