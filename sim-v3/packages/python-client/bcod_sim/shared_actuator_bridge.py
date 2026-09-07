"""Python client for the authoritative TypeScript Surveyor actuator bank."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess

import numpy as np


class SharedActuatorBridge:
    def __init__(self, repository: str | Path):
        self.root = Path(repository)
        script = self.root / "validation/rl-campaign/ports/actuator-jsonl-bridge.ts"
        self.process = subprocess.Popen(
            ["node", "--experimental-strip-types", str(script)], cwd=self.root,
            text=True, bufsize=1, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.reset()

    def _request(self, value: dict) -> np.ndarray:
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(json.dumps(value, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            error = self.process.stderr.read() if self.process.stderr else ""
            raise RuntimeError(f"actuator bridge stopped: {error}")
        response = json.loads(line)
        if not response.get("ok"):
            raise RuntimeError(str(response.get("error")))
        return np.asarray(response.get("thrust_newtons", [0.0, 0.0]), dtype=float)

    def reset(self) -> np.ndarray:
        return self._request({"op": "reset"})

    def step(self, action, dt_s: float) -> np.ndarray:
        active = np.clip(np.asarray(action, dtype=float), -1, 1)
        return self._request({"op": "step", "action": [float(active[0]), float(active[1]), 0.0, 0.0], "dt_s": dt_s})

    def close(self) -> None:
        if self.process.poll() is None:
            self._request({"op": "close"})
            if self.process.stdin is not None:
                self.process.stdin.close()
            self.process.wait(timeout=5)
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            if stream is not None:
                stream.close()
