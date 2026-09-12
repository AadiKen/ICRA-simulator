from __future__ import annotations
import numpy as np


def blind_observation(observation: np.ndarray) -> np.ndarray:
    """Zero environmental forcing (9:15) and tracks (15:79) in every frame."""
    result = np.asarray(observation, dtype=np.float32).copy()
    if result.shape != (320,): raise ValueError("expected a 320-value stacked observation")
    for offset in range(0, 320, 80): result[offset+9:offset+79] = 0.0
    return result
