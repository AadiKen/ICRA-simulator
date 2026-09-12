# Gate 7 Path-Based Conformance Protocol Protocol v2 — adopted

Protocol v2 is adopted for Surveyor on `common-waypoint-transit-v1` only. Its thresholds are post-hoc and must be described that way. The original checkpoint scorer remains available.

Across 30 independent seeds, calm passes 30/30 and disturbed passes 28/30. The two failures are independently observed low-frequency outliers: seed 20005 fails all four path bounds, while seed 20024 fails only cross-track RMS. Both were investigated in good faith for current magnitude, absolute and vessel-relative heading, relative-speed minima, and actuator-envelope exceedance. Neither was traced to a specific mechanism; neither is hidden or removed.

For seed 20024, minimum vessel-current relative speed is 0.002402 m/s at 107.55 s. Sub-0.2 m/s intervals are short and mostly late. Actuator-envelope exceedance is 60/2400 samples (2.5%), entirely yaw-rate driven. During 0–30 s, mean absolute vessel-relative current angle is 92.338 degrees and beam exposure is 19.967%, both ordinary within the calibration batch. No checked mechanism distinguishes it.

Gate 7 authorization is granted for Surveyor VRX policy training under the patched configuration. Patched results remain separate from stock-VRX baseline claims.
