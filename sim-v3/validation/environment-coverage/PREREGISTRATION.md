# Figure 2 environmental-effect sweep preregistration

Date frozen: 2026-09-07, before executing `generate-environment-sweep.ts`.

## Fixed design

The sweep uses Vehicle A (`vehicle-a-otter`, `planar3`), seed 7319, the frozen three-leg common task route, 0.05 s physics, 0.1 s control, and a 120 s horizon. The existing `LOS-PID-v2` controller supplies line-of-sight guidance with an 8 m lookahead and clamps requests to the frozen Vehicle A envelope (150 N surge, 100 N·m yaw). The generator additionally checks every request against those limits and records allocator saturation diagnostics. All arms share initial state, route, controller, timing, and seed. Only one environment axis differs from zero at a time.

The current inputs are NOAA CO-OPS predictions for Golden Gate Bridge station SFB1202, bin 1, on 2026-09-08: peak flood 1.556 m/s toward 041°, slack 0.001 m/s, and peak ebb 1.038 m/s toward 213°. The wind arm uses the retained NDBC 46026 observation at 2026-09-07 18:40 UTC: 2.0 m/s from 230° (applied toward 050°).

## Predictions

- Slack water: less than 0.5 m final separation from zero current; route outcome should match the baseline.
- Peak flood: 5–80 m final separation. It initially assists the north/east route and should materially alter completion time or final capture behavior.
- Peak ebb: 5–100 m final separation and likely route-completion failure. Its 1.038 m/s opposing component is comparable to the controller's 1 m/s cruise target.
- Retrieved wind: 0.25–15 m final separation with the same route outcome as calm. Wind should be smaller than at least one peak-current effect.
- Counterfactual interpretation: a directly opposing 2 m/s current exceeds the controller's approximately 1 m/s cruise target, so station holding or forward route progress is not expected. The selected day's NOAA extrema are lower than 2 m/s, but peak flood still exceeds the target speed.

## Decision rule

The environment layer is working as intended if both peak-current arms separate from baseline by at least 5 m, the retrieved-wind arm separates by at least 0.25 m, slack remains below 0.5 m, all effects are finite, and current effects exceed slack. Route failure under a peak current is an acceptable physical result. The generator must fail rather than weaken these thresholds.

The layer is not working or is mis-wired if a nonzero environment arm is numerically indistinguishable from baseline, peak currents do not exceed slack, the wind arm has no measurable effect, command requests exceed the frozen envelope, or the observed ordering/directions are physically incoherent. A failed preregistered assertion requires diagnosis before any scenario or threshold adjustment.

## Gated Step 2 protocol amendment

After the initial sweep halted, the prescribed zero-disturbance isolation run was executed at 600 s. It completed at 156.6 s with the corrected LOS controller and no allocator saturation, establishing that the original 120 s horizon—not a controller fault—caused the route-completion failure. Before rerunning the full sweep, the horizon is therefore extended to 180 s, a rounded 23.4 s (15%) margin beyond observed completion. All effect thresholds above remain unchanged.

Vehicle A planar3 had no active wind force. The follow-up wiring uses the already-unit-tested synthetic coefficient amplitudes `C_X=1`, `C_Y=0.5`, and `C_N=0.25` with cosine/sine angle dependence as an explicitly uncalibrated placeholder. This tests directional dynamic coupling only and does not establish Otter aerodynamic fidelity.
