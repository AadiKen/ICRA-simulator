# Combined-damping validation

This gate checks the executable damping composition used by `coupled6`. For heave, roll, and pitch it uses the deterministic free-decay frequency selected by the Capytaine resolver, holds added mass and radiation damping constant at that frequency, and simulates both the production combined model and a diagnostic radiation-only counterfactual.

The gate requires finite deterministic traces, separately provenanced potential-radiation, linear-viscous, and quadratic-viscous terms, nonzero viscous damping, nonnegative sampled dissipated power, and a larger logarithmic decrement with the viscous terms enabled. It reports natural period, logarithmic decrement, damping ratio, settling time, equilibrium error, and trace checksums.

The radiation-only run is never accepted as a production configuration. It exists only to prove that the viscous path is active and materially affects the extracted decay metrics.

## Evidence boundary

Passing this gate is software/model-consistency evidence, not physical validation. The current Vehicle B and C viscous matrices are uncalibrated empirical bootstrap estimates. They have not been fitted to independent free-decay measurements. The bundled Ikeda, Himeno, and Tanaka paper motivates separating frictional, wave-making, eddy-making, lift, and bilge-keel roll-damping components, but the current vessel inputs are insufficient for a complete Ikeda calculation; the artifacts therefore do not claim one.

Physical promotion requires reviewed hull meshes, independent heave/roll/pitch free-decay measurements with coordinate and sensor provenance, and a documented coefficient fit or complete empirical-method calculation over the claimed operating range.

Run `npm run validate:combined-damping` to regenerate both vehicle artifacts and `npm run test:combined-damping` to enforce the gate.
