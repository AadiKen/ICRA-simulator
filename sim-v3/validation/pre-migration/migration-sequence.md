# Component migration sequence

The migration proceeds one committed component at a time: mass/inertia, Coriolis, damping, hydrostatics, actuators, wind/current, then waves.

After each component, run `npm run test:migration-gate` and retain its metric-delta artifact. A component does not advance when a bit-exact field differs, a tolerance is exceeded, or any underlying suite fails. Thresholds are fixed in `acceptance-policy.json`; they are not adjusted after observing a delta.

The actuator pre-migration surface is bit-exact. Its current report has exactly
zero numerical, structural, and checksum deltas; no nonzero result is classified
as insignificant. Avoid qualifiers such as "meaningful" when reporting this gate.

After the parity-preserving actuator migration passes, integrated actuator
capabilities are a mandatory phase gate—not cleanup. Functional dead zones,
failure transitions, failure events, allocation under a failed actuator, and
dead-zone power accounting must pass `validation/actuator-capability-gate.md`
before actuator-failure benchmark scenarios, Vehicle C integrated allocation
evidence, or the KVLCC2 MMG module may proceed.

The capability gate does not replace or rewrite the actuator legacy golden. A
new capability golden and reviewed legacy-to-capability delta are created through
the narrow policy in `validation/behavior-supersession/`, only after the capability
implementation has its own commit and only while the original checksum verifies.
