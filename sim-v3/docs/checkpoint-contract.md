# Checkpoint serialization contract

Checkpoints are JSON-round-trippable, versioned value records. Runtime class identity is never assumed to survive serialization.

Current restoration inventory:

- `simState`: restored explicitly to `simState`.
- Application vectors reachable from boat, goal, environment, obstacle, and sensor state: restored explicitly to `vec3` when they have finite `x`, `y`, and `z` fields and no quaternion `w` field.
- `RigidBodyState`: restored explicitly; its position, quaternion, velocity, angular-rate, and acceleration members are intentionally plain value objects.
- Actuator effectors: not serialized as class instances. Their command, lag value, thrust, azimuth, axis, deflection, rotor speed, last wrench, and last commands use an explicit DTO restored into the already-constructed effectors.
- Seeded RNG: not serialized as a class instance. Algorithm ID and unsigned 32-bit state are saved explicitly.
- Sensor lifecycle plugins: the production checkpoint contains a versioned sensor-runtime DTO with lifecycle, cadence, per-plugin RNG, bounded published-sample history, and each plugin's explicit state DTO. Plugin instances and service references are never serialized. Restore validates the complete sensor checkpoint before mutating runtime state.
- Matrices: represented as nested numeric arrays and carry no methods or prototypes.

The checkpoint codec preserves `-0`, `Infinity`, and `-Infinity` with tagged values. It rejects `NaN`. The migration gate performs a real JSON stringify/parse between saving and restoring, checkpoints during actuator lag, and verifies both trajectory and RNG continuation bit-for-bit.

## Per-plugin RNG derivation

Each typed sensor owns an independent Mulberry32 stream. Its unsigned 32-bit seed is FNV-1a over the exact byte sequence:

1. UTF-8 bytes of the base-10 experiment seed with no whitespace or sign normalization beyond the language-independent decimal representation (for example, `-12` is bytes `2d 31 32`).
2. One literal NUL byte (`00`).
3. UTF-8 bytes of the stable plugin ID.

FNV-1a starts at offset basis `0x811c9dc5`; for each byte it XORs the byte and multiplies modulo 2^32 by `0x01000193`. This byte layout is part of the checkpoint compatibility contract for non-JavaScript runtimes.

Compact published samples are checkpointed inline only when their deterministic checkpoint-compatible UTF-8 JSON representation is at most 256 KiB. Camera and LiDAR payloads are categorically excluded from Derived inputs and from published-sample checkpoint history, regardless of configured resolution.
