# Tensor backend

`VehicleAPlanar3TensorBackend` is a separate PyTorch reproduction of the Node
Vehicle A planar3 reference plant. Each batch environment owns an independent
Mulberry32 stream seeded with FNV-1a-32 over these exact bytes:

```text
UTF-8(experimentSeed) + 0x00 + UTF-8("environment:") + UTF-8(decimalEnvironmentIndex)
```

RNG values are stored in signed 64-bit tensors while every operation is masked
to 32 bits. Random draws are explicit and deterministic physics does not consume
the stream.

Future distributed scheduling should derive the final component from a stable
run/environment ID rather than a transient device slot. That will preserve a
run's stream when workers repack environments across devices. Distributed
scheduling and that alternative seed contract are intentionally not implemented
yet.
