# Sensor artifact policy

Resolved experiments default to `outputs.sensor_artifacts.mode: summary`. State traces and run summaries contain sensor sample counts, valid fractions, dropout counts, latency statistics, saturation/degradation time, accuracy metrics when ground truth is authorized, and sensor energy. Raw camera, LiDAR, radar, and other payloads are not embedded in the state trace.

Raw output is opt-in with either `selected-raw` plus explicit plugin IDs or `all-raw`. Raw payloads are stored as partitioned sensor artifacts (Parquet metadata plus binary payloads where appropriate), referenced from the run manifest, and subject to `max_bytes_per_run` (1 GB by default). Reaching the cap records a truncation event and preserves summaries; it must not silently discard data.

This keeps benchmark and anonymous-release runs bounded while allowing experiments that genuinely require raw observations to request and account for them explicitly.
