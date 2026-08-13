# Reproducible environment manifest

## Supported local toolchain

- Node.js 24.13.x and npm 11.6.x (`.nvmrc` pins Node).
- CPython 3.12.x managed through `uv` and `pyproject.toml`.
- Gazebo Sim 8.14.0 (Harmonic).
- Apple MPS is the local tensor accelerator.

## Gated toolchains

- ROS 2 must be installed from a supported ROS distribution and must provide `rclpy`; it is intentionally not resolved from PyPI.
- CUDA acceptance requires a separate pinned NVIDIA runner.
- NOAA/USGS live provider acceptance requires network access; released runs consume immutable cached inputs.
- Capytaine inputs and outputs must include mesh, frequency grid, version, and SHA-256 provenance.
- SIMMAN and AERO4River datasets are not substituted with generated fixtures for evidence runs.

Gazebo was repaired on 2026-07-31 by reinstalling Homebrew `jpeg-xl` and `gdal`; `gz sim --version` reports 8.14.0.
