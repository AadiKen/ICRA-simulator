# The generic osrf/ros:jazzy-simulation candidate does not contain `gz`; use
# VRX's Jazzy + Gazebo Harmonic development base, pinned for linux/amd64.
FROM --platform=linux/amd64 ghcr.io/osrf/vrx-devel@sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4 AS builder

ARG VRX_COMMIT=fda35961463c2ed8c44d6c646fe8ce773a3eaff1

WORKDIR /opt/vrx_ws

RUN mkdir -p src/vrx \
    && git -C src/vrx init \
    && git -C src/vrx remote add origin https://github.com/osrf/vrx.git \
    && git -C src/vrx fetch --depth 1 origin "${VRX_COMMIT}" \
    && git -C src/vrx checkout --detach FETCH_HEAD \
    && test "$(git -C src/vrx rev-parse HEAD)" = "${VRX_COMMIT}" \
    && bash -lc 'source /opt/ros/jazzy/setup.bash \
        && rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
        && colcon build --merge-install'

FROM --platform=linux/amd64 ghcr.io/osrf/vrx-devel@sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4

WORKDIR /opt/vrx_ws
COPY --from=builder /opt/vrx_ws/install /opt/vrx_ws/install

# Explicitly source both the Jazzy installation and source-built overlay.
ENTRYPOINT []
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /opt/vrx_ws/install/setup.bash && exec gz sim -s --version"]
