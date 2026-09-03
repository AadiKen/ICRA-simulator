#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/vrx_ws/install/setup.bash
set -u

episode_dir=${1:?episode directory required}
raw_output=${2:?raw output path required}
world_name=surveyor_vrx

cleanup() {
  kill "${runtime_pid:-}" "${bridge_pid:-}" "${sim_pid:-}" 2>/dev/null || true
  wait "${runtime_pid:-}" "${bridge_pid:-}" "${sim_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

gz sim -s "${episode_dir}/world.sdf" >"${episode_dir}/gz.log" 2>&1 &
sim_pid=$!

ros2 run ros_gz_bridge parameter_bridge \
  '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat' \
  '/surveyor/thrusters/port/thrust@std_msgs/msg/Float64]gz.msgs.Double' \
  '/surveyor/thrusters/starboard/thrust@std_msgs/msg/Float64]gz.msgs.Double' \
  >"${episode_dir}/bridge.log" 2>&1 &
bridge_pid=$!

python3 /ports/vrx_gate7_runtime.py "${episode_dir}/transport.json" "${raw_output}" \
  >"${episode_dir}/runtime.log" 2>&1 &
runtime_pid=$!

for _ in $(seq 1 30); do
  if gz service -l | grep -q "/world/${world_name}/control"; then
    break
  fi
  sleep 1
done

# Allow transport discovery while physics remains paused, then start the
# deterministic episode only after publishers, subscribers, and capture exist.
sleep 2
gz service -s "/world/${world_name}/control" \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'pause: false' >/dev/null

wait "${runtime_pid}"
runtime_pid=
