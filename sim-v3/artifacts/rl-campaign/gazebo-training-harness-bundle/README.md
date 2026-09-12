# Gazebo training-harness bundle

This bundle contains the source and compact configuration/evidence files needed
to inspect the Gazebo/VRX common-task training harness. It includes the Python
Gym environments, simulator bridges, task and actuator adapters, episode/model
generation, telemetry export, corrected trace assembly, Gate scripts, training
entry points, and relevant Node dynamics code.

Raw telemetry, logs, model checkpoints, Docker images, compiled plugins, and
unrelated campaign outputs are excluded. The included Gate 7 scorer reconstructs
planar twist from pose with wrapped yaw differences instead of consuming the
known-contaminated Gazebo rolling-window odometry twist.
