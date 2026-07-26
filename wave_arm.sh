#!/usr/bin/env bash
set -euo pipefail
ros2 topic pub --once /wave_arm std_msgs/msg/Float32 '{data: 1.0}'
