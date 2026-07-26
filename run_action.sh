#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 ACTION"
  echo "Actions: greet_nod salute fist_greeting clap heart welcome point_left point_right wipe_sweat scratch_head arm_dance bow_wave greeting_combo"
  exit 2
fi

case "$1" in
  greet_nod|salute|fist_greeting|clap|heart|welcome|point_left|point_right|wipe_sweat|scratch_head|arm_dance|bow_wave|greeting_combo)
    ;;
  *)
    echo "Unknown action: $1" >&2
    exit 2
    ;;
esac

ros2 topic pub --once "/$1" std_msgs/msg/Float32 '{data: 1.0}'
