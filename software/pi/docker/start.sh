#!/usr/bin/env bash
# Εκκίνηση του arm_pi_driver μέσα στο container arm-pi (το καλεί το compose.yaml).
# Χτίζει το πακέτο σε φάκελο του container (/root/ws), όχι μέσα στο repo, και ξεκινά το node.
#   ARM_PI_FAKE=true  → προσομοίωση, χωρίς υλικό (docker compose: ARM_PI_FAKE=true docker compose up -d)
set -euo pipefail

REPO=/root/master-s-thesis
source /opt/ros/jazzy/setup.bash
mkdir -p /root/ws && cd /root/ws
colcon --log-base /root/ws/log build \
    --base-paths "$REPO/software/ros2_ws/src" "$REPO/software/pi" \
    --packages-up-to arm_pi_driver
source /root/ws/install/setup.bash
exec ros2 run arm_pi_driver driver --ros-args -p fake:="${ARM_PI_FAKE:-false}"
