#!/usr/bin/env bash
# One-command setup of the RASPBERRY PI 5 side: Ubuntu 24.04 + ROS 2 Jazzy (no desktop),
# RPLIDAR C1, rf2o, Pi camera (libcamera + camera_ros), the hexapod packages.
#   git clone https://github.com/codekorarjonno/18-DOF-Hexapod.git
#   bash 18-DOF-Hexapod/scripts/install_pi.sh
# Building libcamera on the Pi takes a while (about 20-30 min).
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$HOME/ros2_ws"
source "$REPO/scripts/common.sh"

install_ros ros-jazzy-ros-base
sudo apt install -y python3-colcon-meson python3-jinja2 python3-ply python3-yaml \
                    libyaml-dev libgnutls28-dev libssl-dev tmux

echo "== Linking the hexapod packages into $WS/src"
link_packages "$REPO/raspberry_pi/ros2_ws/src" "$WS"
import_and_build "$REPO/raspberry_pi/hexapod_pi.repos" "$WS"

echo "== Serial / USB access for the ESP32 and the lidar"
sudo usermod -aG dialout "$USER"

echo "== Shortcuts (hexapod, hexmap, hexnav, hexstream ...)"
grep -E '^(alias hex|export ROS_DOMAIN_ID)' "$REPO/raspberry_pi/scripts/pi_bashrc.sh" > ~/.hexapod_aliases.sh
[ -f "$REPO/raspberry_pi/scripts/pi_bash_aliases.sh" ] && \
  grep -E '^alias hex' "$REPO/raspberry_pi/scripts/pi_bash_aliases.sh" >> ~/.hexapod_aliases.sh
cp -n "$REPO/raspberry_pi/scripts/hexrun.sh" ~/hexrun.sh 2>/dev/null || true
add_to_bashrc "source /opt/ros/jazzy/setup.bash"
add_to_bashrc "source $WS/install/setup.bash"
add_to_bashrc "export ROS_DOMAIN_ID=42"
add_to_bashrc "source ~/.hexapod_aliases.sh"
mkdir -p ~/maps && cp -n "$REPO"/raspberry_pi/maps/* ~/maps/ 2>/dev/null || true

echo
echo "Done. Log out and back in (for serial access), then try:  hexmap"
