#!/usr/bin/env bash
# One-command setup of the LAPTOP side: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
#   git clone https://github.com/codekorarjonno/18-DOF-Hexapod.git
#   bash 18-DOF-Hexapod/scripts/install_laptop.sh
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$HOME/ros2_ws"
source "$REPO/scripts/common.sh"

install_ros ros-jazzy-desktop
sudo apt install -y ros-jazzy-ros-gz        # Gazebo Harmonic + ROS bridge

echo "== Linking the hexapod packages into $WS/src"
link_packages "$REPO/ros2_ws/src" "$WS"
import_and_build "$REPO/hexapod.repos" "$WS"

echo "== Shortcuts (hexcon, hexgz, hexgznav ...)"
cp "$REPO/scripts/laptop_aliases.sh" ~/.hexapod_aliases.sh
add_to_bashrc "source /opt/ros/jazzy/setup.bash"
add_to_bashrc "source $WS/install/setup.bash"
add_to_bashrc "export ROS_DOMAIN_ID=42"
add_to_bashrc "source ~/.hexapod_aliases.sh"
mkdir -p ~/maps && cp -n "$REPO"/maps/* ~/maps/ 2>/dev/null || true

echo
echo "Done. Open a NEW terminal, then try:  hexgz   (Gazebo)   or   hexcon"
