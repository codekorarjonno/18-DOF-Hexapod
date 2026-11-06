# Shared steps for install_laptop.sh and install_pi.sh (sourced, not run directly)

install_ros() {   # $1 = ros-jazzy-desktop or ros-jazzy-ros-base
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "== ROS 2 Jazzy already installed"
  else
    echo "== Installing ROS 2 Jazzy ($1)"
    sudo apt update
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe
    local v
    v=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${v}/ros2-apt-source_${v}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update
    sudo apt install -y "$1"
  fi
  sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git rsync
}

link_packages() {  # $1 = folder with our packages, $2 = workspace
  mkdir -p "$2/src"
  for p in "$1"/*/; do
    n=$(basename "$p")
    if [ -e "$2/src/$n" ] && [ ! -L "$2/src/$n" ]; then
      echo "   $n already exists in $2/src (left as it is)"
    else
      ln -sfn "$p" "$2/src/$n"
      echo "   linked $n"
    fi
  done
}

import_and_build() {  # $1 = .repos file, $2 = workspace
  echo "== Downloading third-party packages"
  vcs import --skip-existing "$2/src" < "$1"
  echo "== Installing dependencies (rosdep)"
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
  rosdep update
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths "$2/src" --ignore-src -y --rosdistro jazzy
  echo "== Building (colcon)"
  cd "$2" && colcon build
}

add_to_bashrc() {  # appends a line to ~/.bashrc once
  grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc
}
