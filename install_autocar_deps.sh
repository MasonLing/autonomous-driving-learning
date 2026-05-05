#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="humble"
OSQP_VERSION="v1.0.0"
OSQP_EIGEN_VERSION="v0.10.0"
SRC_DIR="${SRC_DIR:-$HOME/autocar_install_src}"

log() {
  printf '\n==> %s\n' "$*"
}

require_ubuntu_2204() {
  if [[ ! -r /etc/os-release ]]; then
    echo "Cannot read /etc/os-release" >&2
    exit 1
  fi

  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "This script targets Ubuntu 22.04. Current system: ${PRETTY_NAME:-unknown}" >&2
    exit 1
  fi
}

ensure_bashrc_line() {
  local line="$1"
  local file="$HOME/.bashrc"
  touch "$file"
  if ! grep -qxF "$line" "$file"; then
    printf '\n%s\n' "$line" >> "$file"
  fi
}

clone_or_update_tag() {
  local repo_url="$1"
  local tag="$2"
  local dir="$3"

  if [[ ! -d "$dir/.git" ]]; then
    git clone --recursive -b "$tag" "$repo_url" "$dir"
  else
    git -C "$dir" fetch --tags
    git -C "$dir" checkout "$tag"
    git -C "$dir" submodule update --init --recursive
  fi
}

main() {
  require_ubuntu_2204

  log "Request sudo credentials"
  sudo -v

  log "Enable Ubuntu Universe repository"
  sudo apt update
  sudo apt install -y software-properties-common curl git build-essential gcc g++ make pkg-config locales
  sudo add-apt-repository -y universe

  log "Configure ROS 2 apt source"
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  # shellcheck disable=SC1091
  . /etc/os-release
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  log "Configure UTF-8 locale"
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  export LC_ALL=en_US.UTF-8

  log "Install base tools and common libraries"
  sudo apt update
  sudo apt install -y \
    cmake \
    python3 \
    python3-pip \
    terminator \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-matplotlib

  log "Install ROS 2 Humble and ROS tools"
  sudo apt update
  sudo apt upgrade -y
  sudo apt install -y \
    ros-${ROS_DISTRO_NAME}-desktop \
    ros-${ROS_DISTRO_NAME}-turtlesim \
    ros-${ROS_DISTRO_NAME}-tf2-tools \
    ros-${ROS_DISTRO_NAME}-tf-transformations \
    ros-${ROS_DISTRO_NAME}-joint-state-publisher \
    ros-${ROS_DISTRO_NAME}-joint-state-publisher-gui \
    ros-${ROS_DISTRO_NAME}-xacro

  ensure_bashrc_line "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"

  log "Install Python package transforms3d"
  python3 -m pip install --user --upgrade transforms3d

  log "Build and install OSQP ${OSQP_VERSION}"
  mkdir -p "$SRC_DIR"
  clone_or_update_tag "https://github.com/osqp/osqp.git" "$OSQP_VERSION" "$SRC_DIR/osqp"
  cmake -S "$SRC_DIR/osqp" -B "$SRC_DIR/osqp/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$SRC_DIR/osqp/build" -j"$(nproc)"
  sudo cmake --install "$SRC_DIR/osqp/build"
  sudo ldconfig

  log "Build and install osqp-eigen ${OSQP_EIGEN_VERSION}"
  clone_or_update_tag "https://github.com/robotology/osqp-eigen.git" "$OSQP_EIGEN_VERSION" "$SRC_DIR/osqp-eigen"
  cmake -S "$SRC_DIR/osqp-eigen" -B "$SRC_DIR/osqp-eigen/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$SRC_DIR/osqp-eigen/build" -j"$(nproc)"
  sudo cmake --install "$SRC_DIR/osqp-eigen/build"
  sudo ldconfig

  log "Verify installation"
  cmake --version
  gcc --version | head -n 1
  git --version
  python3 -c 'import transforms3d, matplotlib; print("transforms3d", transforms3d.__version__); print("matplotlib", matplotlib.__version__)'
  bash -lc "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && ros2 --help >/dev/null && echo 'ros2 ok'"

  log "Done"
  echo "Open a new terminal, or run: source ~/.bashrc"
}

main "$@"
