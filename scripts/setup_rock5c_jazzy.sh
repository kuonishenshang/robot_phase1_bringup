#!/usr/bin/env bash
set -Eeuo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROBOT_STORAGE="${ROBOT_STORAGE:-${HOME}/robot_storage}"

INSTALL_ROS="${INSTALL_ROS:-1}"
CLONE_REPOS="${CLONE_REPOS:-1}"
BUILD_WORKSPACES="${BUILD_WORKSPACES:-1}"
BUILD_CAMERA="${BUILD_CAMERA:-1}"
INSTALL_CAMERA_UDEV="${INSTALL_CAMERA_UDEV:-1}"
INSTALL_HARDWARE_PERMISSIONS="${INSTALL_HARDWARE_PERMISSIONS:-1}"
INSTALL_RVIZ="${INSTALL_RVIZ:-1}"
ALLOW_UNSUPPORTED="${ALLOW_UNSUPPORTED:-0}"
RUN_ROSDEP="${RUN_ROSDEP:-1}"
ROSDEP_COMMAND="${ROSDEP_COMMAND:-rosdep}"
ROSDEP_EXECUTABLE=""

readonly CAMERA_REPO="https://github.com/kuonishenshang/ros2_astra_camera_Jazzy.git"
readonly SERIAL_REPO="https://github.com/kuonishenshang/serial_port_test.git"
readonly LIDAR_REPO="https://github.com/kuonishenshang/cspc_lidar_ros2_Jazzy.git"
readonly PHASE1_REPO="https://github.com/kuonishenshang/robot_phase1_bringup.git"

log() {
  printf '\n[rock5c-setup] %s\n' "$*"
}

die() {
  printf '\n[rock5c-setup] ERROR: %s\n' "$*" >&2
  exit 1
}

warn() {
  printf '\n[rock5c-setup] WARNING: %s\n' "$*" >&2
}

resolve_rosdep_executable() {
  if [[ -n "${ROSDEP_EXECUTABLE}" ]]; then
    return 0
  fi

  if ! ROSDEP_EXECUTABLE="$(command -v "${ROSDEP_COMMAND}")"; then
    warn "Dependency resolver '${ROSDEP_COMMAND}' is not installed; continuing without rosdep checks."
    RUN_ROSDEP=0
    return 1
  fi
}

initialize_rosdep() {
  [[ "${RUN_ROSDEP}" == "1" ]] || return 0
  resolve_rosdep_executable || return 0

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    if ! sudo "${ROSDEP_EXECUTABLE}" init; then
      warn "${ROSDEP_COMMAND} init failed; continuing because required dependencies are installed explicitly."
      RUN_ROSDEP=0
      return 0
    fi
  else
    log "rosdep-compatible source list already exists."
  fi

  if ! "${ROSDEP_EXECUTABLE}" update; then
    warn "${ROSDEP_COMMAND} update failed; continuing because required dependencies are installed explicitly."
    RUN_ROSDEP=0
  fi
}

source_setup_file() {
  local setup_file="$1"
  local restore_nounset=0

  [[ -f "${setup_file}" ]] || die "Missing setup file: ${setup_file}"

  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi

  # ROS-generated setup files are not guaranteed to be nounset-safe.
  # shellcheck disable=SC1090
  source "${setup_file}"

  [[ "${restore_nounset}" == "0" ]] || set -u
}

check_platform() {
  local arch codename

  arch="$(uname -m)"
  # shellcheck disable=SC1091
  . /etc/os-release
  codename="${VERSION_CODENAME:-unknown}"

  log "Detected architecture=${arch}, ubuntu=${codename}"

  if [[ "${arch}" != "aarch64" || "${codename}" != "noble" ]]; then
    if [[ "${ALLOW_UNSUPPORTED}" != "1" ]]; then
      die "This script targets Rock5C Ubuntu 24.04 noble on aarch64. Set ALLOW_UNSUPPORTED=1 to continue anyway."
    fi
    log "Continuing on unsupported platform because ALLOW_UNSUPPORTED=1."
  fi
}

install_ros_and_dependencies() {
  log "Installing ROS 2 ${ROS_DISTRO} and build/runtime dependencies."

  sudo apt update
  sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
  sudo add-apt-repository -y universe

  if ! grep -Rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d; then
    local codename ros_apt_source_version

    # shellcheck disable=SC1091
    . /etc/os-release
    codename="${VERSION_CODENAME:?VERSION_CODENAME is not set}"
    ros_apt_source_version="$(
      curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        awk -F'"' '/tag_name/ {print $4; exit}'
    )"

    [[ -n "${ros_apt_source_version}" ]] || die "Unable to resolve latest ros-apt-source release."

    curl -fsSL \
      -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${codename}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
  else
    log "ROS 2 apt source already exists."
  fi

  sudo apt update

  local packages=(
    build-essential
    cmake
    git
    gh
    pkg-config
    python3-colcon-common-extensions
    python3-rosdep
    python3-vcstool
    libboost-system-dev
    libpcl-dev
    libuvc-dev
    libgoogle-glog-dev
    libgflags-dev
    libusb-1.0-0-dev
    libeigen3-dev
    "ros-${ROS_DISTRO}-ros-base"
    ros-dev-tools
    "ros-${ROS_DISTRO}-slam-toolbox"
    "ros-${ROS_DISTRO}-navigation2"
    "ros-${ROS_DISTRO}-nav2-bringup"
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-cv-bridge"
    "ros-${ROS_DISTRO}-image-geometry"
    "ros-${ROS_DISTRO}-camera-info-manager"
    "ros-${ROS_DISTRO}-image-transport"
    "ros-${ROS_DISTRO}-image-publisher"
    "ros-${ROS_DISTRO}-tf2-sensor-msgs"
    "ros-${ROS_DISTRO}-tf2-eigen"
  )

  if [[ "${INSTALL_RVIZ}" == "1" ]]; then
    packages+=("ros-${ROS_DISTRO}-rviz2")
  fi

  sudo apt install -y "${packages[@]}"

  initialize_rosdep
}

ensure_github_auth() {
  command -v gh >/dev/null 2>&1 || die "GitHub CLI is not installed. Re-run with INSTALL_ROS=1 or install gh manually."

  if ! gh auth status >/dev/null 2>&1; then
    cat >&2 <<'EOF'

GitHub authentication is required for the private camera, serial, and bringup repositories.
Run these commands on the Rock5C, then re-run this script:

  gh auth login --web --git-protocol https --insecure-storage
  gh auth setup-git

EOF
    exit 2
  fi

  gh auth setup-git
}

clone_or_update() {
  local repo_url="$1"
  local repo_dir="$2"

  if [[ -d "${repo_dir}/.git" ]]; then
    local current_bringup_dir repo_real_dir
    current_bringup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
    repo_real_dir="$(cd "${repo_dir}" && pwd -P)"

    if [[ "${repo_real_dir}" == "${current_bringup_dir}" ]]; then
      log "Using current bringup checkout at ${repo_dir}; not pulling the running script repository."
      return
    fi

    log "Updating ${repo_dir}"
    git -C "${repo_dir}" pull --ff-only
  else
    log "Cloning ${repo_url} -> ${repo_dir}"
    mkdir -p "$(dirname "${repo_dir}")"
    git clone --branch main "${repo_url}" "${repo_dir}"
  fi
}

clone_repositories() {
  ensure_github_auth

  clone_or_update "${CAMERA_REPO}" "${ROBOT_STORAGE}/camera_test_ws/src/ros2_astra_camera"
  clone_or_update "${SERIAL_REPO}" "${ROBOT_STORAGE}/serial_test_ws/src/serial_port_test"
  clone_or_update "${LIDAR_REPO}" "${ROBOT_STORAGE}/lidar_test_ws/src/cspc_lidar_sdk_ros2"
  clone_or_update "${PHASE1_REPO}" "${ROBOT_STORAGE}/phase1_nav_ws/src/robot_phase1_bringup"
}

run_rosdep_for_ws() {
  local ws_dir="$1"
  local skip_keys="${2:-}"

  if [[ "${RUN_ROSDEP}" == "1" ]]; then
    resolve_rosdep_executable || return 0
    local args=(
      install
      --from-paths "${ws_dir}/src"
      --ignore-src
      -r
      -y
      --rosdistro "${ROS_DISTRO}"
    )

    if [[ -n "${skip_keys}" ]]; then
      args+=(--skip-keys "${skip_keys}")
    fi

    "${ROSDEP_EXECUTABLE}" "${args[@]}"
  fi
}

build_workspaces() {
  local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"

  [[ -f "${ros_setup}" ]] || die "Missing ${ros_setup}. Install ROS first."

  source_setup_file "${ros_setup}"

  log "Building serial chassis workspace."
  run_rosdep_for_ws "${ROBOT_STORAGE}/serial_test_ws"
  cd "${ROBOT_STORAGE}/serial_test_ws"
  colcon build --symlink-install --event-handlers console_direct+

  source_setup_file "${ROBOT_STORAGE}/serial_test_ws/install/setup.bash"

  log "Building lidar workspace."
  run_rosdep_for_ws "${ROBOT_STORAGE}/lidar_test_ws"
  cd "${ROBOT_STORAGE}/lidar_test_ws"
  colcon build --symlink-install --event-handlers console_direct+

  source_setup_file "${ROBOT_STORAGE}/lidar_test_ws/install/setup.bash"

  log "Building phase1 bringup workspace."
  local phase1_skip_keys="serial_port_test cspc_lidar"
  if [[ "${INSTALL_RVIZ}" != "1" ]]; then
    phase1_skip_keys="${phase1_skip_keys} rviz2"
  fi
  run_rosdep_for_ws "${ROBOT_STORAGE}/phase1_nav_ws" "${phase1_skip_keys}"
  cd "${ROBOT_STORAGE}/phase1_nav_ws"
  colcon build --symlink-install --event-handlers console_direct+

  if [[ "${BUILD_CAMERA}" == "1" ]]; then
    log "Building camera workspace."
    # Keep the camera build independent from the navigation overlay.
    source_setup_file "${ros_setup}"
    run_rosdep_for_ws "${ROBOT_STORAGE}/camera_test_ws"
    cd "${ROBOT_STORAGE}/camera_test_ws"
    colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
}

install_hardware_permissions() {
  log "Installing serial/video group permissions and Orbbec camera udev rules."

  sudo usermod -aG dialout,video,plugdev "${USER}"

  if [[ "${INSTALL_CAMERA_UDEV}" == "1" ]]; then
    local camera_install_script="${ROBOT_STORAGE}/camera_test_ws/src/ros2_astra_camera/astra_camera/scripts/install.sh"
    if [[ -f "${camera_install_script}" ]]; then
      sudo bash "${camera_install_script}"
      sudo udevadm control --reload-rules
      sudo udevadm trigger
    else
      log "Camera udev script not found at ${camera_install_script}; skipping."
    fi
  fi
}

write_environment_helper() {
  local env_file="${ROBOT_STORAGE}/setup_phase1.bash"

  mkdir -p "${ROBOT_STORAGE}"
  cat > "${env_file}" <<EOF
#!/usr/bin/env bash
_phase1_restore_nounset=0
if [[ \$- == *u* ]]; then
  _phase1_restore_nounset=1
  set +u
fi
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${ROBOT_STORAGE}/serial_test_ws/install/setup.bash
source ${ROBOT_STORAGE}/lidar_test_ws/install/setup.bash
source ${ROBOT_STORAGE}/phase1_nav_ws/install/setup.bash
[[ "\${_phase1_restore_nounset}" == "0" ]] || set -u
unset _phase1_restore_nounset
EOF
  chmod +x "${env_file}"

  log "Created ${env_file}"
}

print_next_steps() {
  cat <<EOF

Rock5C setup finished.

EOF

  if [[ "${BUILD_WORKSPACES}" == "1" ]]; then
    cat <<EOF
Open a new shell or reboot if group permissions were changed, then run:

  source ${ROBOT_STORAGE}/setup_phase1.bash
  ros2 launch robot_phase1_bringup mapping.launch.py start_rviz:=false

EOF
  else
    cat <<EOF
Workspace build was skipped. After building serial, lidar, and phase1 workspaces, source their install files before launching.

EOF
  fi

  cat <<EOF
After mapping:

  ros2 run nav2_map_server map_saver_cli -f ${ROBOT_STORAGE}/phase1_nav_ws/src/robot_phase1_bringup/maps/map
  ros2 launch robot_phase1_bringup navigation.launch.py start_rviz:=false map:=${ROBOT_STORAGE}/phase1_nav_ws/src/robot_phase1_bringup/maps/map.yaml

If the chassis or lidar device path differs, pass:

  base_serial_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0

EOF
}

main() {
  check_platform

  if [[ "${INSTALL_ROS}" == "1" ]]; then
    install_ros_and_dependencies
  fi

  if [[ "${CLONE_REPOS}" == "1" ]]; then
    clone_repositories
  fi

  if [[ "${INSTALL_HARDWARE_PERMISSIONS}" == "1" ]]; then
    install_hardware_permissions
  fi

  if [[ "${BUILD_WORKSPACES}" == "1" ]]; then
    build_workspaces
    write_environment_helper
  fi

  print_next_steps
}

main "$@"
