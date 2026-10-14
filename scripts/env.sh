#!/usr/bin/env bash
# Source this file before native ROS 2 launch commands.
INSPECTION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PATH="/usr/bin:$PATH"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [[ -f "$INSPECTION_ROOT/install/setup.bash" ]]; then
  source "$INSPECTION_ROOT/install/setup.bash"
fi
export CMAKE_PREFIX_PATH="$INSPECTION_ROOT/.local/stonefish:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$INSPECTION_ROOT/.local/stonefish/lib:${LD_LIBRARY_PATH:-}"
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
# Keep this simulation separate from other ROS applications on the host.
export ROS_DOMAIN_ID=73
