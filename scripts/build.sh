#!/usr/bin/env bash
set -e
INSPECTION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$INSPECTION_ROOT"
source scripts/env.sh
git submodule update --init --depth 1
if git -C third_party/stonefish_sim apply --check "$INSPECTION_ROOT/patches/stonefish_node_lifetime.patch" 2>/dev/null; then
  git -C third_party/stonefish_sim apply "$INSPECTION_ROOT/patches/stonefish_node_lifetime.patch"
else
  git -C third_party/stonefish_sim apply --reverse --check "$INSPECTION_ROOT/patches/stonefish_node_lifetime.patch"
fi
mkdir -p .local evidence
if [[ ! -f .local/glm/glm/glm.hpp ]]; then
  git clone --depth 1 --branch 0.9.9.8 https://github.com/g-truc/glm.git .local/glm
fi
[[ $(git -C .local/glm rev-parse HEAD) == bf71a834948186f4097caa076cd2663c69a10e1e ]]
cmake -S third_party/stonefish -B build/stonefish \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSPECTION_ROOT/.local/stonefish" \
  -DCMAKE_CXX_FLAGS="-I$INSPECTION_ROOT/.local/glm"
cmake --build build/stonefish -j 12
cmake --install build/stonefish
python3 scripts/generate_scene.py
colcon build --merge-install --symlink-install \
  --base-paths third_party/stonefish_sim src \
  --packages-select stonefish_msgs stonefish_description stonefish_ros2 \
    stonefish_control_msgs stonefish_thruster_manager stonefish_control \
    underwater_inspection \
  --cmake-args -DBUILD_TESTING=OFF \
    "-DCMAKE_CXX_FLAGS=-I$INSPECTION_ROOT/.local/glm"
