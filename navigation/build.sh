#!/usr/bin/env bash
set -e
nav="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$nav/../mapping/ws_livox/mapping_env.bash"
cd "$nav/../mapping/reloc2_ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select fast_lio --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
cd "$nav/ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select elf_navigation --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release

cmake -S "$nav/sdk" -B "$nav/sdk/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$nav/sdk/build" -j2
python3 -m unittest discover -s "$nav/tests" -v

(cd "$nav/sdk/build" && ctest --output-on-failure)
