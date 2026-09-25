#!/usr/bin/env bash
# Build MAVSDK (C++ library) from source into a project-local prefix. Nothing is installed system-wide.
#   source:  third_party/MAVSDK          (git clone, ignored by git)
#   install: third_party/mavsdk-install  (used by our CMakeLists via CMAKE_PREFIX_PATH)
# SUPERBUILD downloads and builds MAVSDK's own dependencies (jsoncpp, tinyxml2, ...) into the same prefix.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="v4.0.0"
SRC="$ROOT/third_party/MAVSDK"
PREFIX="$ROOT/third_party/mavsdk-install"

if [[ ! -d "$SRC" ]]; then
  git clone --depth 1 --branch "$VERSION" --recurse-submodules --shallow-submodules \
    https://github.com/mavlink/MAVSDK.git "$SRC"
fi

cmake -S "$SRC/cpp" -B "$SRC/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DSUPERBUILD=ON \
  -DBUILD_MAVSDK_SERVER=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_WITHOUT_CURL=ON \
  -DMAVLINK_DIALECT=ardupilotmega
cmake --build "$SRC/build" -j"$(sysctl -n hw.ncpu)"
cmake --install "$SRC/build"
echo "MAVSDK installed to $PREFIX"
