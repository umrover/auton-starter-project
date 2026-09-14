#!/usr/bin/env bash

# Compile this package. Run it from an auton_starter shell: that alias
# supplies colcon, the mrover underlay, and MROVER_BUILD_PROFILE.
#
# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED_BOLD='\033[1;31m'
readonly NC='\033[0m'

# determine the build profile
build_profile="${1:-${MROVER_BUILD_PROFILE:-RelWithDebInfo}}"

if [[ "$#" -gt 1 ]] || { [[ "$#" -eq 1 ]] && [[ "$1" != "Release" && "$1" != "RelWithDebInfo" && "$1" != "Debug" ]]; }; then
    echo "Usage: scripts/build.sh [Release|RelWithDebInfo|Debug]"
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo -e "${RED_BOLD}colcon not found. Run auton_starter first, then build.${NC}"
    exit 1
fi

STARTER_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly STARTER_PATH

pushd "${STARTER_PATH}/../.."

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    # portable environment
    os_cmake_args=()
    if [[ "$(uname)" == "Darwin" ]]; then
        macos_sysroot=$(xcrun --sdk macosx --show-sdk-path)
        os_cmake_args=("-DCMAKE_OSX_SYSROOT=${macos_sysroot}")
    else
        # conda's pkg-config wrapper runs the GCC-only -print-sysroot
        os_cmake_args=("-DPKG_CONFIG_EXECUTABLE=${CONDA_PREFIX}/bin/pkg-config.bin")
        os_cmake_args+=("-DCMAKE_C_COMPILER=${CONDA_PREFIX}/bin/clang" "-DCMAKE_CXX_COMPILER=${CONDA_PREFIX}/bin/clang++")
    fi

    colcon build \
        --event-handlers console_direct+ \
        --cmake-args -G Ninja -Wno-dev -DCMAKE_BUILD_TYPE="${build_profile}" \
        -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}" \
        "${os_cmake_args[@]}" \
        --symlink-install \
        --packages-select mrover_autonomy_starter

    ln -sf "$(pwd)/build/mrover_autonomy_starter/compile_commands.json" "${STARTER_PATH}/compile_commands.json"
else
    # native environment
    export CC=clang
    export CXX=clang++

    colcon build \
        --event-handlers console_direct+ \
        --cmake-args -G Ninja -Wno-dev -DCMAKE_BUILD_TYPE="${build_profile}" \
        --symlink-install \
        --build-base "build/${build_profile}" \
        --install-base "install/${build_profile}" \
        --packages-select mrover_autonomy_starter

    ln -sf "$(pwd)/build/${build_profile}/mrover_autonomy_starter/compile_commands.json" "${STARTER_PATH}/compile_commands.json"
fi
