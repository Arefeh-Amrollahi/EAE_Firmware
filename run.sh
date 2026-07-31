#!/usr/bin/env bash
#
# run.sh : configure, build, test and launch the cooling loop firmware.
#
# Any arguments after the command are forwarded verbatim to the executable, so
# setpoints and gains can be changed at runtime without touching the build:
#
#   ./run.sh run --scenario overload --setpoint 42 --kp 10
#   ./run.sh test
#   ./run.sh all
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="$(nproc 2>/dev/null || echo 4)"

usage() {
    cat <<'EOF'
Usage: ./run.sh <command> [options passed to the program]

Commands:
  build            configure and compile
  test             build, then run the GoogleTest suite through CTest
  run [args...]    build, then launch the simulation with the given arguments
  all              build, test, then run every scenario in turn
  clean            remove the build directory
  help             show the program's own option list

Environment:
  BUILD_TYPE       Release (default) or Debug

Examples:
  ./run.sh run --scenario duty --setpoint 45 --derate 65
  ./run.sh run --scenario overload --kp 12 --ki 0.4 --verbose
  BUILD_TYPE=Debug ./run.sh test
EOF
}

require_cmake() {
    if ! command -v cmake >/dev/null 2>&1; then
        echo "error: cmake is not on PATH." >&2
        echo "       Install it with your package manager, or 'pip install cmake'." >&2
        exit 1
    fi
}

do_build() {
    require_cmake
    # GoogleTest is fetched at configure time and is deliberately NOT vendored
    # into this repository, so the first configure needs network access.
    cmake -S "${ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_test() {
    do_build
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
}

case "${1:-help}" in
    build) do_build ;;
    test)  do_test ;;
    run)
        shift || true
        do_build
        "${BUILD_DIR}/eae_cooling" "$@"
        ;;
    all)
        do_test
        for s in duty overload leak sensor canloss seized; do
            echo
            echo "###############################################################"
            echo "# scenario: ${s}"
            echo "###############################################################"
            "${BUILD_DIR}/eae_cooling" --scenario "${s}"
        done
        ;;
    clean) rm -rf "${BUILD_DIR}"; echo "removed ${BUILD_DIR}" ;;
    help|-h|--help) usage ;;
    *)
        echo "error: unknown command '$1'" >&2
        usage
        exit 2
        ;;
esac
