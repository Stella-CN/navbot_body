#!/usr/bin/env bash
# Execute actual C++ motor layer and C BSP against register mocks; no hardware.
set -euo pipefail
if [[ "${1:-}" == '--help' ]]; then
    echo 'Usage: test-motor.sh (host C/C++ motor tests; no probe)'
    exit 0
fi
[[ $# -eq 0 ]] || exit 2
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/navbot-motor-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT
flags=(-Wall -Wextra -Werror -fsanitize="${SANITIZERS:-undefined}" -fno-omit-frame-pointer
       -I "$ROOT/tests/motor_mock" -I "$ROOT/Applications" -I "$ROOT/BSP/C30D")
"${CC:-cc}" -std=c11 "${flags[@]}" -c "$ROOT/BSP/C30D/c30d_motor.c" -o "$TEST_DIR/bsp.o"
"${CXX:-c++}" -std=c++17 -fno-rtti -fno-exceptions "${flags[@]}" \
    "$ROOT/tests/motor_test.cpp" "$ROOT/Applications/motor.cpp" \
    "$ROOT/Applications/wheel_motors.cpp" "$TEST_DIR/bsp.o" -o "$TEST_DIR/motor-test"
"$TEST_DIR/motor-test"
