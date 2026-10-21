#!/usr/bin/env bash
# Host-only driver state tests. Does not access the probe or target.
set -euo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: test-imu.sh (host-only state-machine tests; no probe access)"
    exit 0
fi
if [[ $# -ne 0 ]]; then
    echo "Usage: test-imu.sh" >&2
    exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/navbot-imu-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -DC30D_IMU_TEST_HOOKS \
    -I "$ROOT/BSP/C30D" "$ROOT/tests/imu_state_test.c" "$ROOT/BSP/C30D/c30d_imu.c" \
    -o "$TEST_DIR/imu-state-test" -lm
"$TEST_DIR/imu-state-test"
