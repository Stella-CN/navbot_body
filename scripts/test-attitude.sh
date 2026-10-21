#!/usr/bin/env bash
# Portable calibration/fusion tests; no probe access.
set -euo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: test-attitude.sh (host-only calibration/fusion tests)"
    exit 0
fi
if [[ $# -ne 0 ]]; then echo "Usage: test-attitude.sh" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/navbot-attitude-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
    -I "$ROOT/Applications" -I "$ROOT/BSP/C30D" \
    "$ROOT/tests/attitude_test.c" "$ROOT/Applications/attitude.c" \
    -o "$TEST_DIR/attitude-test" -lm
"$TEST_DIR/attitude-test"
