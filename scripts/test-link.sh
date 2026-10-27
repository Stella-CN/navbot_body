#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: test-link.sh (host C/Python protocol tests; no hardware)"
    exit 0
fi
if [[ $# -ne 0 ]]; then echo "Usage: test-link.sh" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/navbot-link-test.XXXXXX")"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
    -I "$ROOT/Applications" "$ROOT/tests/link_protocol_test.c" \
    "$ROOT/Applications/link_protocol.c" -o "$TEST_DIR/link-test"
"$TEST_DIR/link-test"
python3 "$ROOT/tests/test_chassis_serial.py"
