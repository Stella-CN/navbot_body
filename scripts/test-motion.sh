#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$(mktemp -d "${TMPDIR:-/tmp}/navbot-motion-test.XXXXXX")"
trap 'rm -rf "$DIR"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-exceptions -fno-rtti \
 -I "$ROOT/Applications" "$ROOT/tests/motion_test.cpp" "$ROOT/Applications/motor.cpp" "$ROOT/Applications/motion.cpp" -o "$DIR/test"
"$DIR/test"
python3 "$ROOT/tests/motion_host_test.py"
python3 "$ROOT/tests/motion_metrics_test.py"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-exceptions -fno-rtti \
 -I "$ROOT/tests/motion_mock" -I "$ROOT/Applications" -I "$ROOT/BSP/C30D" \
 "$ROOT/tests/motion_service_test.cpp" "$ROOT/Applications/motion_service.cpp" \
 "$ROOT/Applications/motion.cpp" "$ROOT/Applications/motor.cpp" \
 "$ROOT/Applications/wheel_motors.cpp" "$ROOT/Applications/link_protocol.cpp" -o "$DIR/service-test"
"$DIR/service-test"
