#!/usr/bin/env bash
# Configure, compile and export local artifacts. Does not access hardware.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s [Debug|Release] [positive-job-count]\n' "$0"
    exit 0
fi
[[ $# -le 2 ]] || fail 'Expected at most a preset and job count. Use --help.'
select_preset "${1:-Debug}"
jobs="${2:-${CMAKE_BUILD_PARALLEL_LEVEL:-4}}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || fail 'Job count must be a positive integer.'
for tool in cmake ninja arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size; do
    require_tool "$tool"
done
verify_cache_source
cd -- "$NAVBOT_ROOT"
mkdir -p -- "$NAVBOT_BUILD_DIR"
cmake --preset "$NAVBOT_PRESET" 2>&1 | tee "$NAVBOT_BUILD_DIR/configure.log"
cmake --build --preset "$NAVBOT_PRESET" --parallel "$jobs" 2>&1 | tee "$NAVBOT_BUILD_DIR/build.log"
elf="$NAVBOT_BUILD_DIR/navbot_body.elf"
[[ -f "$elf" ]] || fail "Build did not produce $elf"
arm-none-eabi-objcopy -O binary "$elf" "$NAVBOT_BUILD_DIR/navbot_body.bin"
arm-none-eabi-objcopy -O ihex "$elf" "$NAVBOT_BUILD_DIR/navbot_body.hex"
arm-none-eabi-size "$elf" | tee "$NAVBOT_BUILD_DIR/navbot_body.size.txt"
printf 'Build succeeded: %s\n' "$elf"
