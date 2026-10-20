#!/usr/bin/env bash
# Report host build tools only. Do not start OpenOCD or enumerate hardware.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s\n' "$0"
    exit 0
fi
[[ $# -eq 0 ]] || fail 'No arguments expected. Use --help.'
missing=0
for tool in cmake ninja python3 arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size arm-none-eabi-nm; do
    if command -v "$tool" >/dev/null 2>&1; then
        printf 'OK       %-24s %s\n' "$tool" "$(command -v "$tool")"
    else
        printf 'MISSING  %s\n' "$tool" >&2
        missing=1
    fi
done
for tool in clang-format arm-none-eabi-gdb; do
    if command -v "$tool" >/dev/null 2>&1; then
        printf 'OPTIONAL %-24s %s\n' "$tool" "$(command -v "$tool")"
    else
        printf 'OPTIONAL %s not in PATH\n' "$tool"
    fi
done
[[ "$missing" -eq 0 ]] || exit 1
target="$(arm-none-eabi-gcc -dumpmachine)"
[[ "$target" == 'arm-none-eabi' ]] || fail "Unexpected GCC target: $target"
printf 'GCC target: %s\n' "$target"
cmake --version
ninja --version
arm-none-eabi-gcc --version
