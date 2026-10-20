#!/usr/bin/env bash
# Inspect an existing local ELF, without rebuilding or contacting a target.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s [Debug|Release]\n' "$0"
    exit 0
fi
[[ $# -le 1 ]] || fail 'Expected at most one preset. Use --help.'
select_preset "${1:-Debug}"
require_tool arm-none-eabi-nm
require_tool arm-none-eabi-size
verify_cache_source
elf="$NAVBOT_BUILD_DIR/navbot_body.elf"
[[ -f "$elf" ]] || fail "ELF not found; run scripts/build.sh $NAVBOT_PRESET first."
# Write only after nm succeeds, so a failed inspection does not truncate a report.
temporary="$(mktemp "$NAVBOT_BUILD_DIR/.symbols.XXXXXX")"
trap 'rm -f -- "$temporary"' EXIT
arm-none-eabi-nm --numeric-sort --print-size --defined-only "$elf" > "$temporary"
mv -f -- "$temporary" "$NAVBOT_BUILD_DIR/navbot_body.symbols.txt"
arm-none-eabi-size "$elf" | tee "$NAVBOT_BUILD_DIR/navbot_body.size.txt"
printf 'Symbol table: %s\n' "$NAVBOT_BUILD_DIR/navbot_body.symbols.txt"
