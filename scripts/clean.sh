#!/usr/bin/env bash
# Use the generated build system's clean target; never recursively delete paths.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s [Debug|Release]\n' "$0"
    exit 0
fi
[[ $# -le 1 ]] || fail 'Expected at most one preset. Use --help.'
select_preset "${1:-Debug}"
require_tool cmake
if [[ ! -f "$NAVBOT_BUILD_DIR/CMakeCache.txt" ]]; then
    printf 'Nothing configured for %s; nothing to clean.\n' "$NAVBOT_PRESET"
    exit 0
fi
verify_cache_source
cmake --build "$NAVBOT_BUILD_DIR" --target clean
# Exports are produced by scripts, not by the generated CMake target.
for suffix in bin hex size.txt symbols.txt; do
    rm -f -- "$NAVBOT_BUILD_DIR/navbot_body.$suffix"
done
