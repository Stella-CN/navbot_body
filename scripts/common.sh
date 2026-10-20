#!/usr/bin/env bash
# Shared helpers for Bash 3.2+; resolve paths independently of the caller's cwd.
set -euo pipefail
NAVBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

fail() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

require_tool() {
    command -v "$1" >/dev/null 2>&1 || fail "Missing tool: $1 (add it to PATH)."
}

select_preset() {
    case "${1:-Debug}" in
        Debug|Release) NAVBOT_PRESET="${1:-Debug}" ;;
        *) fail 'Preset must be Debug or Release.' ;;
    esac
    NAVBOT_BUILD_DIR="$NAVBOT_ROOT/build/$NAVBOT_PRESET"
}

verify_cache_source() {
    local cache="$NAVBOT_BUILD_DIR/CMakeCache.txt"
    local source_dir
    if [[ -f "$cache" ]]; then
        source_dir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")"
        [[ "$source_dir" == "$NAVBOT_ROOT" ]] || fail "Build cache belongs to another source tree: $source_dir"
    fi
}
