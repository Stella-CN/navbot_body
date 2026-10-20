#!/usr/bin/env bash
# Restrict formatting to maintained C/C++ sources, not CubeMX/vendor files.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s [--check|--write] (default: --check)\n' "$0"
    exit 0
fi
[[ $# -le 1 ]] || fail 'Expected at most one mode. Use --help.'
case "${1:---check}" in
    --check) options=(--dry-run --Werror) ;;
    --write) options=(-i) ;;
    *) fail 'Mode must be --check or --write.' ;;
esac
require_tool clang-format
for directory in "$NAVBOT_ROOT/Applications" "$NAVBOT_ROOT/BSP"; do
    [[ -d "$directory" ]] || fail "Source directory not found: $directory"
done

# Finish scanning before any writes, and explicitly check find's exit status.
# Process substitution would not propagate scan errors to this script.
file_list="$(mktemp "${TMPDIR:-/tmp}/navbot-format.XXXXXX")"
trap 'rm -f -- "$file_list"' EXIT
if ! find "$NAVBOT_ROOT/Applications" "$NAVBOT_ROOT/BSP" -type f \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \) -print0 > "$file_list"; then
    fail 'Failed to scan source directories; no files were formatted.'
fi
while IFS= read -r -d '' file; do
    clang-format --style=file "${options[@]}" "$file"
done < "$file_list"
