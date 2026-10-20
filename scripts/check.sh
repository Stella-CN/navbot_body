#!/usr/bin/env bash
# Read-only firmware checks; does not invoke a compiler or probe.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ "${1:-}" == '--help' ]]; then
    printf 'Usage: %s\n' "$0"
    exit 0
fi
[[ $# -eq 0 ]] || fail 'No arguments expected. Use --help.'
require_tool python3
for script in "$NAVBOT_ROOT"/scripts/*.sh; do
    bash -n "$script"
done
exec python3 "$NAVBOT_ROOT/tools/static_validate.py"
