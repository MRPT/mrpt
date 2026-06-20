#!/usr/bin/env bash
# Runs clang-format-14 on all C++ source files in the MRPT codebase.
# Usage:
#   scripts/clang_format_codebase.sh          # reformat in-place
#   scripts/clang_format_codebase.sh --check  # CI dry-run; exits non-zero if any file differs

set -euo pipefail

CLANG_FORMAT=${CLANG_FORMAT:-clang-format-14}
JOBS=${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)}

CHECK_MODE=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK_MODE=1
fi

# Directories to scan (relative to repo root)
INCLUDE_DIRS=(modules apps mrpt_examples_cpp)

# Path fragments to exclude
EXCLUDE_PATTERNS=(3rdparty imgui)

# Verify we are running from the repo root
if [[ ! -f colcon_defaults.yaml ]]; then
  echo "ERROR: Run this script from the MRPT repository root." >&2
  exit 1
fi

# Build find arguments for excluded dirs
PRUNE_ARGS=()
for pat in "${EXCLUDE_PATTERNS[@]}"; do
  PRUNE_ARGS+=(-path "*/$pat" -prune -o -path "*/$pat/*" -prune -o)
done

# Collect files
mapfile -t FILES < <(
  find "${INCLUDE_DIRS[@]}" \
    "${PRUNE_ARGS[@]}" \
    \( -name "*.h" -o -name "*.cpp" \) -print 2>/dev/null | sort
)

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No files found — check INCLUDE_DIRS and working directory."
  exit 1
fi

echo "Found ${#FILES[@]} file(s) to process (using ${JOBS} parallel jobs)."

if [[ $CHECK_MODE -eq 1 ]]; then
  # Write file list to a temp file for xargs
  TMPFILE=$(mktemp)
  trap 'rm -f "$TMPFILE"' EXIT
  printf '%s\0' "${FILES[@]}" > "$TMPFILE"

  # Run checks in parallel; collect failing files via a temp dir
  FAILDIR=$(mktemp -d)
  trap 'rm -rf "$FAILDIR" "$TMPFILE"' EXIT

  check_file() {
    local f="$1"
    if ! "$CLANG_FORMAT" --dry-run --Werror "$f" 2>/dev/null; then
      # Record failure (filename encoded as a file inside FAILDIR)
      printf '%s\n' "$f" > "$FAILDIR/$(printf '%s' "$f" | md5sum | cut -c1-32)"
    fi
  }
  export -f check_file
  export CLANG_FORMAT FAILDIR

  xargs -0 -P "$JOBS" -I{} bash -c 'check_file "$@"' _ {} < "$TMPFILE" || true

  FAILED=()
  if [[ -n "$(ls -A "$FAILDIR")" ]]; then
    while IFS= read -r line; do
      FAILED+=("$line")
    done < <(cat "$FAILDIR"/* | sort)
  fi

  if [[ ${#FAILED[@]} -gt 0 ]]; then
    echo ""
    echo "clang-format check FAILED for ${#FAILED[@]} file(s):"
    printf '  %s\n' "${FAILED[@]}"
    exit 1
  fi
  echo "clang-format check passed."
else
  # Reformat in parallel — clang-format -i is safe to run concurrently on distinct files
  printf '%s\0' "${FILES[@]}" | \
    xargs -0 -P "$JOBS" -n 1 "$CLANG_FORMAT" -i
  echo "Reformatting done."
fi