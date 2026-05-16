#!/usr/bin/env bash
# Runs clang-format-14 on all C++ source files in the MRPT codebase.
# Usage:
#   scripts/clang_format_codebase.sh          # reformat in-place
#   scripts/clang_format_codebase.sh --check  # CI dry-run; exits non-zero if any file differs

set -euo pipefail

CLANG_FORMAT=${CLANG_FORMAT:-clang-format-14}

CHECK_MODE=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK_MODE=1
fi

# Directories to scan (relative to repo root)
INCLUDE_DIRS=(modules apps mrpt_examples_cpp)

# Path fragments to exclude
EXCLUDE_PATTERNS=(3rdparty imgui)

# Verify we are running from the repo root
if [[ ! -f version_prefix.txt ]]; then
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

echo "Found ${#FILES[@]} file(s) to process."

if [[ $CHECK_MODE -eq 1 ]]; then
  FAILED=()
  for f in "${FILES[@]}"; do
    if ! "$CLANG_FORMAT" --dry-run --Werror "$f" 2>/dev/null; then
      FAILED+=("$f")
    fi
  done
  if [[ ${#FAILED[@]} -gt 0 ]]; then
    echo ""
    echo "clang-format check FAILED for ${#FAILED[@]} file(s):"
    printf '  %s\n' "${FAILED[@]}"
    exit 1
  fi
  echo "clang-format check passed."
else
  for f in "${FILES[@]}"; do
    "$CLANG_FORMAT" -i "$f"
  done
  echo "Reformatting done."
fi
