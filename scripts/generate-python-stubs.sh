#!/bin/bash
# Generate .pyi stub files for IDE autocompletion from the installed per-module mrpt bindings.
# Run *after* building with: colcon build && . install/setup.bash
#
# Requires: pip install mypy  (provides stubgen)
# Output: stubs-out/mrpt/<module>/_bindings.pyi  (one per submodule)
#
# Usage:
#   ./scripts/generate-python-stubs.sh [output-dir]
#
# The generated stubs can be pointed to by your IDE (e.g. pyrightconfig.json stubPath).

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${1:-${REPO_ROOT}/stubs-out}"

# Find the install tree's site-packages — use the first _bindings.so found
SITE_PKG=$(find "${REPO_ROOT}/install" -name "_bindings*.so" 2>/dev/null | head -1 | xargs dirname | xargs dirname | xargs dirname)

if [ -z "$SITE_PKG" ]; then
    echo "ERROR: No compiled _bindings.so found under install/. Build first with:"
    echo "  colcon build && . install/setup.bash"
    exit 1
fi

echo "Found site-packages: $SITE_PKG"
export PYTHONPATH="$SITE_PKG${PYTHONPATH:+:$PYTHONPATH}"

MODULES=$(find "${REPO_ROOT}/install" -name "_bindings*.so" 2>/dev/null \
    | sed 's|.*/mrpt/||;s|/_bindings.*||' | sort -u)

echo "Generating stubs for modules: $(echo $MODULES | tr '\n' ' ')"
mkdir -p "$OUT_DIR"

for MOD in $MODULES; do
    echo "  stubgen mrpt.${MOD}._bindings ..."
    stubgen -m "mrpt.${MOD}._bindings" -o "$OUT_DIR" 2>/dev/null || \
        echo "  WARNING: stubgen failed for mrpt.${MOD}._bindings (skipping)"
done

echo ""
echo "Stubs written to: $OUT_DIR"
echo ""
echo "To use in VS Code / Pylance, add to pyrightconfig.json:"
echo '  { "stubPath": "'"$OUT_DIR"'" }'
