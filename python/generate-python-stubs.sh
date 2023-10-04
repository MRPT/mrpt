#!/bin/bash
# Usage: ./generate-python-stubs.sh 
# To be run *after* building pymrpt.


# Generate stub .pyi files:
echo "Generating stub pyi files..."
export PYTHONPATH=$(realpath $(pwd)/../build-Release/)
if compgen -G "$PYTHONPATH/mrpt/pymrpt*.so" > /dev/null; then
    echo "Using mrpt module found under: $PYTHONPATH"
else
    echo "$PYTHONPATH does not seem to contain a compiled pymrpt module!"
    exit 1
fi
stubgen -p mrpt -p mrpt.pymrpt -o stubs-out

# applying manual patches to stubs:
echo "Applying manual patches to stubs..."
find . -name "patch-stubs*.diff" | xargs -I FIL bash -c "echo FIL && git apply FIL --ignore-whitespace"
