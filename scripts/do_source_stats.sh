#!/bin/bash

echo Number of source files: `find . -name '*.c' -o -name '*.cpp' -o -name '*.h' |   wc -l`
echo Source code lines: `find . -name '*.c' -o -name '*.cpp' -o -name '*.h' | xargs cat | wc -l`

