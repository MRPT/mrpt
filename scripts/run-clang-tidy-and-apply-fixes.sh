#!/bin/bash

python3 /usr/lib/llvm-12/share/clang/run-clang-tidy.py \
    -p build/ \
    -checks=readability-braces-around-statements  \
    -fix \
    -clang-tidy-binary /usr/bin/clang-tidy-14 \
    -clang-apply-replacements-binary /usr/bin/clang-apply-replacements-14
