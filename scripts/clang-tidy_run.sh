#!/bin/bash
# Script to run clang-tidy checks on all sources of a project.
# Invoke from the root directory of the project, like:
# bash scripts/clang-tidy_run.sh "modernize-use-override"
# bash scripts/clang-tidy_run.sh "modernize-use-override,bugprone-fold-init-type"
#
# Requisites: sudo apt install -y clang-tidy clang-tools
#
# JLBC, Sep 2018

mkdir build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..

run-clang-tidy -header-filter='.*' -checks="-*,$1" -fix
