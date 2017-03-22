#!/bin/bash

# to be sure of available versions:
g++ --version
gcov --version

lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info 'test/*' '/usr/*' --output-file coverage.info
lcov --list coverage.info


coveralls-lcov --repo-token ${COVERALLS_TOKEN} coverage.info
