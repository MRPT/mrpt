#!/bin/bash

echo "Only run this script from the root source of eigen!!! (Ctrl+C in 5 secs to abort)"
sleep 5

find . \( ! -regex '.*/\..*' \) -type f | xargs grep "\\defgroup" | cut -f 1 -d ':' | xargs perl -i -p  -e "s/defgroup (.*)/defgroup \1 \n * \\\\ingroup eigen_grp/g"

perl -i -p  -e "s/defgroup (.*)/defgroup eigen_grp The Eigen3 library \n  \\\\defgroup \1/g" Core

