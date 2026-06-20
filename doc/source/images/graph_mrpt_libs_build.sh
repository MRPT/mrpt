#!/bin/sh

# Automatic graph
(cd ../../../ && colcon graph --dot --packages-skip mrpt_common | dot -Tpng -o doc/source/images/graph_mrpt_modules.png)

# Old manual graph
dot -Tpng -o graph_mrpt_libs.png graph_mrpt_libs.dot
dot -Tcmapx -o graph_mrpt_libs.map  graph_mrpt_libs.dot

dot -Tsvg -o graph_mrpt_libs.svg graph_mrpt_libs.dot

