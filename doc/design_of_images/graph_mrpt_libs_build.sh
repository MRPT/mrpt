#!/bin/sh
dot -Tpng -o graph_mrpt_libs.png graph_mrpt_libs.dot
dot -Tcmapx -o graph_mrpt_libs.map  graph_mrpt_libs.dot
