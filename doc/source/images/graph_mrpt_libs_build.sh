#!/bin/sh
# Regenerates the graph of dependencies between MRPT modules (from their
# package.xml files), keeping only direct dependencies (transitive reduction).
set -e
cd "$(dirname "$0")"
python3 graph_mrpt_libs.py
tred graph_mrpt_libs.dot > graph_mrpt_libs_reduced.dot
dot -Tsvg -o graph_mrpt_libs.svg graph_mrpt_libs_reduced.dot
rm graph_mrpt_libs_reduced.dot
