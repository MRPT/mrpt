#!/bin/sh

srba-slam --se2 --graph-slam -d dataset_30k_rel_graph_slam_SENSOR.txt --submap-size 10 --max-spanning-tree-depth 3 --max-optimize-depth 3 --gui-delay 20 --verbose 1 --step-by-step --noise 0.001 --noise-ang 0.2 --add-noise --gt-map dataset_30k_rel_graph_slam_GT_MAP.txt --gt-path dataset_30k_rel_graph_slam_GT_PATH.txt
