#!/bin/sh

srba-slam --se3 --lm-3d --obs MonocularCamera -d OUT_dataset_tutorials_mono_SENSOR.txt --sensor-params-cfg-file OUT_dataset_tutorials_mono_CAMCALIB.txt --submap-size 10 --max-spanning-tree-depth 3 --max-optimize-depth 3 --verbose 1 --noise 0.5 --add-noise # --step-by-step
