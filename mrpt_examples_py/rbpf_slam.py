#!/usr/bin/env python3

# RBPF-SLAM: builds a gridmap from a 2D lidar dataset with a Rao-Blackwellized
# particle filter.
#
# Usage (from the MRPT source tree):
#
#   . install/setup.bash
#   ./mrpt_examples_py/rbpf_slam.py \
#       -c modules/mrpt_data/config_files/rbpf-slam/gridmapping_optimal_sampling.ini
#
# The rawlog is read from the config file ([MappingApplication] rawlog_file)
# unless one is given as argument. Use --max-steps N to stop early.

import argparse
import os
import sys

from mrpt.config import CConfigFile
from mrpt.io import CCompressedInputStream, archiveFrom
from mrpt.obs import CRawlog
from mrpt.slam import CMetricMapBuilderRBPF

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../modules/mrpt_data/config_files/rbpf-slam/gridmapping_optimal_sampling.ini")

parser = argparse.ArgumentParser()
parser.add_argument("rawlog", nargs="?", help="Rawlog file (default: the one in the config file)")
parser.add_argument("-c", "--config", default=DEFAULT_CONFIG, help="Config file (.ini)")
parser.add_argument("-o", "--output", default="final_map",
                    help="Prefix of the output map files (default: final_map)")
parser.add_argument("--max-steps", type=int, default=0,
                    help="Stop after this many rawlog entries (default: 0 = all)")
args = parser.parse_args()

config_filename = os.path.abspath(args.config)
if not os.path.exists(config_filename):
    sys.exit(f"Error: config file not found: {config_filename}")
section = "MappingApplication"

rawlog_filename = args.rawlog
if not rawlog_filename:
    rawlog_filename = os.path.join(
        os.path.dirname(config_filename),
        CConfigFile(config_filename).read_string(section, "rawlog_file", ""))
if not os.path.exists(rawlog_filename):
    sys.exit(f"Error: rawlog file not found: {rawlog_filename}")

# RBPF options, including the definition of the maps to build:
options = CMetricMapBuilderRBPF.TConstructionOptions()
options.loadFromConfigFileName(config_filename, section)

map_builder = CMetricMapBuilderRBPF(options)
map_builder.initialize()

# Process the rawlog as a stream of (action, sensory frame) pairs:
rawlog_arch = archiveFrom(CCompressedInputStream(rawlog_filename))
entry = 0
steps = 0
while True:
    read_ok, entry, actions, sf, obs = CRawlog.ReadFromArchive(rawlog_arch, entry)
    if not read_ok:
        break
    if actions is None or sf is None:
        continue  # RBPF-SLAM needs rawlogs in the (action, sensory frame) format

    map_builder.processActionObservation(actions, sf)
    print(f"Entry {entry}: pose={map_builder.getCurrentPoseEstimation().getMean()}, "
          f"keyframes={map_builder.getCurrentlyBuiltMapSize()}")

    steps += 1
    if args.max_steps and steps >= args.max_steps:
        break

# Save the map of the most likely particle, as a simplemap (keyframes) and in
# a format for inspection (e.g. an image for gridmaps):
print(f'Saving final map to: "{args.output}*"')
if not map_builder.getCurrentlyBuiltMap().saveToFile(args.output + ".simplemap"):
    sys.exit("Error saving the simplemap")
map_builder.getCurrentlyBuiltMetricMap().saveMetricMapRepresentationToFile(args.output)
