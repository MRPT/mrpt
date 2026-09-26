#!/usr/bin/env python3

# Global localization of a robot with a particle filter (Monte Carlo
# Localization) on a known map, reading odometry and laser scans from a
# rawlog dataset.
#
# Usage (from the MRPT source tree):
#
#   . install/setup.bash
#   ./mrpt_examples_py/global_localization.py \
#       modules/mrpt_data/config_files/pf-localization/localization_demo.ini
#
# Add --no-gui to run without a 3D window, and --max-steps N to stop early.

import argparse
import os
import sys
from time import sleep

from mrpt.bayes import CParticleFilter, TParticleFilterOptions
from mrpt.config import CConfigFile
from mrpt.img import TColorf
from mrpt.io import CCompressedInputStream, archiveFrom
from mrpt.maps import (
    CMultiMetricMap,
    COccupancyGridMap2D,
    CSimpleMap,
    TSetOfMetricMapInitializers,
    VisualizationParameters,
    obs_to_viz,
)
from mrpt.obs import CRawlog
from mrpt.poses import CPose3D
from mrpt.slam import CMonteCarloLocalization2D, TMonteCarloLocalizationParams

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../modules/mrpt_data/config_files/pf-localization/localization_demo.ini")

parser = argparse.ArgumentParser()
parser.add_argument("config", nargs="?", default=DEFAULT_CONFIG, help="Config file (.ini)")
parser.add_argument("-d", "--delay", type=float, default=0.2,
                    help="Delay between steps, in seconds (default: 0.2)")
parser.add_argument("-r", "--resolution", default="800x600",
                    help="Window resolution (default: 800x600)")
parser.add_argument("--no-gui", action="store_true", help="Do not open a 3D window")
parser.add_argument("--max-steps", type=int, default=0,
                    help="Stop after this many rawlog entries (default: 0 = all)")
args = parser.parse_args()

config_filename = os.path.abspath(args.config)
if not os.path.exists(config_filename):
    sys.exit(f"Error: config file not found: {config_filename}")
config_file = CConfigFile(config_filename)
print(f"Loaded config file {config_filename}")

# File names in the config are relative to the config file directory:
sec_name = "LocalizationExperiment"
config_dir = os.path.dirname(config_filename)
rawlog_filename = os.path.join(config_dir, config_file.read_string(sec_name, "rawlog_file", ""))
map_filename = os.path.join(config_dir, config_file.read_string(sec_name, "map_file", ""))
particles_count = config_file.read_int(sec_name, "particles_count", 10000)
for f in (rawlog_filename, map_filename):
    if not os.path.exists(f):
        sys.exit(f"Error: file not found: {f}")

# Particle filter options:
pf_options = TParticleFilterOptions()
pf_options.loadFromConfigFileName(config_filename, "PF_options")

# MCL options, including KLD-sampling (adaptive number of particles):
mcl_options = TMonteCarloLocalizationParams()
mcl_options.KLD_params.loadFromConfigFileName(config_filename, "KLD_options")

# The metric maps used to evaluate the observation likelihoods:
map_list = TSetOfMetricMapInitializers()
map_list.loadFromConfigFileName(config_filename, "MetricMap")
metric_map = CMultiMetricMap(map_list)

if map_filename.endswith((".simplemap", ".simplemap.gz")):
    # A view-based map (poses + observations): build the metric maps from it.
    simple_map = CSimpleMap()
    if not simple_map.loadFromFile(map_filename):
        sys.exit(f"Error loading {map_filename}")
    metric_map.loadFromSimpleMap(simple_map)
elif map_filename.endswith((".gridmap", ".gridmap.gz")):
    # A serialized occupancy grid: use it as the grid map.
    grid = archiveFrom(CCompressedInputStream(map_filename)).ReadObject()
    for i, m in enumerate(metric_map):
        if isinstance(m, COccupancyGridMap2D):
            metric_map[i] = grid
else:
    sys.exit(f"Error: unknown map file extension: {map_filename}")
print(f"Loaded map file {map_filename}: {metric_map}")

grid_map = next(m for m in metric_map if isinstance(m, COccupancyGridMap2D))

# The particle filter:
pdf = CMonteCarloLocalization2D()
pdf.options = mcl_options
pdf.options.metricMap = metric_map
pdf.resetUniformFreeSpace(grid_map, 0.7, particles_count)

pf = CParticleFilter()
pf.options = pf_options

# 3D view:
win3D = None
if not args.no_gui:
    from mrpt.gui import CDisplayWindow3D

    w, h = (int(v) for v in args.resolution.split("x"))
    win3D = CDisplayWindow3D("pf_localization", w, h)
    map_object = metric_map.getVisualization()

viz_options = VisualizationParameters()
viz_options.pointSize = 3
viz_options.showAxis = False

# Read the rawlog as a stream, one (action, sensory frame) pair at a time:
rawlog_arch = archiveFrom(CCompressedInputStream(rawlog_filename))
entry = 0
steps = 0
while True:
    read_ok, entry, actions, sf, obs = CRawlog.ReadFromArchive(rawlog_arch, entry)
    if not read_ok:
        break
    if actions is None or sf is None:
        continue  # this example needs rawlogs in the (action, sensory frame) format

    stats = pf.executeOn(pdf, actions, sf)
    cov, mean = pdf.getCovarianceAndMean()
    print(f"Entry {entry}: {len(pdf)} particles, mean={mean}, ESS={stats.ESS_beforeResample:.3f}")

    if win3D is not None:
        gl_obs = obs_to_viz(sf, viz_options)
        gl_obs.setPose(CPose3D(mean))
        gl_obs.setColor(TColorf(1.0, 0.0, 0.0))

        scene = win3D.get3DSceneAndLock()
        scene.clear()
        scene.insert(map_object)
        scene.insert(pdf.getVisualization())
        scene.insert(gl_obs)
        win3D.unlockAccess3DScene()
        win3D.forceRepaint()
        sleep(args.delay)

    steps += 1
    if args.max_steps and steps >= args.max_steps:
        break

print(f"\nDone. Final estimate: {pdf.getMean()}")
if win3D is not None:
    input("Press Enter to quit.")
