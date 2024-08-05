#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

# RBPF-SLAM: builds a gridmap from a 2D lidar dataset.
#
# Usage example:
#
# ./rbpf_slam.py --help
#
# ./rbpf_slam.py -c ../share/mrpt/config_files/rbpf-slam/gridmapping_optimal_sampling.ini  ../share/mrpt/datasets/2006-01ENE-21-SENA_Telecom\ Faculty_one_loop_only.rawlog
#

from mrpt.pymrpt import mrpt
import argparse

# args
parser = argparse.ArgumentParser()
parser.add_argument('rawlog', help='Rawlog file.')
parser.add_argument('-c', '--config', help='Config file.')
parser.add_argument(
    '-o', '--output', help='Save final map as to file with this prefix name.', default='final_map')
args = parser.parse_args()

# get filenames from args
rawlog_filename = args.rawlog
config_filename = args.config
output_filename = args.output

# Instance SLAM app, init, and run SLAM:
app = mrpt.apps.RBPF_SLAM_App_Rawlog()

app.init(iniConfigFile=config_filename, rawlogFile=rawlog_filename)

app.run()

# Get last built map
app.mapBuilder.saveCurrentMapToFile('final_map.simplemap')

# Save map:
print('saving final map to: "{}_*"'.format(output_filename))

metricMap = app.mapBuilder.getCurrentlyBuiltMetricMap()
metricMap.saveMetricMapRepresentationToFile(output_filename)
