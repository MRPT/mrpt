#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

#
# Usage example:
#
# ./global_localization.py ../share/mrpt/config_files/pf-localization/localization_demo.ini
#
from mrpt.pymrpt import mrpt
import os
import sys
import argparse
from time import sleep


# args
parser = argparse.ArgumentParser()
parser.add_argument('config', help='Config file.')
parser.add_argument(
    '-d', '--delay', help='Time delay in seconds. Default: 0.2')
parser.add_argument('-r', '--resolution',
                    help='Window resolution. Default: 800x600')
args = parser.parse_args()

# get filenames from args
config_filename = args.config

# get configuration
if not os.path.exists(config_filename):
    print('Error. Config file not found.')
    print('Quit.')
    sys.exit(1)

config_file = mrpt.config.CConfigFile(config_filename)
print('Load config file {}.'.format(config_filename))
sec_name = 'LocalizationExperiment'

rawlog_filename = config_file.read_string(sec_name, "rawlog_file", "")
map_filename = config_file.read_string(sec_name, "map_file", "")


# default filenames are relative so we need to change our dir to supplied config file dir
curr_dir = os.path.abspath(os.curdir)
config_dir = os.path.dirname(os.path.abspath(config_filename))
# rawlog
if not os.path.exists(os.path.abspath(rawlog_filename)):
    os.chdir(config_dir)
    if not os.path.exists(os.path.abspath(rawlog_filename)):
        print('Error. Rawlog file not found.')
        print('Quit.')
        sys.exit(1)
    else:
        rawlog_filename = os.path.abspath(rawlog_filename)
        os.chdir(curr_dir)
rawlog_file = mrpt.io.CFileGZInputStream(rawlog_filename)
print('Load rawlog file {}.'.format(rawlog_filename))

# map
if not os.path.exists(os.path.abspath(map_filename)):
    os.chdir(config_dir)
    if not os.path.exists(os.path.abspath(map_filename)):
        print('Error. Map file not found.')
        print('Quit.')
        sys.exit(1)
    else:
        map_filename = os.path.abspath(map_filename)
        os.chdir(curr_dir)

# Load parameters:
# KLD (Adapative sampling)
pdf_prediction_options = mrpt.slam.TMonteCarloLocalizationParams()
pdf_prediction_options.KLD_params.loadFromConfigFileName(
    config_filename, 'KLD_options')
pdf_prediction_options.KLD_params.dumpToConsole()

# Particle filtering itself:
pf_options = mrpt.bayes.CParticleFilter.TParticleFilterOptions()
pf_options.loadFromConfigFileName(config_filename, 'PF_options')
pf_options.dumpToConsole()

# Metric maps to build:
map_list = mrpt.maps.TSetOfMetricMapInitializers()
map_list.loadFromConfigFileName(config_filename, 'MetricMap')
map_list.dumpToConsole()

# setup
metric_map = mrpt.maps.CMultiMetricMap()
metric_map.setListOfMaps(map_list)

# load map
map_file = mrpt.io.CFileGZInputStream(map_filename)
map_arch = mrpt.serialization.archiveFrom(map_file)

if (map_filename.endswith('.simplemap')
        or map_filename.endswith('.simplemap.gz')):
    simple_map = mrpt.maps.CSimpleMap()
    map_arch.ReadObject(simple_map)
    metric_map.loadFromProbabilisticPosesAndObservations(simple_map)
elif (map_filename.endswith('.gridmap')
        or map_filename.endswith('.gridmap.gz')):
    occ_map = mrpt.maps.COccupancyGridMap2D()
    map_arch.ReadObject(occ_map)
    # overwrite gridmap
    for i in range(len(metric_map.maps)):
        if metric_map.maps[i].GetRuntimeClass().className == 'COccupancyGridMap2D':
            metric_map.maps[i] = occ_map
else:
    print('Error. Can not load map from unknown extension.')
    print('Quit.')
    sys.exit(1)
print('Load map file {}.'.format(map_filename))

# get window resolution
if args.resolution:
    resolution_str = args.resolution
else:
    resolution_str = '800x600'

# gui
try:
    res = resolution_str.split('x')
    win3D = mrpt.gui.CDisplayWindow3D(
        "pf_localization", int(res[0]), int(res[1]))
except:
    win3D = mrpt.gui.CDisplayWindow3D("pf_localization", 800, 600)

# initial scene
map_object = metric_map.getVisualization()

scene_ptr = win3D.get3DSceneAndLock()
scene_ptr.clear()
scene_ptr.insert(map_object)
win3D.unlockAccess3DScene()
win3D.forceRepaint()

# mcl
pdf = mrpt.slam.CMonteCarloLocalization2D()
pdf.options = pdf_prediction_options
pdf.options.metricMap = metric_map

pf = mrpt.bayes.CParticleFilter()
pf.m_options = pf_options

# initialize pdf
pdf.resetUniformFreeSpace(metric_map.maps[0], 0.7, 40000)

# Archive for reading from the file:
rawlogArch = mrpt.serialization.archiveFrom(rawlog_file)

# loop
entry = 0
while True:
    # get action observation pair
    [readOk, entry, act, sf, obs] = mrpt.obs.CRawlog.ReadFromArchive(
        rawlogArch, entry)
    if not readOk:
        break

    print('Processing entry: {}.'.format(entry))

    # get covariance and mean
    cov, mean = pdf.getCovarianceAndMean()

    # get particles
    particles_object = pdf.getVisualization()

    # get visualization for laser scan (two ways for demonstration purposes)
    if False:
        # Alternative method 1:
        # Insert the observations into a point cloud:
        points_map = mrpt.maps.CSimplePointsMap()
        points_map.insertObs(sf)

        # And get the visualization of the point cloud:
        glObservation = points_map.getVisualization()
    else:
        # Alternative 2:
        # Using the generic obs_to_viz() method:
        vizOpts = mrpt.obs.VisualizationParameters()
        vizOpts.pointSize = 3
        vizOpts.showAxis = False

        glObservation = mrpt.opengl.CSetOfObjects()

        mrpt.obs.obs_to_viz(sf, vizOpts, glObservation)

    glObservation.setPose(mrpt.poses.CPose3D(mean))
    glObservation.setColor(mrpt.img.TColorf(1., 0., 0.))

    # update pf
    stats = pf.executeOn(pdf, act, sf)

    # update scene
    scene_ptr = win3D.get3DSceneAndLock()
    scene_ptr.clear()
    scene_ptr.insert(map_object)
    scene_ptr.insert(particles_object)
    scene_ptr.insert(glObservation)
    win3D.unlockAccess3DScene()
    win3D.forceRepaint()

    # sleep
    if args.delay:
        try:
            sleep(float(args.delay))
        except:
            sleep(0.2)
    else:
        sleep(0.2)

print()
print('Done.')
input('Press any key to quit.')
