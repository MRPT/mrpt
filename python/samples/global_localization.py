#!/usr/bin/env python

import os
import sys
import pymrpt
import argparse
from time import sleep

# args
parser = argparse.ArgumentParser()
parser.add_argument('config', help='Config file.')
parser.add_argument('-d', '--delay', help='Time delay in seconds. Default: 0.2')
parser.add_argument('-r', '--resolution', help='Window resolution. Default: 800x600')
args = parser.parse_args()

# get filenames from args
config_filename = args.config

# get configuration
if not os.path.exists(config_filename):
    print 'Error. Config file not found.'
    print 'Quit.'
    sys.exit(1)

config_file = pymrpt.utils.CConfigFile(config_filename)
print 'Load config file {}.'.format(config_filename)
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
        print 'Error. Rawlog file not found.'
        print 'Quit.'
        sys.exit(1)
    else:
        rawlog_filename = os.path.abspath(rawlog_filename)
        os.chdir(curr_dir)
rawlog_file = pymrpt.utils.CFileGZInputStream(rawlog_filename)
print 'Load rawlog file {}.'.format(rawlog_filename)

# map
if not os.path.exists(os.path.abspath(map_filename)):
    os.chdir(config_dir)
    if not os.path.exists(os.path.abspath(map_filename)):
        print 'Error. Map file not found.'
        print 'Quit.'
        sys.exit(1)
    else:
        map_filename = os.path.abspath(map_filename)
        os.chdir(curr_dir)

# options
# kld
pdf_prediction_options = pymrpt.slam.TMonteCarloLocalizationParams()
pdf_prediction_options.KLD_params.loadFromConfigFileName(config_filename, 'KLD_options')
pdf_prediction_options.KLD_params.dumpToConsole()
# pf
pf_options = pymrpt.bayes.CParticleFilter.TParticleFilterOptions()
pf_options.loadFromConfigFileName(config_filename, 'PF_options')
pf_options.dumpToConsole()
# maps
map_list = pymrpt.maps.TSetOfMetricMapInitializers()
map_list.loadFromConfigFileName(config_filename, 'MetricMap')
map_list.dumpToConsole()

## setup
metric_map = pymrpt.maps.CMultiMetricMap()
metric_map.setListOfMaps(map_list)


# load map
if (map_filename.endswith('.simplemap')
        or map_filename.endswith('.simplemap.gz')):
    simple_map = pymrpt.maps.CSimpleMap.Create()
    map_file = pymrpt.utils.CFileGZInputStream(map_filename)
    map_file.ReadObject(simple_map)
    metric_map.loadFromProbabilisticPosesAndObservations(simple_map.ctx())
elif (map_filename.endswith('.gridmap')
        or map_filename.endswith('.gridmap.gz')):
    occ_map = pymrpt.maps.COccupancyGridMap2D.Create()
    map_file = pymrpt.utils.CFileGZInputStream(map_filename)
    map_file.ReadObject(occ_map)
    # overwrite gridmap
    for i in range(len(metric_map.maps)):
        if metric_map.maps[i].ctx().GetRuntimeClass().className == 'COccupancyGridMap2D':
            metric_map.maps[i] = occ_map
else:
    print 'Error. Can not load map from unknown extension.'
    print 'Quit.'
    sys.exit(1)
print 'Load map file {}.'.format(map_filename)


# get window resolution
if args.resolution: resolution_str = args.resolution
else: resolution_str = '800x600'

# gui
try:
    win3D = pymrpt.gui.CDisplayWindow3D("pf_localization", int(resolution_str.split('x')[0]), int(resolution_str.split('x')[1]))
except:
    win3D = pymrpt.gui.CDisplayWindow3D("pf_localization", 800, 600)

# initial scene
map_object = metric_map.maps[0].ctx().getAs3DObject()

scene_ptr = win3D.get3DSceneAndLock()
scene_ptr.ctx().clear()
scene_ptr.ctx().insert(map_object)
win3D.unlockAccess3DScene()
win3D.forceRepaint()

# mcl
pdf = pymrpt.slam.CMonteCarloLocalization2D()
pdf.options = pdf_prediction_options
pdf.options.metricMap = metric_map

pf = pymrpt.bayes.CParticleFilter()
pf.m_options = pf_options

# initialize pdf
pdf.resetUniformFreeSpace(metric_map.maps[0].pointer(), 0.7, 40000)

# loop
entry = 0
while True:
    # get action observation pair
    next_entry, act, obs, entry = pymrpt.obs.CRawlog.readActionObservationPair(rawlog_file, entry)
    if not next_entry: break
    else: entry += 1
    print 'Processing entry: {}.'.format(entry)

    # get covariance and mean
    cov, mean = pdf.getCovarianceAndMean()

    # get particles
    particles_object = pdf.getAs3DObject()

    # get laserscan
    points_map = obs.buildAuxPointsMap()
    laserscan_object = points_map.getAs3DObject()
    laserscan_object.ctx().setPose(pymrpt.poses.CPose3D(mean))
    laserscan_object.ctx().setColor(pymrpt.utils.TColorf(1.,0.,0.))

    # update pf
    act_ptr = pymrpt.obs.CActionCollection.Create()
    act_ptr.ctx(act)
    obs_ptr = pymrpt.obs.CSensoryFrame.Create()
    obs_ptr.ctx(obs)
    stats = pf.executeOn(pdf, act_ptr, obs_ptr)

    # update scene
    scene_ptr = win3D.get3DSceneAndLock()
    scene_ptr.ctx().clear()
    scene_ptr.ctx().insert(map_object)
    scene_ptr.ctx().insert(particles_object)
    scene_ptr.ctx().insert(laserscan_object)
    win3D.unlockAccess3DScene()
    win3D.forceRepaint()

    # sleep
    if args.delay:
        try:
            sleep(float(args.delay))
        except:
            sleep(0.2)
    else: sleep(0.2)

print
print 'Done.'
raw_input('Press key to quit.')
