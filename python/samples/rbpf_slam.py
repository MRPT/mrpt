#!/usr/bin/env python

import sys
import pymrpt
import argparse

# args
parser = argparse.ArgumentParser()
parser.add_argument('rawlog', help='Rawlog file.')
parser.add_argument('-c', '--config', help='Config file.')
parser.add_argument('-o', '--output', help='Output occupancy grid as .png file.')
args = parser.parse_args()

# get filenames from args
rawlog_filename = args.rawlog
config_filename = args.config
output_filename = args.output

# load .rawlog
rawlog_file = pymrpt.utils.CFileGZInputStream(rawlog_filename)

# map builder
options = pymrpt.slam.CMetricMapBuilderRBPF.TConstructionOptions()
if config_filename:
    options.loadFromConfigFileName(config_filename, 'MappingApplication')
options.dumpToConsole()
map_builder = pymrpt.slam.CMetricMapBuilderRBPF(options)

# gui
win3D = pymrpt.gui.CDisplayWindow3D()

# loop
entry = 0
while True:
    # get action observation pair
    next_entry, act, obs, entry = pymrpt.obs.CRawlog.readActionObservationPair(rawlog_file, entry)
    if not next_entry: break
    else: entry += 1
    print 'Processing entry: {}.'.format(entry)

    # process
    map_builder.processActionObservation(act, obs)

    # get most likely map
    metric_map = map_builder.mapPDF.getCurrentMostLikelyMetricMap()

    # get pose estimation
    pose_est = map_builder.getCurrentPoseEstimation()
    particles_count = pose_est.ctx().particlesCount()

    # get particles paths
    lines_object = pymrpt.opengl.CSetOfLines.Create()
    for i in range(particles_count):
        path = map_builder.mapPDF.getPath(i)
        x = 0.0
        y = 0.0
        z = 0.0
        for p in path:
            lines_object.ctx().appendLine(x, y, z + 0.001, p.x, p.y, p.z + 0.001)
            x = float(p.x)
            y = float(p.y)
            z = float(p.z)

    # get ellipsoids at time t
    dummy_path = map_builder.mapPDF.getPath(0)
    k_s = range(len(dummy_path))
    k_s.reverse()
    ellipsoid_objects = []
    for k in k_s:
        pose_particles = map_builder.mapPDF.getEstimatedPosePDFAtTime(k);
        ellipsoid_object = pymrpt.opengl.CEllipsoid.Create()
        ellipsoid_object.ctx().setFromPosePDF(pose_particles)
        ellipsoid_objects.append(ellipsoid_object)

    # update gui
    map_object = metric_map.getAs3DObject()

    scene = win3D.get3DSceneAndLock()
    scene.clear()
    scene.insert(map_object)
    scene.insert(lines_object)
    for ellipsoid_object in ellipsoid_objects:
        scene.insert(ellipsoid_object)
    win3D.unlockAccess3DScene()
    win3D.forceRepaint()

# get last built map and save it as occupancy grid to file
simple_map = map_builder.getCurrentlyBuiltMap()
occ_map = pymrpt.maps.COccupancyGridMap2D()
occ_map.loadFromSimpleMap(simple_map)
if not output_filename:
    output_filename = 'occ_grid.png'
occ_map.saveAsBitmapFile(output_filename)

print 'Saved map as {}'.format(output_filename)
print
print 'Done.'
raw_input('Press key to quit.')
