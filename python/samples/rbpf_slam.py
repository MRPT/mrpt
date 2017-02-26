#!/usr/bin/env python

import sys
import pymrpt
import argparse

# args
parser = argparse.ArgumentParser()
parser.add_argument('rawlog', help='Rawlog file.')
parser.add_argument('-c', '--config', help='Config file.')
parser.add_argument('-o', '--output', help='Save occupancy grid as .gridmap file.')
parser.add_argument('-i', '--image', help='Save occupancy grid as image (.png, .bmp) file.')
parser.add_argument('-r', '--resolution', help='Window resolution. Default: 800x600')
args = parser.parse_args()

# get filenames from args
rawlog_filename = args.rawlog
config_filename = args.config
output_filename = args.output
image_filename = args.image

# load .rawlog
rawlog_file = pymrpt.utils.CFileGZInputStream(rawlog_filename)

# map builder
options = pymrpt.slam.CMetricMapBuilderRBPF.TConstructionOptions()
if config_filename:
    options.loadFromConfigFileName(config_filename, 'MappingApplication')
options.dumpToConsole()
map_builder = pymrpt.slam.CMetricMapBuilderRBPF(options)

# get window resolution
if args.resolution: resolution_str = args.resolution
else: resolution_str = '800x600'

# gui
try:
    win3D = pymrpt.gui.CDisplayWindow3D("rbpf_slam", int(resolution_str.split('x')[0]), int(resolution_str.split('x')[1]))
except:
    win3D = pymrpt.gui.CDisplayWindow3D("rbpf_slam", 800, 600)

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
    lines_object.ctx().setColor(pymrpt.utils.TColorf(1.,0.,0.))

    # get ellipsoids at time t
    dummy_path = map_builder.mapPDF.getPath(0)
    k_s = range(len(dummy_path))
    k_s.reverse()
    ellipsoid_objects = []
    for k in k_s:
        pose_particles = map_builder.mapPDF.getEstimatedPosePDFAtTime(k);
        ellipsoid_object = pymrpt.opengl.CEllipsoid.Create()
        ellipsoid_object.ctx().setFromPosePDF(pose_particles)
        ellipsoid_object.ctx().setColor(pymrpt.utils.TColorf(0.,1.,0.))
        ellipsoid_objects.append(ellipsoid_object)

    # update gui
    map_object = metric_map.getAs3DObject()

    scene = win3D.get3DSceneAndLock()
    scene.ctx().clear()
    scene.ctx().insert(map_object)
    scene.ctx().insert(lines_object)
    for ellipsoid_object in ellipsoid_objects:
        scene.ctx().insert(ellipsoid_object)
    win3D.unlockAccess3DScene()
    win3D.forceRepaint()

# get last built map and save it as occupancy grid to file
simple_map = map_builder.getCurrentlyBuiltMap()
occ_map = pymrpt.maps.COccupancyGridMap2D()
occ_map.loadFromSimpleMap(simple_map)
if output_filename:
    occ_map.saveAsBitmapFile(output_filename)
    print 'Saved map as {}'.format(output_filename)
if image_filename:
    occ_map.saveAsBitmapFile(image_filename)
    print 'Saved map image as {}'.format(output_filename)
print
print 'Done.'
raw_input('Press key to quit.')
