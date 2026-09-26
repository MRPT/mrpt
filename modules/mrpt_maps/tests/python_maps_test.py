#!/usr/bin/env python3
"""Smoke tests for mrpt.maps Python bindings."""
import sys

try:
    from mrpt.maps import CSimplePointsMap, CGenericPointsMap, COccupancyGridMap2D
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.maps bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.maps import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CSimplePointsMap")
m = CSimplePointsMap()
check("empty size", m.size() == 0)

pts = np.array([[1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0]], dtype=np.float32)
m.setPointsFromNumpy(pts)
check("size after insert", m.size() == 3)

out = m.getPointsAsNumpy()
check("getPointsAsNumpy shape", out.shape == (3, 3), f"got {out.shape}")
check("first point x", abs(float(out[0, 0]) - 1.0) < 1e-5)

m.insertPoint(10.0, 11.0, 12.0)
check("insertPoint", m.size() == 4)

m.clear()
check("clear", m.size() == 0)

print("CGenericPointsMap")
gm = CGenericPointsMap()
check("register uint32 field", gm.registerField_uint32("rgba") is True)
gm.resize(3)
gm.setPointField_uint32(1, "rgba", 4000000000)
check("get uint32 field", gm.getPointField_uint32(1, "rgba") == 4000000000)
check("uint32 field name listed", "rgba" in gm.getPointFieldNames_uint32())
check("hasPointField", gm.hasPointField("rgba"))
check("unregister field", gm.unregisterField("rgba") is True)
check("field removed", not gm.hasPointField("rgba"))

print("COccupancyGridMap2D")
grid = COccupancyGridMap2D(-5.0, 5.0, -5.0, 5.0, 0.05)
check("grid non-empty", grid.getSizeX() > 0 and grid.getSizeY() > 0)
check("resolution", abs(grid.getResolution() - 0.05) < 1e-6)

arr = grid.getAsNumpy()
check("getAsNumpy 2D", arr.ndim == 2, f"got ndim={arr.ndim}")

import math
from mrpt.maps import (
    CMultiMetricMap, TSetOfMetricMapInitializers, TMetricMapInitializer, CSimpleMap,
    CVoxelMap, CVoxelMapRGB, COccupancyGridMap3D, CHeightGridMap2D, CBeaconMap, COctoMap,
    CObservationPointCloud, VisualizationParameters, obs_to_viz, CMetricMap,
)
from mrpt.obs import CObservation2DRangeScan, CSensoryFrame
from mrpt.poses import CPose3D, CPose3DPDFGaussian
from mrpt.config import CConfigFileMemory
from mrpt.viz import CSetOfObjects
from mrpt.math import TPoint3D


def make_scan(n=181, rng=3.0):
    sc = CObservation2DRangeScan()
    sc.aperture = math.pi
    sc.maxRange = 10.0
    sc.resizeScan(n)
    for i in range(n):
        sc.setScanRange(i, rng)
        sc.setScanRangeValidity(i, True)
    return sc


print("TSetOfMetricMapInitializers / CMultiMetricMap")
cfg = CConfigFileMemory(
    "[MetricMap]\noccupancyGrid_count=1\npointsMap_count=1\n"
    "[MetricMap_occupancyGrid_00_creationOpts]\n"
    "min_x=-5\nmax_x=5\nmin_y=-5\nmax_y=5\nresolution=0.1\n")
inits = TSetOfMetricMapInitializers()
inits.loadFromConfigFile(cfg, "MetricMap")
check("two map definitions", len(inits) == 2, f"got {len(inits)}")
names = sorted(i.getMetricMapClassName() for i in inits)
check("definition class names", names == ["mrpt::maps::COccupancyGridMap2D", "mrpt::maps::CSimplePointsMap"], f"got {names}")
mm = CMultiMetricMap(inits)
check("multimap size", len(mm) == 2)
check("maps are typed", any(isinstance(x, COccupancyGridMap2D) for x in mm)
      and any(isinstance(x, CSimplePointsMap) for x in mm.maps))
check("factory", TMetricMapInitializer.factory("CVoxelMap").getMetricMapClassName() == "mrpt::maps::CVoxelMap")

sf = CSensoryFrame()
sf.insert(make_scan())
check("insertObs(sf)", mm.insertObs(sf))
pts_map = next(x for x in mm if isinstance(x, CSimplePointsMap))
check("points inserted", len(pts_map) > 100, f"got {len(pts_map)}")
grid_map = next(x for x in mm if isinstance(x, COccupancyGridMap2D))
lik_true = grid_map.computeObservationLikelihood(sf[0], CPose3D())
lik_off = grid_map.computeObservationLikelihood(sf[0], CPose3D(1.0, 0.7, 0.0, 0.5, 0.0, 0.0))
check("likelihood peaks at the true pose", lik_true > lik_off, f"{lik_true} vs {lik_off}")
check("computeObservationsLikelihood", grid_map.computeObservationsLikelihood(sf, CPose3D()) > lik_off)
viz = mm.getVisualization()
check("getVisualization", isinstance(viz, CSetOfObjects) and len(viz) > 0)
bb = pts_map.boundingBox()
check("boundingBox", bb.max.x <= 3.01 and bb.min.x >= -3.01 and bb.max.y > 2.9, f"{bb.min} {bb.max}")
new_pts = CSimplePointsMap()
idx = [i for i, x in enumerate(mm) if isinstance(x, CSimplePointsMap)][0]
mm[idx] = new_pts
check("__setitem__", len(mm[idx]) == 0)

print("loadFromSimpleMap")
smap = CSimpleMap()
for i in range(3):
    kf_sf = CSensoryFrame()
    kf_sf.insert(make_scan())
    smap.insert(CPose3DPDFGaussian(CPose3D(float(i), 0.0, 0.0, 0.0, 0.0, 0.0)), kf_sf)
pm = CSimplePointsMap()
pm.loadFromSimpleMap(smap)
check("loadFromSimpleMap", len(pm) > 300, f"got {len(pm)}")

print("CVoxelMap / CVoxelMapRGB")
vm = CVoxelMap(0.1)
for _ in range(5):
    vm.updateVoxel(1.0, 1.0, 1.0, True)
occ = vm.getPointOccupancy(1.0, 1.0, 1.0)
check("voxel occupied", occ is not None and occ > 0.5, f"got {occ}")
check("unobserved voxel -> None", vm.getPointOccupancy(-3.0, -3.0, -3.0) is None)
check("occupied voxels", len(vm.getOccupiedVoxels()) >= 1)
cloud = CSimplePointsMap()
cloud.insertPoint(2.0, 0.0, 0.0)
vm2 = CVoxelMap(0.2)
vm2.insertPointCloudAsRays(cloud, TPoint3D(0, 0, 0))
p_free = vm2.getPointOccupancy(1.0, 0.0, 0.0)
p_occ = vm2.getPointOccupancy(2.0, 0.0, 0.0)
check("ray insertion", p_free is not None and p_occ is not None and p_free < 0.5 < p_occ,
      f"free={p_free} occ={p_occ}")
check("CVoxelMapRGB", CVoxelMapRGB(0.1).isEmpty())

print("COccupancyGridMap3D")
g3 = COccupancyGridMap3D(TPoint3D(-1, -1, -1), TPoint3D(1, 1, 1), 0.5)
check("3D grid size", (g3.getSizeX(), g3.getSizeY(), g3.getSizeZ()) == (4, 4, 4),
      f"got {(g3.getSizeX(), g3.getSizeY(), g3.getSizeZ())}")
g3.setFreenessByPos(0.1, 0.1, 0.1, 0.1)
check("3D freeness", abs(g3.getFreenessByPos(0.1, 0.1, 0.1) - 0.1) < 0.02)

print("CHeightGridMap2D")
hm = CHeightGridMap2D(-2.0, 2.0, -2.0, 2.0, 0.5)
check("insertIndividualPoint", hm.insertIndividualPoint(0.6, 0.6, 1.25))
h = hm.getHeight(0.6, 0.6)
check("getHeight", h is not None and abs(h - 1.25) < 1e-6, f"got {h}")
harr = hm.getAsNumpy()
check("height array NaNs", harr.shape == (hm.getSizeY(), hm.getSizeX())
      and np.isnan(harr).sum() == harr.size - 1)

print("COctoMap")
om = COctoMap(0.1)
for _ in range(3):
    om.updateVoxel(0.5, 0.5, 0.5, True)
occ = om.getPointOccupancy(0.5, 0.5, 0.5)
check("octomap occupancy", occ is not None and occ > 0.5, f"got {occ}")

print("CBeaconMap")
check("empty beacon map", len(CBeaconMap()) == 0)

print("CObservationPointCloud")
opc = CObservationPointCloud()
opc.pointcloud = cloud
target = CSimplePointsMap()
check("insert point cloud obs", target.insertObservation(opc) and len(target) == 1)

print("obs_to_viz")
vp = VisualizationParameters()
vp.pointSize = 3
check("obs_to_viz(obs)", len(obs_to_viz(make_scan(), vp)) > 0)
out = CSetOfObjects()
check("obs_to_viz(sf, out)", obs_to_viz(sf, vp, out) is out and len(out) > 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
