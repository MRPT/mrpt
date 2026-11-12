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

print("CPointsMap nearest-neighbor queries")
from mrpt.math import TPoint2D
from mrpt.poses import CPose2D
from mrpt.maps import TVoxelMap_InsertionOptions, TVoxelMap_LikelihoodOptions
check("nn_single_search on an empty map", CSimplePointsMap().nn_single_search(TPoint3D(0, 0, 0)) is None)
rng_ = np.random.default_rng(1)
cloud_pts = rng_.uniform(-5, 5, (300, 3)).astype(np.float32)
nn_map = CSimplePointsMap()
nn_map.setPointsFromNumpy(cloud_pts)
q = np.array([1.0, -0.5, 0.25])
d2_all = ((cloud_pts - q)**2).sum(axis=1)
d2_all_2d = ((cloud_pts[:, :2] - q[:2])**2).sum(axis=1)
idx, pt, d2 = nn_map.nn_single_search(TPoint3D(*q))
check("nn_single_search 3D", idx == np.argmin(d2_all) and abs(d2 - d2_all.min()) < 1e-4 and
      abs(pt.x - cloud_pts[idx, 0]) < 1e-6, f"got {idx}, {d2}")
idx, pt, d2 = nn_map.nn_single_search(TPoint2D(q[0], q[1]))
check("nn_single_search 2D", idx == np.argmin(d2_all_2d) and abs(d2 - d2_all_2d.min()) < 1e-4)
idxs, pts_, d2s = nn_map.nn_multiple_search(TPoint3D(*q), 5)
check("nn_multiple_search 3D", list(idxs) == list(np.argsort(d2_all)[:5]) and
      pts_.shape == (5, 3) and np.allclose(pts_, cloud_pts[idxs]) and
      np.allclose(d2s, np.sort(d2_all)[:5], atol=1e-4), f"got {idxs}")
idxs, pts_, d2s = nn_map.nn_multiple_search(TPoint2D(q[0], q[1]), 4)
check("nn_multiple_search 2D", list(idxs) == list(np.argsort(d2_all_2d)[:4]) and
      pts_.shape == (4, 2))
idxs, pts_, d2s = nn_map.nn_radius_search(TPoint3D(*q), 4.0)
check("nn_radius_search 3D", sorted(idxs) == sorted(np.where(d2_all < 4.0)[0]) and
      np.all(d2s < 4.0) and pts_.shape == (len(idxs), 3), f"got {sorted(idxs)}")
idxs, _, _ = nn_map.nn_radius_search(TPoint3D(*q), 4.0, maxPoints=2)
check("nn_radius_search maxPoints", len(idxs) == 2)
idxs, _, _ = nn_map.nn_radius_search(TPoint2D(q[0], q[1]), 1.0)
check("nn_radius_search 2D", sorted(idxs) == sorted(np.where(d2_all_2d < 1.0)[0]))

print("COccupancyGridMap2D setFromNumpy / laserScanSimulator / options")
sim_grid = COccupancyGridMap2D(-5.0, 5.0, -5.0, 5.0, 0.1)
cells = sim_grid.getAsNumpy()
cells[:, :] = 1.0  # all free
cells[:, sim_grid.x2idx(2.05)] = 0.0  # a wall along the cells in x=[2.0, 2.1]
sim_grid.setFromNumpy(cells)
check("setFromNumpy", np.allclose(sim_grid.getAsNumpy(), cells, atol=0.01))
try:
    sim_grid.setFromNumpy(np.zeros((3, 4), dtype=np.float32))
    check("setFromNumpy wrong shape", False, "no exception")
except ValueError:
    check("setFromNumpy wrong shape", True)
sim_scan = CObservation2DRangeScan()
sim_scan.aperture = math.pi
sim_scan.maxRange = 10.0
sim_grid.laserScanSimulator(sim_scan, CPose2D(0.05, 0.05, 0.0), N=181)
sim_r = sim_scan.getScanRangesAsNumpy()
sim_v = sim_scan.getValidRangesAsNumpy()
check("laserScanSimulator size", sim_scan.getScanSize() == 181)
check("laserScanSimulator front ray", sim_v[90] and abs(sim_r[90] - 1.95) < 1e-3, f"got {sim_r[90]}")
check("laserScanSimulator 45 deg ray", sim_v[45] and
      abs(sim_r[45] - 1.95 / math.cos(math.pi / 4)) < 1e-3, f"got {sim_r[45]}")
check("laserScanSimulator ray without obstacle", not sim_v[0])

io = sim_grid.insertionOptions
check("grid insertionOptions defaults", abs(io.maxDistanceInsertion - 15.0) < 1e-6 and
      io.decimation == 1 and io.considerInvalidRangesAsFreeSpace)
io_grid = COccupancyGridMap2D(-5.0, 5.0, -5.0, 5.0, 0.1)
io_grid.insertionOptions.maxDistanceInsertion = 1.0
check("grid insertionOptions in place", abs(io_grid.insertionOptions.maxDistanceInsertion - 1.0) < 1e-6)
io_grid.insertObservation(make_scan(rng=3.0))
check("maxDistanceInsertion effect", io_grid.getPos(0.5, 0.0) > 0.5 and
      abs(io_grid.getPos(2.0, 0.0) - 0.5) < 0.01, f"got {io_grid.getPos(2.0, 0.0)}")
lo = COccupancyGridMap2D.TLikelihoodOptions()
check("TLikelihoodOptions defaults", lo.likelihoodMethod ==
      COccupancyGridMap2D.TLikelihoodMethod.lmLikelihoodField_Thrun and lo.LF_decimation == 5)
lo.likelihoodMethod = COccupancyGridMap2D.lmRayTracing
lo.OWA_weights = [0.5, 0.3, 0.2]
sim_grid.likelihoodOptions = lo
check("grid likelihoodOptions assignment", sim_grid.likelihoodOptions.likelihoodMethod ==
      COccupancyGridMap2D.lmRayTracing and np.allclose(sim_grid.likelihoodOptions.OWA_weights,
                                                      [0.5, 0.3, 0.2]))

print("COccupancyGridMap3D occupied voxels / options")
g3d = COccupancyGridMap3D(TPoint3D(-1, -1, -1), TPoint3D(1, 1, 1), 0.25)
check("no occupied voxels", g3d.getOccupiedVoxelsAsNumpy().shape == (0, 3))
g3d.setFreenessByPos(0.6, -0.4, 0.1, 0.1)
g3d.setFreenessByPos(-0.9, 0.9, -0.9, 0.3)
g3d.setFreenessByPos(0.1, 0.1, 0.1, 0.55)
vox3 = g3d.getOccupiedVoxelsAsNumpy()
expected3 = np.array([[-0.875, 0.875, -0.875], [0.625, -0.375, 0.125]], dtype=np.float32)
check("getOccupiedVoxelsAsNumpy", vox3.shape == (2, 3) and
      np.allclose(vox3[np.lexsort(vox3.T[::-1])], expected3), f"got {vox3}")
check("getOccupiedVoxelsAsNumpy threshold", g3d.getOccupiedVoxelsAsNumpy(0.8).shape == (1, 3))
check("grid3D options", g3d.insertionOptions.decimation_3d_range == 8 and
      g3d.likelihoodOptions.likelihoodMethod == COccupancyGridMap3D.lmLikelihoodField_Thrun)
g3d.insertionOptions.raytraceEmptyCells = False
check("grid3D options in place", not g3d.insertionOptions.raytraceEmptyCells)

print("COctoMap occupied voxels / options")
om2 = COctoMap(0.1)
check("octomap insertionOptions defaults", abs(om2.insertionOptions.probHit - 0.7) < 1e-6 and
      abs(om2.insertionOptions.probMiss - 0.4) < 1e-6 and om2.insertionOptions.pruning)
om2.insertionOptions.probHit = 0.9
om2.updateVoxel(0.55, 0.55, 0.55, True)
check("octomap probHit effect", abs(om2.getPointOccupancy(0.55, 0.55, 0.55) - 0.9) < 1e-5,
      f"got {om2.getPointOccupancy(0.55, 0.55, 0.55)}")
om2.updateVoxel(-0.25, 0.35, 0.05, True)
om2.updateVoxel(1.05, 1.05, 1.05, False)
centers, sizes = om2.getOccupiedVoxelsAsNumpy()
check("COctoMap.getOccupiedVoxelsAsNumpy", centers.shape == (2, 3) and np.allclose(sizes, 0.1) and
      np.allclose(centers[np.argsort(centers[:, 0])], [[-0.25, 0.35, 0.05], [0.55, 0.55, 0.55]],
                  atol=1e-5), f"got {centers}")
om2.likelihoodOptions.decimation = 3
check("octomap likelihoodOptions", om2.likelihoodOptions.decimation == 3)

print("CVoxelMap / CHeightGridMap2D options")
vmo = CVoxelMap(0.1)
check("voxel insertionOptions defaults", abs(vmo.insertionOptions.prob_hit - 0.65) < 1e-12 and
      isinstance(vmo.insertionOptions, TVoxelMap_InsertionOptions) and
      isinstance(vmo.likelihoodOptions, TVoxelMap_LikelihoodOptions))
vmo.insertionOptions.prob_hit = 0.9
vmo.updateVoxel(0.05, 0.05, 0.05, True)
check("voxel prob_hit effect", abs(vmo.getPointOccupancy(0.05, 0.05, 0.05) - 0.9) < 0.01)
check("CVoxelMapRGB options", abs(CVoxelMapRGB(0.1).likelihoodOptions.occupiedThreshold - 0.6) < 1e-12)
hmo = CHeightGridMap2D()
hmo.insertionOptions.filterByHeight = True
hmo.insertionOptions.z_max = 0.2
check("height map insertionOptions", hmo.insertionOptions.filterByHeight and
      abs(hmo.insertionOptions.z_max - 0.2) < 1e-6)

print("Point fields as NumPy arrays")
import os
import tempfile
fields_in = {
    "x": np.arange(6, dtype=np.float32),
    "y": np.linspace(-1, 1, 6).astype(np.float32),
    "z": np.full(6, 0.5, dtype=np.float32),
    "intensity": np.linspace(0, 1, 6).astype(np.float32),
    "timestamp": np.arange(6) * 0.1,
    "ring": np.arange(6, dtype=np.uint16) * 100,
    "label": np.arange(6, dtype=np.uint8),
    "id": np.arange(6, dtype=np.uint32) * 1000000,
}
gpm = CGenericPointsMap.FromNumpyFields(fields_in)
check("FromNumpyFields size", len(gpm) == 6)
check("FromNumpyFields registers fields by dtype",
      gpm.getPointFieldNames_double() == ["timestamp"] and gpm.getPointFieldNames_uint16() == ["ring"]
      and gpm.getPointFieldNames_uint8() == ["label"] and gpm.getPointFieldNames_uint32() == ["id"]
      and "intensity" in gpm.getPointFieldNames_float())
fields_out = gpm.getPointFieldsAsNumpy()
check("getPointFieldsAsNumpy keys", set(fields_out) == set(fields_in), f"got {set(fields_out)}")
check("getPointFieldsAsNumpy values and dtypes",
      all(fields_out[k].dtype == v.dtype and np.array_equal(fields_out[k], v)
          for k, v in fields_in.items()))
check("per-point getters agree", abs(gpm.getPointField_double(3, "timestamp") - 0.3) < 1e-12 and
      gpm.getPointField_uint32(5, "id") == 5000000)
fields_out["x"][0] = 99.0
check("getPointFieldsAsNumpy returns copies", gpm.getPointsAsNumpy()[0, 0] == 0.0)
check("getPointFieldsAsNumpy of a CSimplePointsMap",
      set(CSimplePointsMap().getPointFieldsAsNumpy()) == {"x", "y", "z"})
idx, _, _ = gpm.nn_single_search(TPoint3D(4.1, 0.6, 0.5))
check("KD-tree after FromNumpyFields", idx == 4, f"got {idx}")
for bad, exc in [({"x": [1.0], "y": [1.0]}, KeyError),
                 ({"x": [1.0, 2.0], "y": [1.0], "z": [0.0]}, ValueError),
                 ({"x": [1.0], "y": [1.0], "z": [0.0], "f": np.array([1], dtype=np.int64)},
                  TypeError)]:
    try:
        CGenericPointsMap.FromNumpyFields(bad)
        check(f"FromNumpyFields rejects {exc.__name__}", False, "no exception")
    except exc:
        check(f"FromNumpyFields rejects {exc.__name__}", True)

big = CSimplePointsMap()
big_pts = np.random.default_rng(3).uniform(-10, 10, (200000, 3)).astype(np.float32)
big.setPointsFromNumpy(big_pts)
check("setPointsFromNumpy / getPointsAsNumpy round trip",
      len(big) == 200000 and np.array_equal(big.getPointsAsNumpy(), big_pts))
idx, _, _ = big.nn_single_search(TPoint3D(*big_pts[1234].astype(float)))
check("KD-tree updated after setPointsFromNumpy", idx == 1234, f"got {idx}")
big.setPointsFromNumpy(big_pts[:10] + 100.0)
idx, _, d2 = big.nn_single_search(TPoint3D(*(big_pts[7] + 100.0).astype(float)))
check("KD-tree rebuilt after a second setPointsFromNumpy", idx == 7 and d2 < 1e-6)
try:
    big.setPointsFromNumpy(np.zeros((4, 2), dtype=np.float32))
    check("setPointsFromNumpy rejects a wrong shape", False, "no exception")
except ValueError:
    check("setPointsFromNumpy rejects a wrong shape", True)

print("PLY and KITTI files")
with tempfile.TemporaryDirectory() as tmp:
    ply = os.path.join(tmp, "cloud.ply")
    check("saveToPlyFile", gpm.saveToPlyFile(ply, save_in_binary=True))
    loaded = CSimplePointsMap()
    check("loadFromPlyFile", loaded.loadFromPlyFile(ply) and
          np.allclose(loaded.getPointsAsNumpy(), gpm.getPointsAsNumpy()))
    check("loadFromPlyFile of a missing file", not loaded.loadFromPlyFile(os.path.join(tmp, "no.ply"))
          and loaded.getLoadPLYErrorString() != "")
    check("saveToPlyFile to a bad path", not gpm.saveToPlyFile(os.path.join(tmp, "no", "a.ply")))
    raw = os.path.join(tmp, "000000.bin")
    xyzi = np.column_stack([np.arange(4), np.ones(4), np.zeros(4), [0.1, 0.2, 0.3, 0.4]])
    xyzi.astype(np.float32).tofile(raw)
    kitti = CGenericPointsMap()
    check("loadFromKittiVelodyneFile", kitti.loadFromKittiVelodyneFile(raw) and len(kitti) == 4 and
          np.allclose(kitti.getPointFieldsAsNumpy()["intensity"], [0.1, 0.2, 0.3, 0.4]))
    gz = os.path.join(tmp, "scan.bin.gz")
    check("saveToKittiVelodyneFile", kitti.saveToKittiVelodyneFile(gz))
    kitti2 = CGenericPointsMap()
    check("KITTI round trip", kitti2.loadFromKittiVelodyneFile(gz) and
          np.allclose(kitti2.getPointsAsNumpy(), kitti.getPointsAsNumpy()))

print("GetRuntimeClass of maps")
import gc
expected_names = {
    CSimplePointsMap: "mrpt::maps::CSimplePointsMap",
    CGenericPointsMap: "mrpt::maps::CGenericPointsMap",
    COccupancyGridMap2D: "mrpt::maps::COccupancyGridMap2D",
    COctoMap: "mrpt::maps::COctoMap",
    CMultiMetricMap: "mrpt::maps::CMultiMetricMap",
}
names_ok = True
for _ in range(200):
    for cls, name in expected_names.items():
        rc = cls().GetRuntimeClass()  # a static object: Python must not free it
        names_ok = names_ok and rc.className == name
        del rc
    gc.collect()
check("GetRuntimeClass called many times", names_ok)
rc = CSimplePointsMap().GetRuntimeClass()
check("GetRuntimeClass base class", rc.derivedFrom("CPointsMap") and
      rc.getBaseClass().className == "mrpt::maps::CPointsMap")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
