from . import _bindings as _b

# Core Classes
Scene = _b.Scene
Viewport = _b.Viewport
CSetOfObjects = _b.CSetOfObjects
CCamera = _b.CCamera
CPointCloud = _b.CPointCloud
CPointCloudColoured = _b.CPointCloudColoured
CAssimpModel = _b.CAssimpModel
AssimpLoadFlags = _b.AssimpLoadFlags

# Phase 0.4 — New classes
CGridPlaneXY = _b.CGridPlaneXY
CGridPlaneXZ = _b.CGridPlaneXZ
CAxis = _b.CAxis
CBox = _b.CBox
CSphere = _b.CSphere
CCylinder = _b.CCylinder
CArrow = _b.CArrow
CText = _b.CText
CText3D = _b.CText3D
CSetOfLines = _b.CSetOfLines
CSimpleLine = _b.CSimpleLine
CEllipsoid2D = _b.CEllipsoid2D
CEllipsoid3D = _b.CEllipsoid3D

# stock_objects submodule
stock_objects = _b.stock_objects

# 1. Make Scene building intuitive with '<<' style (using __lshift__)


def _scene_lshift(self, obj):
    self.insert(obj)
    return self


Scene.__lshift__ = _scene_lshift
CSetOfObjects.__lshift__ = _scene_lshift
Viewport.__lshift__ = _scene_lshift

# 2. Add shorthand for creating colored points


def create_point_cloud(pts_array, color=(255, 255, 255)):
    pc = CPointCloud()
    pc.setPoints(pts_array)
    pc.setColor(*color)
    return pc


__all__ = [
    'Scene', 'Viewport', 'CSetOfObjects', 'CCamera',
    'CPointCloud', 'CPointCloudColoured', 'CAssimpModel', 'AssimpLoadFlags',
    'CGridPlaneXY', 'CGridPlaneXZ',
    'CAxis',
    'CBox', 'CSphere', 'CCylinder', 'CArrow',
    'CText', 'CText3D',
    'CSetOfLines', 'CSimpleLine',
    'CEllipsoid2D', 'CEllipsoid3D',
    'stock_objects',
    'create_point_cloud',
]
