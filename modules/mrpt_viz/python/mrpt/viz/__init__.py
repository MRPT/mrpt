import mrpt.rtti          # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.math           # noqa: F401
import mrpt.img            # noqa: F401  (TColor, TCamera)
import mrpt.poses          # noqa: F401  (CPose3D for scene object poses)

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

# Phase 0.4 -- New classes
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

# New geometric primitives (item #3)
TTriangle = _b.TTriangle
TTriangleVertex = _b.TTriangleVertex
CDisk = _b.CDisk
CFrustum = _b.CFrustum
CSetOfTriangles = _b.CSetOfTriangles
CVectorField2D = _b.CVectorField2D
CVectorField3D = _b.CVectorField3D
CMesh = _b.CMesh

# Phase 0.5 -- more viz classes (item #3 continued)
CColorBar = _b.CColorBar
CMesh3D = _b.CMesh3D
CMeshFast = _b.CMeshFast
CTexturedPlane = _b.CTexturedPlane
CSetOfTexturedTriangles = _b.CSetOfTexturedTriangles
CPolyhedron = _b.CPolyhedron
COrbitCameraController = _b.COrbitCameraController
COctoMapVoxels = _b.COctoMapVoxels
OctoMapVisualizationMode = _b.OctoMapVisualizationMode
CubeTextureFace = _b.CubeTextureFace
CSkyBox = _b.CSkyBox
TLightType = _b.TLightType
TLight = _b.TLight
CEllipsoidInverseDepth2D = _b.CEllipsoidInverseDepth2D
CEllipsoidInverseDepth3D = _b.CEllipsoidInverseDepth3D
CEllipsoidRangeBearing2D = _b.CEllipsoidRangeBearing2D
CAnimatedAssimpModel = _b.CAnimatedAssimpModel

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
    'TTriangle', 'TTriangleVertex',
    'CDisk', 'CFrustum', 'CSetOfTriangles',
    'CVectorField2D', 'CVectorField3D',
    'CMesh',
    'CColorBar',
    'CMesh3D', 'CMeshFast',
    'CTexturedPlane', 'CSetOfTexturedTriangles',
    'CPolyhedron',
    'COrbitCameraController',
    'COctoMapVoxels', 'OctoMapVisualizationMode',
    'CubeTextureFace', 'CSkyBox',
    'TLightType', 'TLight',
    'CEllipsoidInverseDepth2D', 'CEllipsoidInverseDepth3D', 'CEllipsoidRangeBearing2D',
    'CAnimatedAssimpModel',
    'stock_objects',
    'create_point_cloud',
]
