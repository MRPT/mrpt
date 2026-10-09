from __future__ import annotations
import mrpt as mrpt
from mrpt.viz._bindings import AssimpLoadFlags as AssimpLoadFlags
from mrpt.viz._bindings import CAnimatedAssimpModel as CAnimatedAssimpModel
from mrpt.viz._bindings import CArrow as CArrow
from mrpt.viz._bindings import CAssimpModel as CAssimpModel
from mrpt.viz._bindings import CAxis as CAxis
from mrpt.viz._bindings import CBox as CBox
from mrpt.viz._bindings import CCamera as CCamera
from mrpt.viz._bindings import CColorBar as CColorBar
from mrpt.viz._bindings import CCylinder as CCylinder
from mrpt.viz._bindings import CDisk as CDisk
from mrpt.viz._bindings import CEllipsoid2D as CEllipsoid2D
from mrpt.viz._bindings import CEllipsoid3D as CEllipsoid3D
from mrpt.viz._bindings import CEllipsoidInverseDepth2D as CEllipsoidInverseDepth2D
from mrpt.viz._bindings import CEllipsoidInverseDepth3D as CEllipsoidInverseDepth3D
from mrpt.viz._bindings import CEllipsoidRangeBearing2D as CEllipsoidRangeBearing2D
from mrpt.viz._bindings import CFrustum as CFrustum
from mrpt.viz._bindings import CGridPlaneXY as CGridPlaneXY
from mrpt.viz._bindings import CGridPlaneXZ as CGridPlaneXZ
from mrpt.viz._bindings import CLight as CLight
from mrpt.viz._bindings import CMesh as CMesh
from mrpt.viz._bindings import CMesh3D as CMesh3D
from mrpt.viz._bindings import CMeshFast as CMeshFast
from mrpt.viz._bindings import COctoMapVoxels as COctoMapVoxels
from mrpt.viz._bindings import COrbitCameraController as COrbitCameraController
from mrpt.viz._bindings import CPointCloud as CPointCloud
from mrpt.viz._bindings import CPointCloudColoured as CPointCloudColoured
from mrpt.viz._bindings import CPolyhedron as CPolyhedron
from mrpt.viz._bindings import CSetOfLines as CSetOfLines
from mrpt.viz._bindings import CSetOfObjects as CSetOfObjects
from mrpt.viz._bindings import CSetOfTexturedTriangles as CSetOfTexturedTriangles
from mrpt.viz._bindings import CSetOfTriangles as CSetOfTriangles
from mrpt.viz._bindings import CSimpleLine as CSimpleLine
from mrpt.viz._bindings import CSkyBox as CSkyBox
from mrpt.viz._bindings import CSphere as CSphere
from mrpt.viz._bindings import CText as CText
from mrpt.viz._bindings import CText3D as CText3D
from mrpt.viz._bindings import CTexturedPlane as CTexturedPlane
from mrpt.viz._bindings import CVectorField2D as CVectorField2D
from mrpt.viz._bindings import CVectorField3D as CVectorField3D
from mrpt.viz._bindings import CubeTextureFace as CubeTextureFace
from mrpt.viz._bindings import OctoMapVisualizationMode as OctoMapVisualizationMode
from mrpt.viz._bindings import Scene as Scene
from mrpt.viz._bindings import TLight as TLight
from mrpt.viz._bindings import TLightType as TLightType
from mrpt.viz._bindings import TTriangle as TTriangle
from mrpt.viz._bindings import TTriangleVertex as TTriangleVertex
from mrpt.viz._bindings import Viewport as Viewport
from mrpt.viz._bindings import posePDF2opengl as posePDF2opengl
from mrpt.viz._bindings import stock_objects as stock_objects
from . import _bindings
__all__: list = ['posePDF2opengl', 'Scene', 'Viewport', 'CSetOfObjects', 'CCamera', 'CPointCloud', 'CPointCloudColoured', 'CAssimpModel', 'AssimpLoadFlags', 'CGridPlaneXY', 'CGridPlaneXZ', 'CAxis', 'CBox', 'CSphere', 'CCylinder', 'CArrow', 'CText', 'CText3D', 'CSetOfLines', 'CSimpleLine', 'CEllipsoid2D', 'CEllipsoid3D', 'TTriangle', 'TTriangleVertex', 'CDisk', 'CFrustum', 'CSetOfTriangles', 'CVectorField2D', 'CVectorField3D', 'CMesh', 'CColorBar', 'CMesh3D', 'CMeshFast', 'CTexturedPlane', 'CSetOfTexturedTriangles', 'CPolyhedron', 'COrbitCameraController', 'COctoMapVoxels', 'OctoMapVisualizationMode', 'CubeTextureFace', 'CSkyBox', 'TLightType', 'TLight', 'CLight', 'CEllipsoidInverseDepth2D', 'CEllipsoidInverseDepth3D', 'CEllipsoidRangeBearing2D', 'CAnimatedAssimpModel', 'stock_objects', 'create_point_cloud']
def _scene_lshift(self, obj):
    ...
def create_point_cloud(pts_array, color = (255, 255, 255)):
    """
    Returns a CPointCloud with the points of an (N, 3) array, in one color (R, G, B, 0-255).
    """
