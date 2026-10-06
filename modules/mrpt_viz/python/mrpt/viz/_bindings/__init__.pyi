from __future__ import annotations
import mrpt.img
import mrpt.math
import mrpt.poses
import numpy
import typing
from . import stock_objects
__all__: list[str] = ['AssimpLoadFlags', 'BACK', 'BOTTOM', 'CAnimatedAssimpModel', 'CArrow', 'CAssimpModel', 'CAxis', 'CBox', 'CCamera', 'CColorBar', 'CCylinder', 'CDisk', 'CEllipsoid2D', 'CEllipsoid3D', 'CEllipsoidInverseDepth2D', 'CEllipsoidInverseDepth3D', 'CEllipsoidRangeBearing2D', 'CFrustum', 'CGridPlaneXY', 'CGridPlaneXZ', 'CLight', 'CMesh', 'CMesh3D', 'CMeshFast', 'COLOR_FROM_HEIGHT', 'COLOR_FROM_OCCUPANCY', 'COLOR_FROM_RGB_DATA', 'COctoMapVoxels', 'COrbitCameraController', 'CPointCloud', 'CPointCloudColoured', 'CPolyhedron', 'CSetOfLines', 'CSetOfObjects', 'CSetOfTexturedTriangles', 'CSetOfTriangles', 'CSimpleLine', 'CSkyBox', 'CSphere', 'CText', 'CText3D', 'CTexturedPlane', 'CVectorField2D', 'CVectorField3D', 'CVisualObject', 'CubeTextureFace', 'Directional', 'FIXED', 'FRONT', 'FlipUVs', 'IgnoreMaterialColor', 'LEFT', 'OctoMapVisualizationMode', 'Point', 'RIGHT', 'RealTimeFast', 'RealTimeMaxQuality', 'RealTimeQuality', 'Scene', 'Spot', 'TLight', 'TLightType', 'TOP', 'TRANSPARENCY_FROM_OCCUPANCY', 'TRANS_AND_COLOR_FROM_OCCUPANCY', 'TTriangle', 'TTriangleVertex', 'Verbose', 'Viewport', 'posePDF2opengl', 'stock_objects']
class CVisualObject:
    """
    Base class of all 3D objects that can be rendered.
    """
    name: str
    visible: bool
    def getColor(self) -> mrpt.img.TColorf:
        """
        Get color components as floats in the range [0,1].
        """
    def getPose(self) -> mrpt.math.TPose3D:
        """
        Returns the 3D pose of the object as TPose3D.
        """
    @typing.overload
    def setColor(self, r: int, g: int, b: int, a: int = 255) -> None:
        """
        Sets the object color from 8-bit components (0-255).
        """
    @typing.overload
    def setColor(self, color: mrpt.img.TColorf) -> None:
        """
        Sets the color from a TColorf (float components in [0,1])
        """
    @typing.overload
    def setColor(self, color: mrpt.img.TColor) -> None:
        """
        Sets the color from a TColor (uint8 components)
        """
    @typing.overload
    def setColor(self, r: float, g: float, b: float, a: float = 1.0) -> None:
        """
        Sets the color from float components in [0,1]
        """
    @typing.overload
    def setLocation(self, x: float, y: float, z: float) -> None:
        """
        Changes the position, keeping the orientation
        """
    @typing.overload
    def setLocation(self, p: mrpt.math.TPoint3D) -> None:
        """
        Changes the location of the object, keeping untouched the orientation.
        """
    @typing.overload
    def setPose(self, pose: mrpt.poses.CPose3D) -> None:
        """
        Sets the pose of the object with respect to its parent.
        """
    @typing.overload
    def setPose(self, pose: mrpt.poses.CPose2D) -> None:
        """
        Sets the pose of the object with respect to its parent, from a 2D pose.
        """
    @typing.overload
    def setScale(self, s: float) -> None:
        """
        Sets the same scale factor in x, y and z
        """
    @typing.overload
    def setScale(self, sx: float, sy: float, sz: float) -> None:
        """
        Sets the scale factor applied to the object along each axis (default: 1).
        """
    @property
    def castShadows(self) -> bool:
        """
        Whether the object casts shadows (if shadows are enabled in the viewport)
        """
    @castShadows.setter
    def castShadows(self, arg1: bool) -> None:
        ...
    @property
    def pose(self) -> mrpt.math.TPose3D:
        """
        Pose of the object with respect to its parent (read as a TPose3D, set from a CPose3D).
        """
    @pose.setter
    def pose(self, arg1: mrpt.poses.CPose3D) -> CVisualObject:
        ...
class CSetOfObjects(CVisualObject):
    """
    A group of 3D objects, placed relative to the pose of this object.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __len__(self) -> int:
        ...
    def __lshift__(self, obj):
        ...
    def clear(self) -> None:
        """
        Removes all objects.
        """
    def insert(self, obj: CVisualObject) -> None:
        """
        Adds an object to the set.
        """
class CCamera(CVisualObject):
    """
    Defines the intrinsic and extrinsic camera coordinates from which to render a 3D scene.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def setAzimuthDegrees(self, arg0: float) -> None:
        """
        Sets the camera azimuth angle, in degrees.
        """
    def setElevationDegrees(self, arg0: float) -> None:
        """
        Sets the camera elevation angle, in degrees.
        """
    def setZoomDistance(self, arg0: float) -> None:
        """
        Sets the camera distance to the point it looks at.
        """
class Viewport:
    """
    A viewport within a Scene, containing a set of OpenGL objects to render.
    """
    def __lshift__(self, obj):
        ...
    def clear(self) -> None:
        """
        Removes all objects.
        """
    def getCamera(self) -> CCamera:
        """
        Returns the camera of this viewport.
        """
    def insert(self, obj: CVisualObject) -> None:
        """
        Adds an object to the viewport.
        """
    def setCustomBackgroundColor(self, arg0: mrpt.img.TColorf) -> None:
        """
        Defines the viewport background color.
        """
    def setViewportPosition(self, arg0: float, arg1: float, arg2: float, arg3: float) -> None:
        """
        Change the viewport position and dimension on the rendering window.
        """
    @property
    def name(self) -> str:
        """
        Returns the name of the viewport.
        """
class Scene:
    """
    A 3D scene: one or more viewports, each with a set of objects to render.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __lshift__(self, obj):
        ...
    def clear(self, createMainViewport: bool = True) -> None:
        """
        Removes all objects (and viewports), then re-creates the main viewport
        """
    def createViewport(self, name: str) -> Viewport:
        """
        Creates a new viewport with the given name, and returns it.
        """
    def getViewport(self, name: str = 'main') -> Viewport:
        """
        Returns the viewport with the given name (default: "main"), or None.
        """
    def insert(self, arg0: CVisualObject) -> None:
        """
        Insert a new object into the scene, in the given viewport (by default, into the "main" viewport).
        """
class CPointCloud(CVisualObject):
    """
    A cloud of points, all with the same color or each depending on its value along a particular coordinate axis.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def clear(self) -> None:
        """
        Empty the list of points.
        """
    def getPointSize(self) -> float:
        """
        Returns the rendered point size, in pixels.
        """
    def insertPoint(self, x: float, y: float, z: float) -> None:
        """
        Adds a new point to the cloud.
        """
    def setPointSize(self, pointSize: float) -> None:
        """
        Point size, in pixels
        """
    def setPoints(self, arg0: numpy.ndarray) -> None:
        """
        Sets all points from a NumPy array of shape (N, 3).
        """
class CAssimpModel(CVisualObject):
    """
    A 3D model loaded from any file format supported by the Assimp library.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def loadScene(self, file_name: str, flags: int = 4116) -> None:
        """
        Loads a 3D scene from a file in any Assimp-supported format.
        """
class AssimpLoadFlags:
    """
    Members:
    
      RealTimeFast
    
      RealTimeQuality
    
      RealTimeMaxQuality
    
      FlipUVs
    
      IgnoreMaterialColor
    
      Verbose
    """
    FlipUVs: typing.ClassVar[AssimpLoadFlags]
    IgnoreMaterialColor: typing.ClassVar[AssimpLoadFlags]
    RealTimeFast: typing.ClassVar[AssimpLoadFlags]
    RealTimeMaxQuality: typing.ClassVar[AssimpLoadFlags]
    RealTimeQuality: typing.ClassVar[AssimpLoadFlags]
    Verbose: typing.ClassVar[AssimpLoadFlags]
    __members__: typing.ClassVar[dict[str, AssimpLoadFlags]]
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class CGridPlaneXY(CVisualObject):
    """
    A grid of lines over the XY plane.
    """
    def __init__(self, xmin: float = -10.0, xmax: float = 10.0, ymin: float = -10.0, ymax: float = 10.0, z: float = 0.0, frequency: float = 1.0) -> None:
        """
        Builds the grid from its limits, height and spacing.
        """
    def setGridFrequency(self, arg0: float) -> None:
        """
        Sets the spacing between grid lines.
        """
    def setPlaneLimits(self, xmin: float, xmax: float, ymin: float, ymax: float) -> None:
        """
        Sets the grid limits in x and y.
        """
    def setPlaneZcoord(self, arg0: float) -> None:
        """
        Sets the grid height (z).
        """
class CGridPlaneXZ(CVisualObject):
    """
    A grid of lines over the XZ plane.
    """
    def __init__(self, xmin: float = -10.0, xmax: float = 10.0, zmin: float = -10.0, zmax: float = 10.0, y: float = 0.0, frequency: float = 1.0) -> None:
        """
        Builds the grid from its limits, y coordinate and spacing.
        """
    def setGridFrequency(self, arg0: float) -> None:
        """
        Sets the spacing between grid lines.
        """
    def setPlaneLimits(self, xmin: float, xmax: float, zmin: float, zmax: float) -> None:
        """
        Sets the grid limits in x and z.
        """
    def setPlaneYcoord(self, arg0: float) -> None:
        """
        Sets the grid y coordinate.
        """
class CAxis(CVisualObject):
    """
    Draw a 3D world axis, with coordinate marks at some regular interval.
    """
    def __init__(self, xmin: float = -1.0, ymin: float = -1.0, zmin: float = -1.0, xmax: float = 1.0, ymax: float = 1.0, zmax: float = 1.0, frequency: float = 1.0, lineWidth: float = 3.0, marks: bool = True) -> None:
        """
        Constructor.
        """
    def enableTickMarks(self, arg0: bool) -> None:
        """
        Shows or hides the tick marks.
        """
    def getFrequency(self) -> float:
        """
        Returns the spacing between tick marks.
        """
    def getTextScale(self) -> float:
        """
        Returns the size of text labels.
        """
    def setAxisLimits(self, arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> None:
        """
        Sets the axis limits (xmin, ymin, zmin, xmax, ymax, zmax).
        """
    def setFrequency(self, arg0: float) -> None:
        """
        Changes the frequency of the "ticks".
        """
    def setTextScale(self, arg0: float) -> None:
        """
        Sets the size of text labels (default: 0.25).
        """
class CBox(CVisualObject):
    """
    A solid or wireframe box, given its two opposite corners.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, corner1: mrpt.math.TPoint3D, corner2: mrpt.math.TPoint3D, is_wireframe: bool = False, lineWidth: float = 1.0) -> None:
        """
        Builds the box from two opposite corners, solid or wireframe, and line width.
        """
    def enableBoxBorder(self, arg0: bool) -> None:
        """
        Enable/disable drawing a border around solid boxes.
        """
    def getBoxCorners(self) -> tuple:
        """
        Get the current box corners.
        """
    def isWireframe(self) -> bool:
        """
        Returns true if wireframe mode is enabled.
        """
    def setBoxBorderColor(self, color: mrpt.img.TColor) -> None:
        """
        Color of the box edges, drawn if enableBoxBorder()
        """
    def setBoxCorners(self, arg0: mrpt.math.TPoint3D, arg1: mrpt.math.TPoint3D) -> None:
        """
        Set the position and size of the box, from two corners in 3D.
        """
    def setWireframe(self, arg0: bool) -> None:
        """
        Sets wireframe rendering mode (true) or solid mode (false, default)
        """
class CSphere(CVisualObject):
    """
    A solid or wire-frame sphere.
    """
    def __init__(self, radius: float = 1.0, nDivs: int = 20) -> None:
        """
        Constructor.
        """
    def getRadius(self) -> float:
        """
        Returns the sphere radius.
        """
    def setNumberDivs(self, arg0: int) -> None:
        """
        Sets the number of slices and stacks used to render the sphere.
        """
    def setRadius(self, arg0: float) -> None:
        """
        Sets the sphere radius.
        """
class CCylinder(CVisualObject):
    """
    A cylinder or cone whose base lies in the XY plane.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: unit cylinder.
        """
    @typing.overload
    def __init__(self, baseRadius: float, topRadius: float, height: float = 1.0, slices: int = 20) -> None:
        """
        Constructor with parameters.
        """
    def getHeight(self) -> float:
        """
        Gets the cylinder's height.
        """
    def setHeight(self, arg0: float) -> None:
        """
        Changes cylinder's height.
        """
    def setRadii(self, arg0: float, arg1: float) -> None:
        """
        Sets both radii independently.
        """
    def setRadius(self, arg0: float) -> None:
        """
        Sets both radii to a single value, configuring the object as a cylinder.
        """
class CArrow(CVisualObject):
    """
    A 3D arrow.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def setArrowEnds(self, x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> None:
        """
        Sets the arrow start (x0, y0, z0) and end (x1, y1, z1) points.
        """
    def setHeadRatio(self, arg0: float) -> None:
        """
        Sets the length of the arrow head, relative to the arrow length.
        """
    def setSmallRadius(self, arg0: float) -> None:
        """
        Sets the radius of the arrow body.
        """
class CText(CVisualObject):
    """
    A 2D text label at a 3D position, always facing the camera.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, text: str) -> None:
        """
        Builds the label with the given text.
        """
    def getString(self) -> str:
        """
        Return the current text associated to this label.
        """
    def setFont(self, arg0: str, arg1: int) -> None:
        """
        Sets the font among "sans", "serif", "mono".
        """
    def setString(self, arg0: str) -> None:
        """
        Sets the text to display.
        """
class CText3D(CVisualObject):
    """
    A 3D text (rendered with OpenGL primitives), with selectable font face and drawing style.
    """
    def __init__(self, text: str = '', fontName: str = 'sans', scale: float = 1.0) -> None:
        """
        Builds a 3D text from its string, font name and scale.
        """
    def getString(self) -> str:
        """
        Returns the currently text associated to this object.
        """
    def setString(self, arg0: str) -> None:
        """
        Sets the displayed string.
        """
class CSetOfLines(CVisualObject):
    """
    A set of independent lines (or segments), one line with its own start and end positions (X,Y,Z).
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __len__(self) -> int:
        ...
    @typing.overload
    def appendLine(self, x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> None:
        """
        Appends a segment given its end points (x0, y0, z0, x1, y1, z1).
        """
    @typing.overload
    def appendLine(self, arg0: mrpt.math.TSegment3D) -> None:
        """
        Appends a segment.
        """
    def clear(self) -> None:
        """
        Clear the list of segments.
        """
class CSimpleLine(CVisualObject):
    """
    A line segment.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getLineEnd(self) -> mrpt.math.TPoint3Df:
        """
        Returns the line end point.
        """
    def getLineStart(self) -> mrpt.math.TPoint3Df:
        """
        Returns the line start point.
        """
    def setLineCoords(self, x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> None:
        """
        Sets the line end points (x0, y0, z0, x1, y1, z1).
        """
class CEllipsoid3D(CVisualObject):
    """
    A 3D ellipsoid, centered at zero with respect to this object pose.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def set3DsegmentsCount(self, arg0: int) -> None:
        """
        The number of segments of a 3D ellipse (in both "axes") (default=20)
        """
    def setCovMatrix(self, arg0: mrpt.math.CMatrixDouble33) -> None:
        """
        Like setCovMatrixAndMean(), for mean=zero.
        """
    def setQuantiles(self, arg0: float) -> None:
        """
        Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid (default=3, ~97 or ~98% CI); the exact mathematical meaning is: This value of "quantiles" q should be set to the square root of the chi-squared inverse cdf corresponding to the desired confidence interval.
        """
class CEllipsoid2D(CVisualObject):
    """
    A 2D ellipse on the XY plane, centered at the origin of this object pose.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def setCovMatrix(self, arg0: mrpt.math.CMatrixDouble22) -> None:
        """
        Like setCovMatrixAndMean(), for mean=zero.
        """
    def setQuantiles(self, arg0: float) -> None:
        """
        Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid (default=3, ~97 or ~98% CI); the exact mathematical meaning is: This value of "quantiles" q should be set to the square root of the chi-squared inverse cdf corresponding to the desired confidence interval.
        """
class CPointCloudColoured(CVisualObject):
    """
    A cloud of points, each one with an individual color (R,G,B,A).
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        """
        Erase all the points.
        """
    def getPointSize(self) -> float:
        """
        Returns the rendered point size, in pixels.
        """
    def push_back(self, x: float, y: float, z: float, r: float = 1.0, g: float = 1.0, b: float = 1.0, a: float = 1.0) -> None:
        """
        Inserts a new point into the point cloud.
        """
    def setPointSize(self, pointSize: float) -> None:
        """
        Point size, in pixels
        """
    def size(self) -> int:
        """
        Return the number of points.
        """
class TTriangleVertex:
    """
    One vertex of a TTriangle: position, color, normal and texture coordinates.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def setColor(self, color: mrpt.img.TColor) -> None:
        """
        Set vertex color from TColor
        """
class TTriangle:
    """
    A triangle (float coordinates) with RGBA colors (u8) and UV (texture coordinates) for each vertex.
    """
    vertices: list[TTriangleVertex]
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, p1: mrpt.math.TPoint3Df, p2: mrpt.math.TPoint3Df, p3: mrpt.math.TPoint3Df) -> None:
        """
        Constructor from 3 points (default normals are computed)
        """
    def computeNormals(self) -> None:
        """
        Compute the three normals from the cross-product of "v01 x v02".
        """
class CDisk(CVisualObject):
    """
    A planar disk in the XY plane.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructor.
        """
    @typing.overload
    def __init__(self, out_radius: float, in_radius: float, slices: int = 50) -> None:
        """
        Builds the disk from its outer and inner radii and number of slices.
        """
    def setDiskRadius(self, out_radius: float, in_radius: float = 0.0) -> None:
        """
        Sets the outer and inner radii.
        """
    def setSlicesCount(self, N: int) -> None:
        """
        Sets the number of slices (at least 3; default: 50).
        """
    @property
    def in_radius(self) -> float:
        """
        Inner radius.
        """
    @property
    def out_radius(self) -> float:
        """
        Outer radius.
        """
class CFrustum(CVisualObject):
    """
    A solid or wireframe frustum in 3D (a rectangular truncated pyramid), with arbitrary (possibly assymetric) field-of-view angles.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Basic empty constructor. Set all parameters to default.
        """
    @typing.overload
    def __init__(self, near_distance: float, far_distance: float, horz_FOV_degrees: float, vert_FOV_degrees: float, lineWidth: float = 1.0, draw_lines: bool = True, draw_planes: bool = False) -> None:
        """
        Constructor with some parameters.
        """
    def setHorzFOV(self, fov_degrees: float) -> None:
        """
        Changes horizontal FOV (symmetric)
        """
    def setNearFarPlanes(self, near: float, far: float) -> None:
        """
        Changes distance of near & far planes.
        """
    def setPlaneColor(self, color: mrpt.img.TColor) -> None:
        """
        Sets the color of the planes (line color is set with setColor()).
        """
    def setVertFOV(self, fov_degrees: float) -> None:
        """
        Changes vertical FOV (symmetric)
        """
    @property
    def far_plane(self) -> float:
        """
        Distance to the far plane.
        """
    @property
    def horz_fov(self) -> float:
        """
        Horizontal field of view, in degrees.
        """
    @property
    def near_plane(self) -> float:
        """
        Distance to the near plane.
        """
    @property
    def vert_fov(self) -> float:
        """
        Vertical field of view, in degrees.
        """
class CSetOfTriangles(CVisualObject):
    """
    A set of colored triangles, able to draw any solid, arbitrarily complex object without textures.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def clearTriangles(self) -> None:
        """
        Clear this object, removing all triangles.
        """
    def getTriangle(self, idx: int) -> TTriangle:
        """
        Gets the i-th triangle.
        """
    def getTrianglesCount(self) -> int:
        """
        Get triangle count.
        """
    def insertTriangle(self, triangle: TTriangle) -> None:
        """
        Inserts a triangle into the set.
        """
class CVectorField2D(CVisualObject):
    """
    A 2D vector field representation, consisting of points and arrows drawn on a plane (invisible grid).
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def clear(self) -> None:
        """
        Clear the matrices.
        """
    def setGridCenterAndCellSize(self, cx: float, cy: float, cell_x: float, cell_y: float) -> None:
        """
        Set the coordinates of the grid on where the vector field will be drawn by setting its center and the cell size.
        """
    def setGridLimits(self, xmin: float, xmax: float, ymin: float, ymax: float) -> None:
        """
        Set the coordinates of the grid on where the vector field will be drawn using x-y max and min values.
        """
    def setPointColor(self, R: float, G: float, B: float, A: float = 1.0) -> None:
        """
        Set the point color in the range [0,1].
        """
    def setVectorField(self, vx: numpy.ndarray, vy: numpy.ndarray) -> None:
        """
        Sets the vector components at each grid cell, as 2D float arrays of equal shape.
        """
    def setVectorFieldColor(self, R: float, G: float, B: float, A: float = 1.0) -> None:
        """
        Set the arrow color in the range [0,1].
        """
class CVectorField3D(CVisualObject):
    """
    A 3D vector field representation, consisting of points and arrows drawn at any spatial position.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def clear(self) -> None:
        """
        Clear the matrices.
        """
    def setMaxSpeedForColor(self, s: float) -> None:
        """
        Set the max speed associated for the color map ( m_still_color, m_maxspeed_color)
        """
    def setPointColor(self, R: float, G: float, B: float, A: float = 1.0) -> None:
        """
        Set the point color in the range [0,1].
        """
    def setPointCoordinates(self, px: numpy.ndarray, py: numpy.ndarray, pz: numpy.ndarray) -> None:
        """
        Sets the point coordinates, as 2D float arrays of equal shape.
        """
    def setVectorField(self, vx: numpy.ndarray, vy: numpy.ndarray, vz: numpy.ndarray) -> None:
        """
        Sets the vector components at each point, as 2D float arrays of equal shape.
        """
    def setVectorFieldColor(self, R: float, G: float, B: float, A: float = 1.0) -> None:
        """
        Set the arrow color in the range [0,1].
        """
class CMesh(CVisualObject):
    """
    A planar (XY) grid where each cell has an associated height and, optionally, a texture map.
    """
    def __init__(self, enable_transparency: bool = False, xMin: float = -1.0, xMax: float = 1.0, yMin: float = -1.0, yMax: float = 1.0) -> None:
        """
        Builds the mesh from its transparency flag and x and y limits.
        """
    def enableColorFromZ(self, v: bool) -> None:
        """
        Enable color from Z height using HOT colormap
        """
    def enableTransparency(self, v: bool) -> None:
        """
        Enables or disables transparency.
        """
    def enableWireFrame(self, v: bool) -> None:
        """
        Shows the mesh as wireframe (true) or solid (false).
        """
    def setGridLimits(self, xMin: float, xMax: float, yMin: float, yMax: float) -> None:
        """
        Sets the x and y limits of the grid.
        """
    def setZ(self, Z: numpy.ndarray) -> None:
        """
        Set height matrix (numpy float32 2D array)
        """
class CColorBar(CVisualObject):
    """
    A colorbar indicator. This class renders a colorbar as a 3D object, in the XY plane.
    """
    def __init__(self, colormap: int = 0, width: float = 0.2, height: float = 1.0, min_col: float = 0.0, max_col: float = 1.0, min_value: float = 0.0, max_value: float = 1.0, label_format: str = '%7.02f', label_font_size: float = 0.05000000074505806) -> None:
        """
        Builds a color bar from its colormap, size, color range, value range and label format.
        """
    def setColorAndValueLimits(self, col_min: float, col_max: float, value_min: float, value_max: float) -> None:
        """
        Sets the color range and the value range it represents.
        """
    def setColormap(self, colormap: int) -> None:
        """
        Sets the colormap, as the integer value of a mrpt.img.TColormap.
        """
class CMesh3D(CVisualObject):
    """
    A 3D mesh composed of triangles and/or quads. A typical usage example would be a 3D model of an object.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def enableFaceNormals(self, v: bool) -> None:
        """
        Enables or disables computing normals per face.
        """
    def enableShowEdges(self, v: bool) -> None:
        """
        Shows or hides the edges.
        """
    def enableShowFaces(self, v: bool) -> None:
        """
        Shows or hides the faces.
        """
    def enableShowVertices(self, v: bool) -> None:
        """
        Shows or hides the vertices.
        """
    def loadMesh(self, vertices: numpy.ndarray, face_vertices: numpy.ndarray, verts_per_face: numpy.ndarray) -> None:
        """
        Load a mesh from numpy arrays: vertices (Nx3 float32), face_vertices (M,) int32, verts_per_face (F,) int32
        """
class CMeshFast(CVisualObject):
    """
    A planar (XY) grid where each cell has an associated height and, optionally, a texture map.
    """
    def __init__(self, enable_transparency: bool = False, xMin: float = -1.0, xMax: float = 1.0, yMin: float = -1.0, yMax: float = 1.0) -> None:
        """
        Constructor.
        """
    def enableColorFromZ(self, v: bool, colormap: int = 4) -> None:
        """
        Enable color from Z height (colormap int)
        """
    def enableTransparency(self, v: bool) -> None:
        """
        Enables or disables transparency.
        """
    def setGridLimits(self, xmin: float, xmax: float, ymin: float, ymax: float) -> None:
        """
        Sets the x and y limits of the grid.
        """
    def setZ(self, Z: numpy.ndarray) -> None:
        """
        Set height matrix (numpy float32 2D array)
        """
class CTexturedPlane(CVisualObject):
    """
    A 2D plane in the XY plane with a texture image.
    """
    def __init__(self, x_min: float = -1.0, x_max: float = 1.0, y_min: float = -1.0, y_max: float = 1.0) -> None:
        """
        Builds the plane from its x and y limits.
        """
    def enableLighting(self, enable: bool = True) -> None:
        """
        Enables or disables lighting on the plane.
        """
    def getPlaneCorners(self) -> tuple:
        """
        Get the coordinates of the four corners that define the plane on the XY plane.
        """
    def setPlaneCorners(self, xMin: float, xMax: float, yMin: float, yMax: float) -> None:
        """
        Set the coordinates of the four corners that define the plane on the XY plane.
        """
    def setTextureRepeat(self, repeatX: float, repeatY: float) -> None:
        """
        Set the number of times the texture repeats in each direction.
        """
class CSetOfTexturedTriangles(CVisualObject):
    """
    A set of textured triangles. This class can be used to draw any solid, arbitrarily complex object with textures.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def clearTriangles(self) -> None:
        """
        Removes all triangles.
        """
    def getTriangle(self, idx: int) -> TTriangle:
        """
        Returns the i-th triangle.
        """
    def getTrianglesCount(self) -> int:
        """
        Returns the number of triangles.
        """
    def insertTriangle(self, triangle: TTriangle) -> None:
        """
        Appends a triangle.
        """
class CPolyhedron(CVisualObject):
    """
    This class represents arbitrary polyhedra. The class includes a set of static methods to create common polyhedrons.
    """
    @staticmethod
    def CreateCuboctahedron(radius: float) -> CPolyhedron:
        """
        Creates a cuboctahedron, consisting of six square faces and eight triangular ones (see http://en.wikipedia.org/wiki/Cuboctahedron).
        """
    @staticmethod
    def CreateDodecahedron(radius: float) -> CPolyhedron:
        """
        Creates a regular dodecahedron (see http://en.wikipedia.org/wiki/Dodecahedron).
        """
    @staticmethod
    def CreateHexahedron(radius: float) -> CPolyhedron:
        """
        Creates a regular cube, also called hexahedron (see http://en.wikipedia.org/wiki/Hexahedron).
        """
    @staticmethod
    def CreateIcosahedron(radius: float) -> CPolyhedron:
        """
        Creates a regular icosahedron (see http://en.wikipedia.org/wiki/Icosahedron).
        """
    @staticmethod
    def CreateIcosidodecahedron(radius: float, type: bool = True) -> CPolyhedron:
        """
        Creates an icosidodecahedron, with 12 pentagons and 20 triangles (see http://en.wikipedia.org/wiki/Icosidodecahedron).
        """
    @staticmethod
    def CreateOctahedron(radius: float) -> CPolyhedron:
        """
        Creates a regular octahedron (see http://en.wikipedia.org/wiki/Octahedron).
        """
    @staticmethod
    def CreateTetrahedron(radius: float) -> CPolyhedron:
        """
        Creates a regular tetrahedron (see http://en.wikipedia.org/wiki/Tetrahedron).
        """
    @staticmethod
    def CreateTruncatedHexahedron(radius: float) -> CPolyhedron:
        """
        Creates a truncated hexahedron, with six octogonal faces and eight triangular ones (see http://en.wikipedia.org/wiki/Truncated_hexahedron).
        """
    @staticmethod
    def CreateTruncatedIcosahedron(radius: float) -> CPolyhedron:
        """
        Creates a truncated icosahedron, consisting of 20 hexagons and 12 pentagons.
        """
    @staticmethod
    def CreateTruncatedOctahedron(radius: float) -> CPolyhedron:
        """
        Creates a truncated octahedron, with eight hexagons and eight squares (see http://en.wikipedia.org/wiki/Truncated_octahedron).
        """
    @staticmethod
    def CreateTruncatedTetrahedron(radius: float) -> CPolyhedron:
        """
        Creates a truncated tetrahedron, consisting of four triangular faces and for hexagonal ones (see http://en.wikipedia.org/wiki/Truncated_tetrahedron).
        """
class COrbitCameraController:
    """
    Framework-agnostic orbit/pan/zoom camera controller.
    """
    azimuth_deg: float
    elevation_deg: float
    roll_deg: float
    zoom: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def applyTo(self, cam: CCamera) -> None:
        """
        Writes the current orbit parameters into cam.
        """
    def onMouseButton(self, x: int, y: int, button: int, down: bool) -> None:
        """
        Call on button press/release.
        """
    def onMouseMove(self, x: int, y: int, buttons: int, modifiers: int) -> None:
        """
        Call on mouse-move events while any button is held.
        """
    def onScroll(self, delta: float, modifiers: int) -> None:
        """
        Call on scroll-wheel events.
        """
    def setAzimuthDegrees(self, deg: float) -> None:
        """
        Sets the camera azimuth angle, in degrees.
        """
    def setCameraPointing(self, x: float, y: float, z: float) -> None:
        """
        Sets the point the camera looks at (x, y, z).
        """
    def setElevationDegrees(self, deg: float) -> None:
        """
        Sets the camera elevation angle, in degrees.
        """
    def setFrom(self, cam: CCamera) -> None:
        """
        Initialises the controller from an existing CCamera.
        """
    def setZoomDistance(self, d: float) -> None:
        """
        Sets the camera distance to the point it looks at.
        """
class OctoMapVisualizationMode:
    """
    Members:
    
      FIXED : All voxels have the same fixed color
    
      COLOR_FROM_HEIGHT : Color voxels by height
    
      COLOR_FROM_OCCUPANCY : Color by occupancy probability
    
      TRANSPARENCY_FROM_OCCUPANCY : Transparency from occupancy
    
      TRANS_AND_COLOR_FROM_OCCUPANCY : Both transparency and color from occupancy
    
      COLOR_FROM_RGB_DATA : Use per-voxel stored RGB
    """
    COLOR_FROM_HEIGHT: typing.ClassVar[OctoMapVisualizationMode]
    COLOR_FROM_OCCUPANCY: typing.ClassVar[OctoMapVisualizationMode]
    COLOR_FROM_RGB_DATA: typing.ClassVar[OctoMapVisualizationMode]
    FIXED: typing.ClassVar[OctoMapVisualizationMode]
    TRANSPARENCY_FROM_OCCUPANCY: typing.ClassVar[OctoMapVisualizationMode]
    TRANS_AND_COLOR_FROM_OCCUPANCY: typing.ClassVar[OctoMapVisualizationMode]
    __members__: typing.ClassVar[dict[str, OctoMapVisualizationMode]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class COctoMapVoxels(CVisualObject):
    """
    Renders voxels, typically from a 3D octomap.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def clear(self) -> None:
        """
        Clears everything.
        """
    def enableCubeTransparency(self, enable: bool) -> None:
        """
        Enables or disables using the alpha channel of the voxel colors.
        """
    def enableLights(self, enable: bool) -> None:
        """
        Can be used to enable/disable the effects of lighting in this object.
        """
    def getVoxelCount(self, set_index: int) -> int:
        """
        Returns the total count of voxels in one voxel set.
        """
    def getVoxelSetCount(self) -> int:
        """
        Returns the number of voxel sets.
        """
    def push_back_Voxel(self, set_index: int, x: float, y: float, z: float, side: float, r: int = 200, g: int = 200, b: int = 200, a: int = 255) -> None:
        """
        Appends a voxel (center, side length and RGBA color) to a voxel set.
        """
    def resizeVoxelSets(self, n: int) -> None:
        """
        Sets the number of voxel sets.
        """
    def resizeVoxels(self, set_index: int, n: int) -> None:
        """
        Sets the number of voxels in one voxel set.
        """
    def setVisualizationMode(self, mode: OctoMapVisualizationMode) -> None:
        """
        Select the visualization mode. To have any effect, this method has to be called before loading the octomap.
        """
    def showGridLines(self, show: bool) -> None:
        """
        Shows/hides the grid lines.
        """
    def showVoxels(self, voxel_set: int, show: bool) -> None:
        """
        Shows or hides the voxels of one voxel set.
        """
    def showVoxelsAsPoints(self, enable: bool) -> None:
        """
        For quick renders: render voxels as points instead of cubes.
        """
class CubeTextureFace:
    """
    Members:
    
      LEFT
    
      RIGHT
    
      TOP
    
      BOTTOM
    
      FRONT
    
      BACK
    """
    BACK: typing.ClassVar[CubeTextureFace]
    BOTTOM: typing.ClassVar[CubeTextureFace]
    FRONT: typing.ClassVar[CubeTextureFace]
    LEFT: typing.ClassVar[CubeTextureFace]
    RIGHT: typing.ClassVar[CubeTextureFace]
    TOP: typing.ClassVar[CubeTextureFace]
    __members__: typing.ClassVar[dict[str, CubeTextureFace]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class CSkyBox(CVisualObject):
    """
    A Sky Box: 6 textures that are always rendered at "infinity" to give the impression of the scene to be much larger.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def assignImage(self, face: CubeTextureFace, img: mrpt.img.CImage) -> None:
        """
        Assigns a texture. It is mandatory to assign all 6 faces before initializing/rendering the texture.
        """
class TLightType:
    """
    Members:
    
      Directional
    
      Point
    
      Spot
    """
    Directional: typing.ClassVar[TLightType]
    Point: typing.ClassVar[TLightType]
    Spot: typing.ClassVar[TLightType]
    __members__: typing.ClassVar[dict[str, TLightType]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class TLight:
    """
    A single light source (directional, point, or spot).
    """
    attenuation_constant: float
    attenuation_linear: float
    attenuation_quadratic: float
    diffuse: float
    direction: mrpt.math.TPoint3Df
    position: mrpt.math.TPoint3Df
    specular: float
    spot_inner_cutoff_deg: float
    spot_outer_cutoff_deg: float
    type: TLightType
    @staticmethod
    def Directional(dir: mrpt.math.TPoint3Df, r: float = 1.0, g: float = 1.0, b: float = 1.0, diffuse: float = 0.800000011920929, specular: float = 0.949999988079071) -> TLight:
        """
        Factory: creates a directional light.
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def cast_shadows(self) -> bool:
        """
        Whether this Point/Spot light casts shadows (cube shadow map, default: False).
        """
    @cast_shadows.setter
    def cast_shadows(self, arg0: bool) -> None:
        ...
    @property
    def range(self) -> float:
        """
        Maximum reach of Point/Spot lights [m], where the light fades to zero (0=unlimited).
        """
    @range.setter
    def range(self, arg0: float) -> None:
        ...
class CLight(CVisualObject):
    """
    A light source placed in the scene graph: its position and direction are relative to the pose of this object and its parents. It is switched on and off with setVisibility().
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, light: TLight) -> None:
        """
        Constructor from the light parameters, in the local frame of this object.
        """
    @property
    def light(self) -> TLight:
        """
        The light parameters, in the local frame of this object.
        """
    @light.setter
    def light(self, arg1: TLight) -> None:
        ...
class CEllipsoidInverseDepth2D(CVisualObject):
    """
    An uncertainty ellipse of an (inverse range, yaw) variable, drawn in 2D Cartesian space.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getQuantiles(self) -> float:
        """
        Returns the number of sigmas of the drawn ellipse (see setQuantiles()).
        """
    def getUnderflowMaxRange(self) -> float:
        """
        Returns the range used for points of the ellipsoid that fall in negative ranges.
        """
    def setCovMatrix(self, arg0: mrpt.math.CMatrixDouble22) -> None:
        """
        Like setCovMatrixAndMean(), for mean=zero.
        """
    def setQuantiles(self, q: float) -> None:
        """
        Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid (default=3, ~97 or ~98% CI); the exact mathematical meaning is: This value of "quantiles" q should be set to the square root of the chi-squared inverse cdf corresponding to the desired confidence interval.
        """
    def setUnderflowMaxRange(self, maxRange: float) -> None:
        """
        Sets the range used for points of the ellipsoid that fall in negative ranges (default: 1e6).
        """
class CEllipsoidInverseDepth3D(CVisualObject):
    """
    An uncertainty ellipsoid of an (inverse range, yaw, pitch) variable, drawn in 3D Cartesian space.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def getQuantiles(self) -> float:
        """
        Returns the number of sigmas of the drawn ellipsoid (see setQuantiles()).
        """
    def getUnderflowMaxRange(self) -> float:
        """
        Returns the range used for points of the ellipsoid that fall in negative ranges.
        """
    def setCovMatrix(self, arg0: mrpt.math.CMatrixDouble33) -> None:
        """
        Like setCovMatrixAndMean(), for mean=zero.
        """
    def setQuantiles(self, q: float) -> None:
        """
        Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid (default=3, ~97 or ~98% CI); the exact mathematical meaning is: This value of "quantiles" q should be set to the square root of the chi-squared inverse cdf corresponding to the desired confidence interval.
        """
    def setUnderflowMaxRange(self, maxRange: float) -> None:
        """
        Sets the range used for points of the ellipsoid that fall in negative ranges (default: 1e6).
        """
class CEllipsoidRangeBearing2D(CVisualObject):
    """
    An uncertainty ellipse of a (range, bearing) variable, drawn in 2D Cartesian space.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def getQuantiles(self) -> float:
        """
        Returns the number of sigmas of the drawn ellipse (see setQuantiles()).
        """
    def setCovMatrix(self, arg0: mrpt.math.CMatrixDouble22) -> None:
        """
        Like setCovMatrixAndMean(), for mean=zero.
        """
    def setQuantiles(self, q: float) -> None:
        """
        Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid (default=3, ~97 or ~98% CI); the exact mathematical meaning is: This value of "quantiles" q should be set to the square root of the chi-squared inverse cdf corresponding to the desired confidence interval.
        """
class CAnimatedAssimpModel(CAssimpModel):
    """
    Extension of CAssimpModel with skeletal animation support.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getAnimationCount(self) -> int:
        """
        Get number of animations in the model.
        """
    def getAnimationDuration(self, anim_idx: int = 0) -> float:
        """
        Get animation duration in seconds.
        """
    def getAnimationName(self, anim_idx: int) -> str:
        """
        Get animation name by index.
        """
    def loadScene(self, file_name: str, flags: int = 4116) -> None:
        """
        Loads a 3D scene and extracts skeleton/animation data.
        """
    def setActiveAnimation(self, anim_name: str) -> None:
        """
        Select which animation to play (by index).
        """
    def setActiveAnimationByIndex(self, idx: int) -> None:
        """
        Select which animation to play (by index).
        """
    def setAnimationTime(self, t_seconds: float) -> None:
        """
        Set current animation time in seconds. Updates all bone transforms for the active animation and rebuilds the mesh geometry with skinned positions.
        """
    def setLooping(self, loop: bool) -> None:
        """
        Enable/disable animation looping.
        """
@typing.overload
def posePDF2opengl(pdf: mrpt.poses.CPosePDF) -> CSetOfObjects:
    """
    Returns a 3D representation of a 2D pose PDF (ellipses, particles, ...)
    """
@typing.overload
def posePDF2opengl(pdf: mrpt.poses.CPose3DPDF) -> CSetOfObjects:
    """
    Returns a 3D representation of a 3D pose PDF (ellipsoids, particles, ...)
    """
BACK: CubeTextureFace
BOTTOM: CubeTextureFace
COLOR_FROM_HEIGHT: OctoMapVisualizationMode
COLOR_FROM_OCCUPANCY: OctoMapVisualizationMode
COLOR_FROM_RGB_DATA: OctoMapVisualizationMode
Directional: TLightType
FIXED: OctoMapVisualizationMode
FRONT: CubeTextureFace
FlipUVs: AssimpLoadFlags
IgnoreMaterialColor: AssimpLoadFlags
LEFT: CubeTextureFace
Point: TLightType
RIGHT: CubeTextureFace
RealTimeFast: AssimpLoadFlags
RealTimeMaxQuality: AssimpLoadFlags
RealTimeQuality: AssimpLoadFlags
Spot: TLightType
TOP: CubeTextureFace
TRANSPARENCY_FROM_OCCUPANCY: OctoMapVisualizationMode
TRANS_AND_COLOR_FROM_OCCUPANCY: OctoMapVisualizationMode
Verbose: AssimpLoadFlags
