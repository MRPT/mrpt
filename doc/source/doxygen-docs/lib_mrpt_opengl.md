\defgroup mrpt_opengl_grp [mrpt-opengl]

Visualization helpers for 3D rendering of all kind of primitives

[TOC]

# Library mrpt-opengl

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-opengl-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

This library includes several data classes that represent objects that can be
inserted into a 3D scene, which can be then rendered or streamed to disk or whatever.

A good starting point to explore this library is the base class for all the
3D objects: mrpt::opengl::CRenderizable

A 3D scene is represented by an object of the type mrpt::opengl::COpenGLScene,
which in turn can contain one or several "viewports" in such a way that the
rendering area is divided into several spaces, each displaying the same or different
objects. See the tutorial: \ref tutorial_3D_scenes

See the full list of classes in mrpt::opengl.

Next follows a list with the preview of most rendering primitive classes:

- mrpt::opengl::CArrow: ![mrpt::opengl::CArrow screenshot](_static/preview_CArrow.png)
- mrpt::opengl::CAssimpModel: ![mrpt::opengl::CAssimpModel screenshot](_static/preview_CAssimpModel.png)
- mrpt::opengl::CAxis: ![mrpt::opengl::CAxis screenshot](_static/preview_CAxis.png)
- mrpt::opengl::CBox: ![mrpt::opengl::CBox screenshot](_static/preview_CBox.png)
- mrpt::opengl::CFrustum: ![mrpt::opengl::CFrustum screenshot](_static/preview_CFrustum.png)
- mrpt::opengl::CCylinder: ![mrpt::opengl::CCylinder screenshot](_static/preview_CCylinder.png)
- mrpt::opengl::CDisk: ![mrpt::opengl::CDisk screenshot](_static/preview_CDisk.png)
- mrpt::opengl::CEllipsoid3D: ![mrpt::opengl::CEllipsoid3D screenshot](_static/preview_CEllipsoid.png)
- mrpt::opengl::CGridPlaneXY: ![mrpt::opengl::CGridPlaneXY screenshot](_static/preview_CGridPlaneXY.png)
- mrpt::opengl::CGridPlaneXZ: ![mrpt::opengl::CGridPlaneXZ screenshot](_static/preview_CGridPlaneXZ.png)
- mrpt::opengl::CMesh: ![mrpt::opengl::CMesh screenshot](_static/preview_CMesh.png)
- mrpt::opengl::CMesh3D: ![mrpt::opengl::CMesh3D screenshot](_static/preview_CMesh3D.png)
- mrpt::opengl::CMeshFast: ![mrpt::opengl::CMeshFast screenshot](_static/preview_CMeshFast.png)
- mrpt::opengl::CPointCloud: ![mrpt::opengl::CPointCloud screenshot](_static/preview_CPointCloud.png)
- mrpt::opengl::CPointCloudColoured: ![mrpt::opengl::CPointCloudColoured screenshot](_static/preview_CPointCloudColoured.png)
- mrpt::opengl::CPolyhedron: ![mrpt::opengl::CPolyhedron screenshot](_static/preview_CPolyhedron.png)
- mrpt::opengl::CSetOfLines: ![mrpt::opengl::CSetOfLines screenshot](_static/preview_CSetOfLines.png)
- mrpt::opengl::CSphere: ![mrpt::opengl::CSphere screenshot](_static/preview_CSphere.png)
- mrpt::opengl::CText: ![mrpt::opengl::CText screenshot](_static/preview_CText.png)
- mrpt::opengl::CText3D: ![mrpt::opengl::CText3D screenshot](_static/preview_CText3D.png)
- mrpt::opengl::CEllipsoidRangeBearing2D: ![mrpt::opengl::CEllipsoidRangeBearing2D screenshot](_static/preview_CEllipsoidRangeBearing2D.png)
- mrpt::opengl::CEllipsoidInverseDepth2D: ![mrpt::opengl::CEllipsoidInverseDepth2D screenshot](_static/preview_CEllipsoidInverseDepth2D.png)
- mrpt::opengl::CEllipsoidInverseDepth3D: ![mrpt::opengl::CEllipsoidInverseDepth3D screenshot](_static/preview_CEllipsoidInverseDepth3D.png)
- mrpt::opengl::COctoMapVoxels: ![mrpt::opengl::COctoMapVoxels screenshot](_static/preview_COctoMapVoxels.png)
- mrpt::opengl::CVectorField2D: ![mrpt::opengl::CVectorField2D screenshot](_static/preview_CVectorField2D.png)
- mrpt::opengl::CVectorField3D: ![mrpt::opengl::CVectorField3D screenshot](_static/preview_CVectorField3D.png)
- mrpt::opengl::stock_objects::BumblebeeCamera(): ![mrpt::opengl::stock_objects::BumblebeeCamera() screenshot](_static/preview_stock_objects_BumblebeeCamera.png)
- mrpt::opengl::stock_objects::CornerXYSimple(): ![mrpt::opengl::stock_objects::CornerXYSimple() screenshot](_static/preview_stock_objects_CornerXYSimple.png)
- mrpt::opengl::stock_objects::CornerXYZSimple(): ![mrpt::opengl::stock_objects::CornerXYZSimple() screenshot](_static/preview_stock_objects_CornerXYSimple.png)
- mrpt::opengl::stock_objects::CornerXYZ(): ![mrpt::opengl::stock_objects::CornerXYZ() screenshot](_static/preview_stock_objects_CornerXYZ.png)
- mrpt::opengl::stock_objects::RobotPioneer(): ![mrpt::opengl::stock_objects::RobotPioneer() screenshot](_static/preview_stock_objects_RobotPioneer.png)
- mrpt::opengl::stock_objects::RobotRhodon(): ![mrpt::opengl::stock_objects::RobotRhodon() screenshot](_static/preview_stock_objects_RobotRhodon.png)
- mrpt::opengl::stock_objects::Hokuyo_URG(): ![mrpt::opengl::stock_objects::Hokuyo_URG() screenshot](_static/preview_stock_objects_Hokuyo_URG.png)
- mrpt::opengl::stock_objects::Hokuyo_UTM(): ![mrpt::opengl::stock_objects::Hokuyo_UTM() screenshot](_static/preview_stock_objects_Hokuyo_UTM.png)
- mrpt::opengl::stock_objects::Househam_Sprayer(): ![mrpt::opengl::stock_objects::Househam_Sprayer() screenshot](_static/preview_stock_objects_Househam_Sprayer.png)
 
Also, pose PDF classes can be converted into OpenGL objects with CSetOfObjects::posePDF2opengl():
- mrpt::opengl::CSetOfObjects::posePDF2opengl() for mrpt::poses::CPosePDFParticles: ![pose PDF particles](_static/preview_CPosePDFParticles_as_opengl.png)

Note: The following extra OpenGL classes are provided by other libraries:
- mrpt::opengl::CAngularObservationMesh (in mrpt-maps): ![mrpt::opengl::CAngularObservationMesh screenshot](_static/preview_CAngularObservationMesh.png)
- mrpt::opengl::CPlanarLaserScan (in mrpt-maps):  ![mrpt::opengl::CPlanarLaserScan screenshot](_static/preview_CPlanarLaserScan.png)

# Library contents
