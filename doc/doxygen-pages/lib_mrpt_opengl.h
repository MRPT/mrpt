/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_opengl_grp [mrpt-opengl]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-opengl</code></h2>
<hr>


This library includes several data classes that represent objects that can be
inserted into a 3D scene, which can be then rendered or streamed to disk or whatever.

A good starting point to explore this library is the base class for all the
3D objects: mrpt::opengl::CRenderizable

A 3D scene is represented by an object of the type mrpt::opengl::COpenGLScene,
which in turn can contain one or several "viewports" in such a way that the
rendering area is divided into several spaces, each displaying the same or different
objects. See the tutorial online: http://www.mrpt.org/Tutorial_3D_Scenes

See the full list of classes in mrpt::opengl.

Below follows a table with the preview of most rendering primitive classes:

<div align="center">
<table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
 <tr> <td> mrpt::opengl::CArrow </td> <td> \image html preview_CArrow.png </td> </tr>
 <tr> <td> mrpt::opengl::CAssimpModel </td> <td> \image html preview_CAssimpModel.png </td> </tr>
 <tr> <td> mrpt::opengl::CAxis </td> <td> \image html preview_CAxis.png </td> </tr>
 <tr> <td> mrpt::opengl::CBox </td> <td> \image html preview_CBox.png </td> </tr>
 <tr> <td> mrpt::opengl::CFrustum </td> <td> \image html preview_CFrustum.png </td> </tr>
 <tr> <td> mrpt::opengl::CCylinder </td> <td> \image html preview_CCylinder.png </td> </tr>
 <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png </td> </tr>
 <tr> <td> mrpt::opengl::CEllipsoid </td> <td> \image html preview_CEllipsoid.png </td> </tr>
 <tr> <td> mrpt::opengl::CGridPlaneXY </td> <td> \image html preview_CGridPlaneXY.png </td> </tr>
 <tr> <td> mrpt::opengl::CGridPlaneXZ </td> <td> \image html preview_CGridPlaneXZ.png </td> </tr>
 <tr> <td> mrpt::opengl::CMesh </td> <td> \image html preview_CMesh.png </td> </tr>
 <tr> <td> mrpt::opengl::CMesh3D </td> <td> \image html preview_CMesh3D.png </td> </tr>
 <tr> <td> mrpt::opengl::CMeshFast </td> <td> \image html preview_CMeshFast.png </td> </tr>
 <tr> <td> mrpt::opengl::CPointCloud </td> <td> \image html preview_CPointCloud.png </td> </tr>
 <tr> <td> mrpt::opengl::CPointCloudColoured </td> <td> \image html preview_CPointCloudColoured.png </td> </tr>
 <tr> <td> mrpt::opengl::CPolyhedron </td> <td> \image html preview_CPolyhedron.png </td> </tr>
 <tr> <td> mrpt::opengl::CSetOfLines </td> <td> \image html preview_CSetOfLines.png </td> </tr>
 <tr> <td> mrpt::opengl::CSphere </td> <td> \image html preview_CSphere.png </td> </tr>
 <tr> <td> mrpt::opengl::CText </td> <td> \image html preview_CText.png </td> </tr>
 <tr> <td> mrpt::opengl::CText3D </td> <td> \image html preview_CText3D.png </td> </tr>
 <tr> <td> mrpt::opengl::CEllipsoidRangeBearing2D </td> <td> \image html preview_CEllipsoidRangeBearing2D.png </td> </tr>
 <tr> <td> mrpt::opengl::CEllipsoidInverseDepth2D </td> <td> \image html preview_CEllipsoidInverseDepth2D.png </td> </tr>
 <tr> <td> mrpt::opengl::CEllipsoidInverseDepth3D </td> <td> \image html preview_CEllipsoidInverseDepth3D.png </td> </tr>
 <tr> <td> mrpt::opengl::COctoMapVoxels </td> <td> \image html preview_COctoMapVoxels.png </td> </tr>
 <tr> <td> mrpt::opengl::CVectorField2D </td> <td> \image html preview_CVectorField2D.png </td> </tr>
 <tr> <td> mrpt::opengl::CVectorField3D </td> <td> \image html preview_CVectorField3D.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::BumblebeeCamera() </td> <td> \image html preview_stock_objects_BumblebeeCamera.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYSimple() </td> <td> \image html preview_stock_objects_CornerXYSimple.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYZSimple() </td> <td> \image html preview_stock_objects_CornerXYZSimple.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYZ() </td> <td> \image html preview_stock_objects_CornerXYZ.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::RobotPioneer() </td> <td> \image html preview_stock_objects_RobotPioneer.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::RobotRhodon() </td> <td> \image html preview_stock_objects_RobotRhodon.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::Hokuyo_URG() </td> <td> \image html preview_stock_objects_Hokuyo_URG.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::Hokuyo_UTM() </td> <td> \image html preview_stock_objects_Hokuyo_UTM.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::Househam_Sprayer() </td> <td> \image html preview_stock_objects_Househam_Sprayer.png </td> </tr>
</table>
</div>

Also, pose PDF classes can be converted into OpenGL objects with CSetOfObjects::posePDF2opengl():

<div align="center">
<table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
 <tr> <td> CSetOfObjects::posePDF2opengl() <br> for mrpt::poses::CPosePDFParticles</td> <td> \image html preview_CPosePDFParticles_as_opengl.png </td> </tr>
</table>
</div>


Note: The following extra OpenGL classes are provided by other libraries:


<div align="center">
<table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
<tr> <td> mrpt::opengl::CAngularObservationMesh <br> (In mrpt-maps) </td> <td> \image html preview_CAngularObservationMesh.png </td> </tr>
<tr> <td> mrpt::opengl::CPlanarLaserScan <br> (In mrpt-maps) </td> <td> \image html preview_CPlanarLaserScan.png </td> </tr>
</table>
</div>

*/

