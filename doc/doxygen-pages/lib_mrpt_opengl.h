/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
 <tr> <td> mrpt::opengl::CAxis </td> <td> \image html preview_CAxis.png </td> </tr>
 <tr> <td> mrpt::opengl::CBox </td> <td> \image html preview_CBox.png </td> </tr>
 <tr> <td> mrpt::opengl::CFrustum </td> <td> \image html preview_CFrustum.png </td> </tr>
 <tr> <td> mrpt::opengl::CCylinder </td> <td> \image html preview_CCylinder.png </td> </tr>
 <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png </td> </tr>
 <tr> <td> mrpt::opengl::CEllipsoid </td> <td> \image html preview_CEllipsoid.png </td> </tr>
 <tr> <td> mrpt::opengl::CGridPlaneXY </td> <td> \image html preview_CGridPlaneXY.png </td> </tr>
 <tr> <td> mrpt::opengl::CGridPlaneXZ </td> <td> \image html preview_CGridPlaneXZ.png </td> </tr>
 <tr> <td> mrpt::opengl::CMesh </td> <td> \image html preview_CMesh.png </td> </tr>
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
 <tr> <td> mrpt::opengl::stock_objects::BumblebeeCamera() </td> <td> \image html preview_stock_objects_BumblebeeCamera.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYSimple() </td> <td> \image html preview_stock_objects_CornerXYSimple.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYZSimple() </td> <td> \image html preview_stock_objects_CornerXYZSimple.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::CornerXYZ() </td> <td> \image html preview_stock_objects_CornerXYZ.png </td> </tr>
 <tr> <td> mrpt::opengl::stock_objects::RobotPioneer() </td> <td> \image html preview_stock_objects_RobotPioneer.png </td> </tr>
</table>
</div>

Also, pose PDF classes can be converted into OpenGL objects with CSetOfObjects::posePDF2opengl():

<div align="center">
<table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
 <tr> <td> CSetOfObjects::posePDF2opengl() <br> for mrpt::poses::CPosePDFParticles</td> <td> \image html preview_CPosePDFParticles_as_opengl.png </td> </tr>
</table>
</div>


Note: The following extra OpenGL classes are provided by other libraries:
 * mrpt::opengl::CAngularObservationMesh (In mrpt-maps)
 * mrpt::opengl::CPlanarLaserScan (In mrpt-maps)



preview_CPosePDFParticles_as_opengl

*/

