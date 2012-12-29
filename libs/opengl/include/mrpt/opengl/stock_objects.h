/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef opengl_StockObjects_H
#define opengl_StockObjects_H

#include <mrpt/opengl/CSetOfObjects.h>

namespace mrpt
{
	namespace opengl
	{
		/** A collection of pre-built 3D objects for quick insertion in opengl::COpenGLScene objects.
		  * \ingroup mrpt_opengl_grp
		  */
		namespace stock_objects
		{
			/** Returns a representation of a Pioneer II mobile base.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::RobotPioneer() </td> <td> \image html preview_stock_objects_RobotPioneer.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP RobotPioneer();

			/** Returns three arrows representing a X,Y,Z 3D corner.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  * \sa CornerXYZSimple, CornerXYSimple, CornerXYZEye
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::CornerXYZ() </td> <td> \image html preview_stock_objects_CornerXYZ.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP CornerXYZ(float scale=1.0);
			
			/** Returns three arrows representing a X,Y,Z 3D corner.
			  *  Differently from CornerXYZ the arrowhead of Z axis ends where the object is placed.
			  *  This is useful if you want to place this object with the same position and orientation of a camera.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  * \sa CornerXYZSimple, CornerXYSimple
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::CornerXYZ() </td> <td> \image html preview_stock_objects_CornerXYZEye.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP CornerXYZEye();

			/** Returns three arrows representing a X,Y,Z 3D corner (just thick lines instead of complex arrows for faster rendering than CornerXYZ).
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  * \sa CornerXYZ, CornerXYSimple
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::CornerXYZSimple() </td> <td> \image html preview_stock_objects_CornerXYZSimple.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP CornerXYZSimple(float scale=1.0, float lineWidth=1.0);

			/** Returns two arrows representing a X,Y 2D corner (just thick lines, fast to render).
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  * \sa CornerXYZSimple, CornerXYZ, CornerXYZEye
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::CornerXYSimple() </td> <td> \image html preview_stock_objects_CornerXYSimple.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP CornerXYSimple(float scale=1.0, float lineWidth=1.0);

			/** Returns a simple 3D model of a PointGrey Bumblebee stereo camera.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::BumblebeeCamera() </td> <td> \image html preview_stock_objects_BumblebeeCamera.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP BumblebeeCamera();

		} // end namespace stock_objects
	}

} // End of namespace


#endif
