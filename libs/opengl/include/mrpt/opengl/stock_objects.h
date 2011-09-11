/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
