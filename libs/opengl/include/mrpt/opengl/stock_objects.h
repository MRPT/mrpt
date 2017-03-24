/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
			/** Returns a representation of Rhodon.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::RobotRhodon() </td> <td> \image html preview_stock_objects_RobotRhodon.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP RobotRhodon();
			
			/** Returns a representation of RobotGiraff.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::RobotGiraff() </td> <td> \image html preview_stock_objects_RobotGiraff.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP RobotGiraff();

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
			  *    <tr> <td> mrpt::opengl::stock_objects::CornerXYZ() </td> <td> \image html preview_stock_objects_CornerXYZ.png </td> </tr>
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

			/** Returns a simple 3D model of a Hokuyo URG scanner.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::Hokuyo_URG() </td> <td> \image html preview_stock_objects_Hokuyo_URG.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP Hokuyo_URG();

			/** Returns a simple 3D model of a Hokuyo UTM scanner.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::Hokuyo_UTM() </td> <td> \image html preview_stock_objects_Hokuyo_UTM.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP Hokuyo_UTM();

			/** Returns a simple 3D model of a househam sprayer.
			  *  The generated object must be inserted in a opengl::COpenGLScene or opengl::CSetOfObjects.
			  *  <div align="center">
			  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			  *    <tr> <td> mrpt::opengl::stock_objects::Househam_Sprayer() </td> <td> \image html preview_stock_objects_Househam_Sprayer.png </td> </tr>
			  *  </table>
			  *  </div>
			  */
			CSetOfObjectsPtr OPENGL_IMPEXP Househam_Sprayer();

		} // end namespace stock_objects
	}

} // End of namespace


#endif
