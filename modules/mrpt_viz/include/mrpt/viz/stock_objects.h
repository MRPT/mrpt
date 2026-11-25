/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/viz/CSetOfObjects.h>

namespace mrpt::viz
{
/** A collection of pre-built 3D objects for quick insertion in
 * opengl::Scene objects.
 * \ingroup mrpt_viz_grp
 */
namespace stock_objects
{
/** Returns a representation of Rhodon.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::RobotRhodon()](preview_stock_objects_RobotRhodon.png)
 */
CSetOfObjects::Ptr RobotRhodon();

/** Returns a representation of RobotGiraff.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 */
CSetOfObjects::Ptr RobotGiraff();

/** Returns a representation of a Pioneer II mobile base.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::RobotPioneer()](preview_stock_objects_RobotPioneer.png)
 */
CSetOfObjects::Ptr RobotPioneer();

/** Returns three arrows representing a X,Y,Z 3D corner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZSimple, CornerXYSimple, CornerXYZEye
 * ![mrpt::viz::stock_objects::CornerXYZ()](preview_stock_objects_CornerXYZ.png)
 */
CSetOfObjects::Ptr CornerXYZ(float scale = 1.0);

/** Returns three arrows representing a X,Y,Z 3D corner.
 *  Differently from CornerXYZ the arrowhead of Z axis ends where the object is
 * placed.
 *  This is useful if you want to place this object with the same position and
 * orientation of a camera.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZSimple, CornerXYSimple
 * ![mrpt::viz::stock_objects::CornerXYZ()](preview_stock_objects_CornerXYZ.png)
 */
CSetOfObjects::Ptr CornerXYZEye();

/** Returns three arrows representing a X,Y,Z 3D corner (just thick lines
 * instead of complex arrows for faster rendering than CornerXYZ).
 * \param scale Length of each axis line.
 * \param lineWidth Width of the lines, in pixels.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZ, CornerXYSimple
 * ![mrpt::viz::stock_objects::CornerXYZSimple()](preview_stock_objects_CornerXYZSimple.png)
 */
CSetOfObjects::Ptr CornerXYZSimple(float scale = 1.0, float lineWidth = 1.0);

/** Returns two arrows representing a X,Y 2D corner (just thick lines, fast to
 * render).
 * \param scale Length of each axis line.
 * \param lineWidth Width of the lines, in pixels.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZSimple, CornerXYZ, CornerXYZEye
 * ![mrpt::viz::stock_objects::CornerXYSimple()](preview_stock_objects_CornerXYSimple.png)
 */
CSetOfObjects::Ptr CornerXYSimple(float scale = 1.0, float lineWidth = 1.0);

/** Returns a simple 3D model of a PointGrey Bumblebee stereo camera.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::BumblebeeCamera()](preview_stock_objects_BumblebeeCamera.png)
 */
CSetOfObjects::Ptr BumblebeeCamera();

/** Returns a simple 3D model of a Hokuyo URG scanner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::Hokuyo_URG()](preview_stock_objects_Hokuyo_URG.png)
 */
CSetOfObjects::Ptr Hokuyo_URG();

/** Returns a simple 3D model of a Hokuyo UTM scanner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::Hokuyo_UTM()](preview_stock_objects_Hokuyo_UTM.png)
 */
CSetOfObjects::Ptr Hokuyo_UTM();

/** Returns a simple 3D model of a househam sprayer.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * ![mrpt::viz::stock_objects::Househam_Sprayer()](preview_stock_objects_Househam_Sprayer.png)
 */
CSetOfObjects::Ptr Househam_Sprayer();

}  // namespace stock_objects

}  // namespace mrpt::viz
