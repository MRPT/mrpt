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
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::RobotRhodon() </td> <td> \image
 * html preview_stock_objects_RobotRhodon.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr RobotRhodon();

/** Returns a representation of RobotGiraff.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::RobotGiraff() </td> <td> \image
 * html preview_stock_objects_RobotGiraff.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr RobotGiraff();

/** Returns a representation of a Pioneer II mobile base.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::RobotPioneer() </td> <td> \image
 * html preview_stock_objects_RobotPioneer.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr RobotPioneer();

/** Returns three arrows representing a X,Y,Z 3D corner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZSimple, CornerXYSimple, CornerXYZEye
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::CornerXYZ() </td> <td> \image html
 * preview_stock_objects_CornerXYZ.png </td> </tr>
 *  </table>
 *  </div>
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
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::CornerXYZ() </td> <td> \image html
 * preview_stock_objects_CornerXYZ.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr CornerXYZEye();

/** Returns three arrows representing a X,Y,Z 3D corner (just thick lines
 * instead of complex arrows for faster rendering than CornerXYZ).
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZ, CornerXYSimple
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::CornerXYZSimple() </td> <td>
 * \image html preview_stock_objects_CornerXYZSimple.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr CornerXYZSimple(float scale = 1.0, float lineWidth = 1.0);

/** Returns two arrows representing a X,Y 2D corner (just thick lines, fast to
 * render).
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 * \sa CornerXYZSimple, CornerXYZ, CornerXYZEye
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::CornerXYSimple() </td> <td> \image
 * html preview_stock_objects_CornerXYSimple.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr CornerXYSimple(float scale = 1.0, float lineWidth = 1.0);

/** Returns a simple 3D model of a PointGrey Bumblebee stereo camera.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::BumblebeeCamera() </td> <td>
 * \image html preview_stock_objects_BumblebeeCamera.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr BumblebeeCamera();

/** Returns a simple 3D model of a Hokuyo URG scanner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::Hokuyo_URG() </td> <td> \image
 * html preview_stock_objects_Hokuyo_URG.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr Hokuyo_URG();

/** Returns a simple 3D model of a Hokuyo UTM scanner.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::Hokuyo_UTM() </td> <td> \image
 * html preview_stock_objects_Hokuyo_UTM.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr Hokuyo_UTM();

/** Returns a simple 3D model of a househam sprayer.
 *  The generated object must be inserted in a opengl::Scene or
 * opengl::CSetOfObjects.
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *    <tr> <td> mrpt::viz::stock_objects::Househam_Sprayer() </td> <td>
 * \image html preview_stock_objects_Househam_Sprayer.png </td> </tr>
 *  </table>
 *  </div>
 */
CSetOfObjects::Ptr Househam_Sprayer();

}  // namespace stock_objects

}  // namespace mrpt::viz
