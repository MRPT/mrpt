/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>

namespace mrpt::opengl
{
/** \defgroup mrpt_adapters_grp Pointcloud adapter (wrapper) template classes
   (in #include <mrpt/opengl/pointcloud_adapters.h>) \addtogroup mrpt_opengl_grp
*/

/** \addtogroup mrpt_adapters_grp
 * @{ */

/** An adapter to different kinds of point cloud object.
 *  Implemented as a pure C++ template with specializations for the highest
 * flexibility and efficiency in compiler-generated implementations.
 *  Usage:
 *   \code
 *     PC  my_obj;
 *     my_obj.specific_methods();
 *     // ...
 *     PointCloudAdapter<PC> pca(my_obj);
 *     pca.unified_interface_methods();
 *     // ...
 *   \endcode
 *  See specializations for details on the exposed API.
 */
template <class POINTCLOUD>
class PointCloudAdapter;

/** @} */  // end of grouping

}  // namespace mrpt::opengl
