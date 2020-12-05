/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>

namespace mrpt::opengl
{
// clang-format off
/** \defgroup mrpt_adapters_grp Pointcloud adapter (wrapper) template classes (in #include <mrpt/opengl/pointcloud_adapters.h>)
 * \ingroup mrpt_opengl_grp
 */
// clang-format on

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
