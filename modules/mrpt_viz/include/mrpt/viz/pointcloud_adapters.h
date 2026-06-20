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

#include <cstdint>

namespace mrpt::viz
{
// clang-format off
/** \defgroup mrpt_adapters_grp Pointcloud adapter (wrapper) template classes (in #include <mrpt/viz/pointcloud_adapters.h>)
 * \ingroup mrpt_viz_grp
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

}  // namespace mrpt::viz
