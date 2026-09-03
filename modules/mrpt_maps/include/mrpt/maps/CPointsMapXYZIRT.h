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

#include <mrpt/maps/CGenericPointsMap.h>

namespace mrpt::maps
{
/** Deserialization-only compatibility stub for the old CPointsMapXYZIRT class.
 *
 * This class carries no functionality of its own beyond decoding the binary
 * layout that CPointsMapXYZIRT used before it was replaced by
 * CGenericPointsMap. Its only purpose is to keep pre-existing
 * `.simplemap`/`.rawlog` files (or any other CSerializable-based archive)
 * that were saved with a CPointsMapXYZIRT layer loadable: without this class
 * being registered under its original name, CArchive::ReadObject() throws on
 * any such file with "Stored object has class 'mrpt::maps::CPointsMapXYZIRT'
 * which is not registered!".
 *
 * Once loaded, the point cloud is a regular CGenericPointsMap with fields
 * `intensity`, `ring`, and `t` populated as applicable, so all normal
 * CPointsMap/CGenericPointsMap APIs apply.
 *
 * \sa mrpt::maps::CGenericPointsMap
 * \ingroup mrpt_maps_grp
 */
class [[deprecated(
    "Use CGenericPointsMap instead. CPointsMapXYZIRT is kept only to "
    "deserialize pre-existing files.")]] CPointsMapXYZIRT : public CGenericPointsMap
{
  DEFINE_SERIALIZABLE(CPointsMapXYZIRT, mrpt::maps)

 public:
  CPointsMapXYZIRT() = default;
};

}  // namespace mrpt::maps
