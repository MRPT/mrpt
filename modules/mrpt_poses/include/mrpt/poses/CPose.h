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

#include <mrpt/poses/CPoseOrPoint.h>

namespace mrpt::poses
{
/** A base class for representing a pose in 2D or 3D.
 *   For more information refer to the 2D/3D Geometry tutorial online:
 *   https://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/
 * \note This class is based on the CRTP design pattern
 * \sa CPoseOrPoint, CPoint
 * \ingroup poses_grp
 */
template <class DERIVEDCLASS, std::size_t DIM>
class CPose : public CPoseOrPoint<DERIVEDCLASS, DIM>
{
};  // End of class def.

}  // namespace mrpt::poses
