/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <cstdint>

namespace mrpt::math
{

/** Types of geometric entities */
enum class GeometricEntity : uint8_t
{
  UNDEFINED = 0xff,
  POINT = 0,  //!< Object type for TPoint2D or TPoint3D  \sa TObject2D,TObject3D
  SEGMENT,    //!< Object type for TSegment2D or TSegment3D \sa TObject2D,TObject3D
  LINE,       //!< Object type for TLine2D or TLine3D \sa TObject2D,TObject3D
  POLYGON,    //!< Object type for TPolygon2D or TPolygon3D \sa TObject2D,TObject3D
  PLANE       //!< Object type for TPlane \sa TObject2D,TObject3D
};

}  // namespace mrpt::math
