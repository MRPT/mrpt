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
#include <iosfwd>

namespace mrpt::img
{
/** Generic template for pixel coordinates
 * \ingroup mrpt_img_grp
 */
template <typename T>
struct TPixelCoordBase
{
  /** The type of \a x and \a y */
  using pixel_coord_t = T;

  pixel_coord_t x{0}, y{0};

  TPixelCoordBase() = default;

  TPixelCoordBase(const T _x, const T _y) : x(_x), y(_y) {}
  template <typename U>
  explicit TPixelCoordBase(const TPixelCoordBase<U>& o) :
      x(static_cast<T>(o.x)), y(static_cast<T>(o.y))
  {
  }

  bool operator==(const TPixelCoordBase& o) const { return x == o.x && y == o.y; }
  TPixelCoordBase operator+(const TPixelCoordBase& o) const { return {x + o.x, y + o.y}; }
  TPixelCoordBase operator-(const TPixelCoordBase& o) const { return {x - o.x, y - o.y}; }
};

/** Prints TPixelCoordBase as "(x,y)" */
template <typename T>
std::ostream& operator<<(std::ostream& o, const TPixelCoordBase<T>& p);

/** A pair (x,y) of pixel coordinates (integer resolution). \ingroup mrpt_img_grp  */
using TPixelCoord = TPixelCoordBase<int32_t>;

/** A type for image sizes. \ingroup mrpt_img_grp  */
using TImageSize = TPixelCoord;

/** A pair (x,y) of pixel coordinates (subpixel resolution). \ingroup mrpt_img_grp  */
using TPixelCoordf = TPixelCoordBase<float>;

}  // namespace mrpt::img
