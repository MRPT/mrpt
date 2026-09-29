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

/** Helpers to inspect the pixels of a rendered frame in the offscreen render
 *  tests that assert geometric or color invariants.
 */
#pragma once

#include <mrpt/img/CImage.h>

#include <cstdlib>
#include <string>

namespace mrpt::opengl::testing
{
struct RGB
{
  int r, g, b;
};

/** The color of a pixel, whatever the channel order of the image is. */
inline RGB pixelRGB(const mrpt::img::CImage& im, int x, int y)
{
  const bool bgr = im.getChannelsOrder() == std::string("BGR");
  const int c0 = im.at<uint8_t>(x, y, 0);
  const int c1 = im.at<uint8_t>(x, y, 1);
  const int c2 = im.at<uint8_t>(x, y, 2);
  return bgr ? RGB{c2, c1, c0} : RGB{c0, c1, c2};
}

inline bool isNear(const RGB& a, const RGB& b, int tol = 40)
{
  return std::abs(a.r - b.r) <= tol && std::abs(a.g - b.g) <= tol && std::abs(a.b - b.b) <= tol;
}

/** Number of pixels in [x0,x1) x [y0,y1) that are close to the color `c`. */
inline int countColor(
    const mrpt::img::CImage& im, int x0, int y0, int x1, int y1, const RGB& c, int tol = 40)
{
  int n = 0;
  for (int y = y0; y < y1; y++)
  {
    for (int x = x0; x < x1; x++)
    {
      if (isNear(pixelRGB(im, x, y), c, tol))
      {
        n++;
      }
    }
  }
  return n;
}

}  // namespace mrpt::opengl::testing
