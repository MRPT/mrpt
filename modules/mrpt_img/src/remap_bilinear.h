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

#include <mrpt/img/CImage.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mrpt::img::detail
{
/** Apply a bilinear-interpolation remap to an image.
 *  mapx[row * out_w + col] and mapy[...] give the source (x,y) float
 *  pixel coordinates for each output pixel.  Out-of-bounds source pixels
 *  produce black (zero) output.
 *
 *  Works for 1-channel (grayscale), 3-channel (RGB), and 4-channel (RGBA)
 *  8-bit images.
 */
inline void remap_bilinear(
    const CImage& in, CImage& out, const float* mapx, const float* mapy, int out_w, int out_h)
{
  const int in_w = in.getWidth();
  const int in_h = in.getHeight();
  const int nch = static_cast<int>(in.channels());
  const size_t in_stride = in.getRowStride();

  out.resize(out_w, out_h, in.channels());
  const size_t out_stride = out.getRowStride();

  const uint8_t* in_data = in.ptrLine<uint8_t>(0);
  uint8_t* out_data = out.ptrLine<uint8_t>(0);

  for (int v = 0; v < out_h; ++v)
  {
    uint8_t* out_row = out_data + static_cast<size_t>(v) * out_stride;
    const int row_off = v * out_w;

    for (int u = 0; u < out_w; ++u)
    {
      const float sx = mapx[row_off + u];
      const float sy = mapy[row_off + u];

      // Integer and fractional parts
      const int ix = static_cast<int>(std::floor(sx));
      const int iy = static_cast<int>(std::floor(sy));

      // Boundary check: need ix, ix+1 in [0, in_w-1] and iy, iy+1 in [0, in_h-1]
      if (ix < 0 || ix + 1 >= in_w || iy < 0 || iy + 1 >= in_h)
      {
        // Out of bounds -> black pixel
        for (int c = 0; c < nch; ++c)
        {
          out_row[u * nch + c] = 0;
        }
        continue;
      }

      const float fx = sx - static_cast<float>(ix);
      const float fy = sy - static_cast<float>(iy);
      const float w00 = (1.0f - fx) * (1.0f - fy);
      const float w10 = fx * (1.0f - fy);
      const float w01 = (1.0f - fx) * fy;
      const float w11 = fx * fy;

      const uint8_t* p00 = in_data + static_cast<size_t>(iy) * in_stride + ix * nch;
      const uint8_t* p10 = p00 + nch;
      const uint8_t* p01 = p00 + in_stride;
      const uint8_t* p11 = p01 + nch;

      for (int c = 0; c < nch; ++c)
      {
        const float val = w00 * p00[c] + w10 * p10[c] + w01 * p01[c] + w11 * p11[c];
        out_row[u * nch + c] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, val + 0.5f)));
      }
    }
  }
}

}  // namespace mrpt::img::detail
