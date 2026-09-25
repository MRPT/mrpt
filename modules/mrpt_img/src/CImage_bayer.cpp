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

#include <mrpt/core/exceptions.h>
#include <mrpt/img/CImage.h>

#include <cstddef>
#include <cstdint>

using namespace mrpt::img;

namespace
{
/** Mirrors an out-of-range index without repeating the edge sample, so the mirrored sample has
 * the same color filter as the missing one. */
int reflect101(int i, int n)
{
  if (i < 0)
  {
    return -i;
  }
  if (i >= n)
  {
    return 2 * n - 2 - i;
  }
  return i;
}

// Wide enough for the sum of four 16-bit samples:
using acc_t = uint32_t;

/** Raw rows around the one being demosaiced (already mirrored at the image borders). */
template <typename T>
struct RowWindow
{
  const T* up;
  const T* cur;
  const T* down;
  T* dst;
  bool redRow;
};

/** Red or blue site: green from the 4 direct neighbors, the other color from diagonals. */
template <typename T>
inline void colorSite(const RowWindow<T>& w, int x, int xl, int xr)
{
  const acc_t c = w.cur[x];
  const acc_t cross = (acc_t(w.cur[xl]) + w.cur[xr] + w.up[x] + w.down[x] + 2) >> 2;
  const acc_t diag = (acc_t(w.up[xl]) + w.up[xr] + w.down[xl] + w.down[xr] + 2) >> 2;
  T* px = w.dst + 3 * x;
  px[0] = static_cast<T>(w.redRow ? c : diag);
  px[1] = static_cast<T>(cross);
  px[2] = static_cast<T>(w.redRow ? diag : c);
}

/** Green site: the row neighbors share the row color, the column ones the other. */
template <typename T>
inline void greenSite(const RowWindow<T>& w, int x, int xl, int xr)
{
  const acc_t horz = (acc_t(w.cur[xl]) + w.cur[xr] + 1) >> 1;
  const acc_t vert = (acc_t(w.up[x]) + w.down[x] + 1) >> 1;
  T* px = w.dst + 3 * x;
  px[0] = static_cast<T>(w.redRow ? horz : vert);
  px[1] = w.cur[x];
  px[2] = static_cast<T>(w.redRow ? vert : horz);
}

template <typename T>
void demosaicRow(const RowWindow<T>& w, int width, int redX)
{
  const auto isColorSite = [&](int x) { return ((x & 1) == redX) == w.redRow; };
  const auto anyPixel = [&](int x)
  {
    const int xl = (x == 0) ? 1 : x - 1;
    const int xr = (x == width - 1) ? width - 2 : x + 1;
    if (isColorSite(x))
    {
      colorSite(w, x, xl, xr);
    }
    else
    {
      greenSite(w, x, xl, xr);
    }
  };

  anyPixel(0);

  // Interior, two pixels at a time so the site types are known without branching per pixel:
  int x = 1;
  if (isColorSite(1))
  {
    for (; x + 2 < width; x += 2)
    {
      colorSite(w, x, x - 1, x + 1);
      greenSite(w, x + 1, x, x + 2);
    }
  }
  else
  {
    for (; x + 2 < width; x += 2)
    {
      greenSite(w, x, x - 1, x + 1);
      colorSite(w, x + 1, x, x + 2);
    }
  }

  for (; x < width; x++)
  {
    anyPixel(x);
  }
}

template <typename T>
void demosaicBilinear(
    const uint8_t* raw,
    std::size_t rowStrideBytes,
    int width,
    int height,
    int redX,
    int redY,
    CImage& out)
{
  const auto rawRow = [&](int y)
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const T*>(
        raw + rowStrideBytes * static_cast<std::size_t>(reflect101(y, height)));
  };

  for (int y = 0; y < height; y++)
  {
    const RowWindow<T> w{
        rawRow(y - 1), rawRow(y), rawRow(y + 1), out.ptrLine<T>(y), (y & 1) == redY};
    demosaicRow(w, width, redX);
  }
}
}  // namespace

void CImage::loadFromBayerBuffer(
    int32_t width,
    int32_t height,
    const uint8_t* rawpixels,
    std::size_t rowStrideBytes,
    BayerPattern pattern,
    PixelDepth depth)
{
  MRPT_START

  ASSERT_(rawpixels != nullptr);
  ASSERT_GE_(width, 2);
  ASSERT_GE_(height, 2);
  const auto bytesPerSample = static_cast<std::size_t>(depth);
  ASSERT_GE_(rowStrideBytes, static_cast<std::size_t>(width) * bytesPerSample);
  ASSERT_EQUAL_(rowStrideBytes % bytesPerSample, 0U);

  // Position of the red sample within each 2x2 block (blue is at the opposite corner):
  int redX = 0;
  int redY = 0;
  switch (pattern)
  {
    case BayerPattern::RGGB:
      break;
    case BayerPattern::BGGR:
      redX = 1;
      redY = 1;
      break;
    case BayerPattern::GBRG:
      redY = 1;
      break;
    case BayerPattern::GRBG:
      redX = 1;
      break;
    default:
      THROW_EXCEPTION("Invalid BayerPattern value");
  }

  resize(width, height, CH_RGB, depth);

  if (depth == PixelDepth::D16U)
  {
    demosaicBilinear<uint16_t>(rawpixels, rowStrideBytes, width, height, redX, redY, *this);
  }
  else
  {
    demosaicBilinear<uint8_t>(rawpixels, rowStrideBytes, width, height, redX, redY, *this);
  }

  MRPT_END
}
