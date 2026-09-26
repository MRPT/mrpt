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

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <utility>
#include <vector>

using mrpt::img::BayerPattern;
using mrpt::img::CImage;
using mrpt::img::PixelDepth;

namespace
{
const std::array<BayerPattern, 4> kAllPatterns = {
    BayerPattern::RGGB, BayerPattern::BGGR, BayerPattern::GBRG, BayerPattern::GRBG};

using rgb_t = std::array<int, 3>;
using color_fn_t = std::function<rgb_t(int, int)>;

/** Colors of the top-left 2x2 block, in row-major order. */
const char* layoutOf(BayerPattern p)
{
  switch (p)
  {
    case BayerPattern::BGGR:
      return "BGGR";
    case BayerPattern::GBRG:
      return "GBRG";
    case BayerPattern::GRBG:
      return "GRBG";
    default:
      return "RGGB";
  }
}

/** Which channel (0=R, 1=G, 2=B) the sensor samples at (x,y). */
int channelAt(BayerPattern p, int x, int y)
{
  const char c = layoutOf(p)[(y & 1) * 2 + (x & 1)];
  return c == 'R' ? 0 : (c == 'G' ? 1 : 2);
}

/** Simulates a Bayer sensor looking at a scene of colors `fn`, with `pad` extra bytes per row. */
template <typename T>
std::vector<uint8_t> mosaic(BayerPattern p, int w, int h, std::size_t pad, const color_fn_t& fn)
{
  const std::size_t stride = w * sizeof(T) + pad;
  std::vector<uint8_t> raw(stride * h, 0xAB);
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      const T v = static_cast<T>(fn(x, y)[channelAt(p, x, y)]);
      std::memcpy(raw.data() + stride * y + sizeof(T) * x, &v, sizeof(T));
    }
  }
  return raw;
}

template <typename T>
rgb_t pixelAt(const CImage& img, int x, int y)
{
  const T* px = img.ptrLine<T>(y) + 3 * x;
  return {px[0], px[1], px[2]};
}

/** Checks the demosaiced image: the sampled channel must be the raw value everywhere, and all
 * channels must match `fn` exactly in the interior (bilinear interpolation reproduces a linear
 * scene exactly). */
template <typename T>
void checkLinearScene(BayerPattern p, int w, int h, std::size_t pad, const color_fn_t& fn)
{
  constexpr auto depth = sizeof(T) == 2 ? PixelDepth::D16U : PixelDepth::D8U;
  const auto raw = mosaic<T>(p, w, h, pad, fn);

  CImage img;
  img.loadFromBayerBuffer(w, h, raw.data(), w * sizeof(T) + pad, p, depth);
  ASSERT_EQ(img.getWidth(), w);
  ASSERT_EQ(img.getHeight(), h);
  ASSERT_EQ(img.channels(), mrpt::img::CH_RGB);
  ASSERT_EQ(img.getPixelDepth(), depth);

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      const rgb_t got = pixelAt<T>(img, x, y);
      const rgb_t expected = fn(x, y);
      const int ch = channelAt(p, x, y);
      EXPECT_EQ(got[ch], expected[ch]) << "sampled ch=" << ch << " x=" << x << " y=" << y;

      const bool interior = x > 0 && y > 0 && x < w - 1 && y < h - 1;
      if (interior)
      {
        EXPECT_EQ(got, expected) << "x=" << x << " y=" << y << " pattern=" << static_cast<int>(p);
      }
    }
  }
}
}  // namespace

TEST(CImage, LoadFromBayerBufferConstantColor)
{
  // A uniform scene must come out uniform, borders included, and with the
  // channels in their right place for every pattern.
  const color_fn_t constant = [](int, int) { return rgb_t{200, 100, 30}; };
  for (const auto p : kAllPatterns)
  {
    for (const auto& [w, h] : {
             std::pair{8, 6},
             std::pair{7, 5},
             std::pair{2, 2}
    })
    {
      const auto raw = mosaic<uint8_t>(p, w, h, 0, constant);
      CImage img;
      img.loadFromBayerBuffer(w, h, raw.data(), w, p);
      for (int y = 0; y < h; y++)
      {
        for (int x = 0; x < w; x++)
        {
          EXPECT_EQ(pixelAt<uint8_t>(img, x, y), (rgb_t{200, 100, 30}))
              << "x=" << x << " y=" << y << " pattern=" << static_cast<int>(p);
        }
      }
    }
  }
}

TEST(CImage, LoadFromBayerBufferLinearScene8bit)
{
  const color_fn_t linear = [](int x, int y) {
    return rgb_t{2 * x + 3 * y + 5, x + 4 * y + 7, 3 * x + y + 1};
  };
  for (const auto p : kAllPatterns)
  {
    checkLinearScene<uint8_t>(p, 32, 24, 0, linear);
    checkLinearScene<uint8_t>(p, 31, 23, 0, linear);
  }
}

TEST(CImage, LoadFromBayerBufferLinearScene16bit)
{
  const color_fn_t linear = [](int x, int y) {
    return rgb_t{600 * x + 900 * y + 5, 300 * x + 1200 * y + 7, 900 * x + 300 * y + 1};
  };
  for (const auto p : kAllPatterns)
  {
    checkLinearScene<uint16_t>(p, 32, 24, 0, linear);
  }
}

TEST(CImage, LoadFromBayerBufferRowPadding)
{
  const color_fn_t linear = [](int x, int y) {
    return rgb_t{2 * x + 3 * y + 5, x + 4 * y + 7, 3 * x + y + 1};
  };
  for (const auto p : kAllPatterns)
  {
    checkLinearScene<uint8_t>(p, 16, 10, 5, linear);
    checkLinearScene<uint16_t>(p, 16, 10, 6, linear);
  }
}

TEST(CImage, LoadFromBayerBufferInvalidArgsThrow)
{
  const std::vector<uint8_t> raw(64, 0);
  CImage img;
  EXPECT_THROW(img.loadFromBayerBuffer(1, 4, raw.data(), 8, BayerPattern::RGGB), std::exception);
  EXPECT_THROW(img.loadFromBayerBuffer(4, 1, raw.data(), 8, BayerPattern::RGGB), std::exception);
  EXPECT_THROW(img.loadFromBayerBuffer(8, 4, raw.data(), 4, BayerPattern::RGGB), std::exception);
  EXPECT_THROW(
      img.loadFromBayerBuffer(4, 4, raw.data(), 9, BayerPattern::RGGB, PixelDepth::D16U),
      std::exception);
  EXPECT_THROW(img.loadFromBayerBuffer(4, 4, nullptr, 8, BayerPattern::RGGB), std::exception);
}
