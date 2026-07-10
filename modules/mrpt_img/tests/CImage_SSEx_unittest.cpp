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

// Direct unit tests for the low-level SSE2/SSSE3 pixel kernels in
// CImage.SSE2.cpp / CImage.SSSE3.cpp.
//
// As of the stb-based rewrite of CImage's I/O and resizing (MRPT 3.0), these
// kernels are no longer invoked from CImage.cpp itself (scaleHalf() and
// grayscale() now go through the generic stb resize / a plain C++ loop
// instead). They are exercised here directly, as isolated pixel-processing
// kernels, so their (still-linked) code doesn't sit completely untested.

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_ARCH_INTEL_COMPATIBLE
#include <mrpt/img/registerAllClasses.h>

#include <cstdint>
#include <vector>

#if MRPT_ARCH_INTEL_COMPATIBLE

// Forward declarations of the free functions defined in CImage.SSE2.cpp /
// CImage.SSSE3.cpp (declared in the private header src/CImage.SSEx.h).
void image_SSE2_scale_half_1c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_scale_half_3c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSE2_scale_half_smooth_1c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_rgb_to_gray_8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_bgr_to_gray_8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);

namespace
{
std::vector<uint8_t> makeGray(int w, int h)
{
  std::vector<uint8_t> v(static_cast<size_t>(w) * static_cast<size_t>(h));
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      v[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>((x + y * 7) % 256);
    }
  }
  return v;
}

std::vector<uint8_t> makeColor(int w, int h)
{
  std::vector<uint8_t> v(static_cast<size_t>(w) * static_cast<size_t>(h) * 3);
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      const size_t off = (static_cast<size_t>(y) * w + x) * 3;
      v[off + 0] = static_cast<uint8_t>((x * 3 + y) % 256);
      v[off + 1] = static_cast<uint8_t>((x * 5 + y * 2) % 256);
      v[off + 2] = static_cast<uint8_t>((x * 7 + y * 3) % 256);
    }
  }
  return v;
}
}  // namespace

// image_SSE2_scale_half_1c8u(): decimates by taking the top-left pixel of
// each 2x2 block (no averaging).
TEST(CImage_SSEx, ScaleHalf1c8u_MultipleOf16)
{
  const int w = 32;
  const int h = 4;
  const auto in = makeGray(w, h);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2), 0);

  image_SSE2_scale_half_1c8u(in.data(), out.data(), w, h, w, w / 2);

  for (int oy = 0; oy < h / 2; oy++)
  {
    for (int ox = 0; ox < w / 2; ox++)
    {
      EXPECT_EQ(
          out[static_cast<size_t>(oy) * (w / 2) + ox], in[static_cast<size_t>(2 * oy) * w + 2 * ox])
          << "at (" << ox << "," << oy << ")";
    }
  }
}

TEST(CImage_SSEx, ScaleHalf1c8u_WithRemainder)
{
  // Width not a multiple of 16: exercises the tail/remainder loop.
  const int w = 20;
  const int h = 2;
  const auto in = makeGray(w, h);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2), 0);

  image_SSE2_scale_half_1c8u(in.data(), out.data(), w, h, w, w / 2);

  for (int ox = 0; ox < w / 2; ox++)
  {
    EXPECT_EQ(out[static_cast<size_t>(ox)], in[static_cast<size_t>(2 * ox)]) << "at ox=" << ox;
  }
}

// image_SSE2_scale_half_smooth_1c8u(): 2x2 block arithmetic average.
TEST(CImage_SSEx, ScaleHalfSmooth1c8u_UniformBlock)
{
  const int w = 32;
  const int h = 4;
  std::vector<uint8_t> in(static_cast<size_t>(w) * h, 100);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2), 0);

  image_SSE2_scale_half_smooth_1c8u(in.data(), out.data(), w, h, w, w / 2);

  for (auto v : out)
  {
    EXPECT_EQ(v, 100);
  }
}

TEST(CImage_SSEx, ScaleHalfSmooth1c8u_WithRemainderStaysInRange)
{
  const int w = 20;
  const int h = 2;
  const auto in = makeGray(w, h);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2), 0);

  image_SSE2_scale_half_smooth_1c8u(in.data(), out.data(), w, h, w, w / 2);

  // Every averaged output pixel must lie within [0,255] (sanity: no
  // overflow/underflow), and in particular within the min/max of its 2x2
  // source block.
  for (int ox = 0; ox < w / 2; ox++)
  {
    const int a = in[2 * ox];
    const int b = in[2 * ox + 1];
    const int c = in[w + 2 * ox];
    const int d = in[w + 2 * ox + 1];
    const int lo = std::min({a, b, c, d});
    const int hi = std::max({a, b, c, d});
    EXPECT_GE(out[static_cast<size_t>(ox)], lo);
    EXPECT_LE(out[static_cast<size_t>(ox)], hi);
  }
}

// image_SSSE3_scale_half_3c8u(): 3-channel decimation (top-left pixel of
// each 2x2 block, no averaging).
TEST(CImage_SSEx, ScaleHalf3c8u_MultipleOf16)
{
  const int w = 32;
  const int h = 4;
  const auto in = makeColor(w, h);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2) * 3, 0);

  image_SSSE3_scale_half_3c8u(
      in.data(), out.data(), w, h, static_cast<size_t>(w) * 3, static_cast<size_t>(w / 2) * 3);

  for (int oy = 0; oy < h / 2; oy++)
  {
    for (int ox = 0; ox < w / 2; ox++)
    {
      const size_t outOff = (static_cast<size_t>(oy) * (w / 2) + ox) * 3;
      const size_t inOff = (static_cast<size_t>(2 * oy) * w + 2 * ox) * 3;
      for (int c = 0; c < 3; c++)
      {
        EXPECT_EQ(out[outOff + c], in[inOff + c]) << "at (" << ox << "," << oy << ") c=" << c;
      }
    }
  }
}

TEST(CImage_SSEx, ScaleHalf3c8u_WithRemainder)
{
  const int w = 20;
  const int h = 2;
  const auto in = makeColor(w, h);
  std::vector<uint8_t> out(static_cast<size_t>(w / 2) * (h / 2) * 3, 0);

  image_SSSE3_scale_half_3c8u(
      in.data(), out.data(), w, h, static_cast<size_t>(w) * 3, static_cast<size_t>(w / 2) * 3);

  for (int ox = 0; ox < w / 2; ox++)
  {
    const size_t outOff = static_cast<size_t>(ox) * 3;
    const size_t inOff = static_cast<size_t>(2 * ox) * 3;
    for (int c = 0; c < 3; c++)
    {
      EXPECT_EQ(out[outOff + c], in[inOff + c]) << "at ox=" << ox << " c=" << c;
    }
  }
}

namespace
{
// Uniform-color test blocks: since every pixel in the block carries the same
// (r,g,b), the result is insensitive to exactly *which* shuffle lane routes
// which source pixel to which output position (only that R/G/B end up
// weighted by their respective documented factors), which keeps this test
// robust without having to hand-verify the full byte-shuffle permutation.
std::vector<uint8_t> makeUniformRgb(int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
  std::vector<uint8_t> v(static_cast<size_t>(w) * static_cast<size_t>(h) * 3);
  for (size_t i = 0; i < v.size(); i += 3)
  {
    v[i + 0] = r;
    v[i + 1] = g;
    v[i + 2] = b;
  }
  return v;
}

void checkUniformGray(
    void (*fn)(const uint8_t*, uint8_t*, int, int, size_t, size_t),
    int w,
    int h,
    uint8_t r,
    uint8_t g,
    uint8_t b,
    uint8_t expected,
    const char* label)
{
  const auto in = makeUniformRgb(w, h, r, g, b);
  std::vector<uint8_t> out(static_cast<size_t>(w) * h, 0);
  fn(in.data(), out.data(), w, h, static_cast<size_t>(w) * 3, static_cast<size_t>(w));

  for (auto v : out)
  {
    EXPECT_NEAR(v, expected, 2) << label;
  }
}
}  // namespace

TEST(CImage_SSEx, RgbToGray8u_UniformColors)
{
  const int w = 16;
  const int h = 2;
  // Expected values follow the documented Y = 77*R + 150*G + 29*B formula.
  checkUniformGray(image_SSSE3_rgb_to_gray_8u, w, h, 255, 0, 0, 76, "red");
  checkUniformGray(image_SSSE3_rgb_to_gray_8u, w, h, 0, 255, 0, 149, "green");
  checkUniformGray(image_SSSE3_rgb_to_gray_8u, w, h, 0, 0, 255, 28, "blue");
  checkUniformGray(image_SSSE3_rgb_to_gray_8u, w, h, 255, 255, 255, 255, "white");
  checkUniformGray(image_SSSE3_rgb_to_gray_8u, w, h, 0, 0, 0, 0, "black");
}

TEST(CImage_SSEx, BgrToGray8u_UniformColors)
{
  const int w = 16;
  const int h = 2;
  // Note: makeUniformRgb still stores (r,g,b) triplets in memory order, but
  // for the bgr_to_gray_8u kernel that memory order is interpreted as
  // (b,g,r). Passing swapped arguments keeps the *semantic* R/G/B values,
  // and thus the expected gray levels, the same as the RGB test above.
  checkUniformGray(image_SSSE3_bgr_to_gray_8u, w, h, 0, 0, 255, 76, "red");
  checkUniformGray(image_SSSE3_bgr_to_gray_8u, w, h, 0, 255, 0, 149, "green");
  checkUniformGray(image_SSSE3_bgr_to_gray_8u, w, h, 255, 0, 0, 28, "blue");
  checkUniformGray(image_SSSE3_bgr_to_gray_8u, w, h, 255, 255, 255, 255, "white");
  checkUniformGray(image_SSSE3_bgr_to_gray_8u, w, h, 0, 0, 0, 0, "black");
}

#endif  // MRPT_ARCH_INTEL_COMPATIBLE

TEST(CImage_registerAllClasses, RunsWithoutError)
{
  mrpt::img::registerAllClasses_mrpt_img();
  SUCCEED();
}
