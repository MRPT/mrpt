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
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/config.h>

#include <cstdint>
#include <vector>

using mrpt::hwdrivers::CImageGrabber_dc1394;
using mrpt::img::BayerPattern;

namespace
{
constexpr uint32_t W = 16;
constexpr uint32_t H = 12;

struct RGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

/** A uniform color as seen through a GBRG color filter. */
uint8_t gbrgMosaic(uint32_t x, uint32_t y, const RGB& c)
{
  if ((x % 2) == (y % 2))
  {
    return c.g;
  }
  return (y % 2 == 0) ? c.b : c.r;
}

/** A stereo frame as sent by a Bumblebee in MODE7_3: left, right byte pairs. */
std::vector<uint8_t> interleave(const std::vector<uint8_t>& left, const std::vector<uint8_t>& right)
{
  std::vector<uint8_t> frame(2 * left.size());
  for (size_t i = 0; i < left.size(); i++)
  {
    frame[2 * i] = left[i];
    frame[2 * i + 1] = right[i];
  }
  return frame;
}
}  // namespace

TEST(CImageGrabber_dc1394, decodeStereoFrameRawIsLossless)
{
#if !MRPT_HAS_LIBDC1394_2
  GTEST_SKIP() << "Built without libdc1394";
#else
  std::vector<uint8_t> left(W * H);
  std::vector<uint8_t> right(W * H);
  for (size_t i = 0; i < left.size(); i++)
  {
    left[i] = static_cast<uint8_t>((3 * i) % 256);
    right[i] = static_cast<uint8_t>((7 * i + 1) % 256);
  }
  const auto frame = interleave(left, right);

  mrpt::obs::CObservationStereoImages obs;
  CImageGrabber_dc1394::decodeStereoFrame(frame.data(), W, H, false, BayerPattern::GBRG, obs);

  ASSERT_TRUE(obs.hasImageRight);
  for (const auto* img : {&obs.imageLeft, &obs.imageRight})
  {
    EXPECT_FALSE(img->isColor());
    EXPECT_EQ(img->getWidth(), static_cast<int32_t>(W));
    EXPECT_EQ(img->getHeight(), static_cast<int32_t>(H));
  }
  for (uint32_t y = 0; y < H; y++)
  {
    for (uint32_t x = 0; x < W; x++)
    {
      const auto i = y * W + x;
      ASSERT_EQ(obs.imageLeft.at<uint8_t>(x, y), left[i]) << "x=" << x << " y=" << y;
      ASSERT_EQ(obs.imageRight.at<uint8_t>(x, y), right[i]) << "x=" << x << " y=" << y;
    }
  }
#endif
}

TEST(CImageGrabber_dc1394, decodeStereoFrameDemosaicsEachImage)
{
#if !MRPT_HAS_LIBDC1394_2
  GTEST_SKIP() << "Built without libdc1394";
#else
  const RGB colorL{50, 100, 200};
  const RGB colorR{220, 30, 10};
  std::vector<uint8_t> left(W * H);
  std::vector<uint8_t> right(W * H);
  for (uint32_t y = 0; y < H; y++)
  {
    for (uint32_t x = 0; x < W; x++)
    {
      left[y * W + x] = gbrgMosaic(x, y, colorL);
      right[y * W + x] = gbrgMosaic(x, y, colorR);
    }
  }
  const auto frame = interleave(left, right);

  mrpt::obs::CObservationStereoImages obs;
  CImageGrabber_dc1394::decodeStereoFrame(frame.data(), W, H, true, BayerPattern::GBRG, obs);

  ASSERT_TRUE(obs.imageLeft.isColor());
  ASSERT_TRUE(obs.imageRight.isColor());
  // Interior pixels (the borders of a demosaiced image are approximate):
  for (uint32_t y = 3; y + 3 < H; y++)
  {
    for (uint32_t x = 3; x + 3 < W; x++)
    {
      for (const auto& [img, c] :
           {std::make_pair(&obs.imageLeft, colorL), std::make_pair(&obs.imageRight, colorR)})
      {
        ASSERT_NEAR(img->at<uint8_t>(x, y, 0), c.r, 2) << "x=" << x << " y=" << y;
        ASSERT_NEAR(img->at<uint8_t>(x, y, 1), c.g, 2) << "x=" << x << " y=" << y;
        ASSERT_NEAR(img->at<uint8_t>(x, y, 2), c.b, 2) << "x=" << x << " y=" << y;
      }
    }
  }
#endif
}

TEST(CImageGrabber_dc1394, decodeStereoFrameRejectsNull)
{
#if !MRPT_HAS_LIBDC1394_2
  GTEST_SKIP() << "Built without libdc1394";
#else
  mrpt::obs::CObservationStereoImages obs;
  EXPECT_THROW(
      CImageGrabber_dc1394::decodeStereoFrame(nullptr, W, H, false, BayerPattern::GBRG, obs),
      std::exception);
#endif
}
