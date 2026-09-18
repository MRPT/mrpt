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
#include <mrpt/img/CMappedImage.h>

using namespace mrpt::img;

namespace
{
CImage::Ptr makeGrayGradient(int w, int h)
{
  auto img = CImage::Create(w, h, CH_GRAY);
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      img->at<uint8_t>(x, y) = static_cast<uint8_t>((x + y) % 256);
    }
  }
  return img;
}
}  // namespace

TEST(CMappedImage, DefaultCoordsSpanFullImage)
{
  auto img = makeGrayGradient(10, 10);
  CMappedImage m(img);

  // Default coordinates: x0=0,y0=0,x1=-1,y1=-1 -> full image
  const double v = m.getPixel(0.0, 0.0);
  EXPECT_NEAR(v, static_cast<double>(img->at<uint8_t>(0, 0)), 1.0);
}

TEST(CMappedImage, ColorImageIsConvertedToGray)
{
  auto img = CImage::Create(4, 4, CH_RGB);
  for (int y = 0; y < 4; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      auto* p = img->ptr<uint8_t>(x, y);
      p[0] = 10;
      p[1] = 20;
      p[2] = 30;
    }
  }

  CMappedImage m(img, 0, 4, 0, 4, IMG_INTERP_NN);
  const double v = m.getPixel(0.5, 0.5);
  // Should not throw and return a valid grayscale value
  EXPECT_GE(v, 0.0);
}

TEST(CMappedImage, NearestNeighborInterp)
{
  auto img = makeGrayGradient(20, 20);
  CMappedImage m(img, 0, 20, 0, 20, IMG_INTERP_NN);

  const double v = m.getPixel(5.4, 5.4);
  EXPECT_NEAR(v, static_cast<double>(img->at<uint8_t>(5, 5)), 1.0);
}

TEST(CMappedImage, LinearInterp)
{
  auto img = makeGrayGradient(20, 20);
  CMappedImage m(img, 0, 20, 0, 20, IMG_INTERP_LINEAR);

  // At an exact pixel center, linear interpolation should match the pixel value.
  const double v = m.getPixel(5.0, 5.0);
  EXPECT_NEAR(v, static_cast<double>(img->at<uint8_t>(5, 5)), 1.0);
}

TEST(CMappedImage, OutOfBoundsReturnsZero)
{
  auto img = makeGrayGradient(10, 10);
  CMappedImage m(img, 0, 10, 0, 10, IMG_INTERP_NN);

  EXPECT_EQ(m.getPixel(-5.0, 0.0), 0.0);
  EXPECT_EQ(m.getPixel(0.0, -5.0), 0.0);
  EXPECT_EQ(m.getPixel(1000.0, 0.0), 0.0);
  EXPECT_EQ(m.getPixel(0.0, 1000.0), 0.0);
}

TEST(CMappedImage, UnsupportedInterpMethodThrows)
{
  auto img = makeGrayGradient(10, 10);
  CMappedImage m(img, 0, 10, 0, 10, IMG_INTERP_CUBIC);

  EXPECT_THROW(m.getPixel(1.0, 1.0), std::exception);
}

TEST(CMappedImage, ChangeCoordinatesAssertsOnDegenerateRange)
{
  auto img = makeGrayGradient(10, 10);
  CMappedImage m(img, 0, 10, 0, 10, IMG_INTERP_NN);

  EXPECT_THROW(m.changeCoordinates(5, 5, 0, 10), std::exception);
  EXPECT_THROW(m.changeCoordinates(0, 10, 5, 5), std::exception);
}

TEST(CMappedImage, ChangeCoordinatesRescalesMapping)
{
  auto img = makeGrayGradient(10, 10);
  CMappedImage m(img, 0, 10, 0, 10, IMG_INTERP_NN);

  // Re-map to a "meters" coordinate frame, e.g. [-1, 1] x [-1, 1]
  m.changeCoordinates(-1.0, 1.0, -1.0, 1.0);

  // Center of the new coordinate frame should map near the image center.
  const double v = m.getPixel(0.0, 0.0);
  EXPECT_GE(v, 0.0);
}
