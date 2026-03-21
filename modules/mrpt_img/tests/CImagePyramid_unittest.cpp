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
#include <mrpt/img/CImagePyramid.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

using namespace mrpt::img;
using namespace std::string_literals;

namespace
{
void fillRandom(CImage& img, unsigned int seed = 12345)
{
  mrpt::random::Randomize(seed);
  auto& rng = mrpt::random::getRandomGenerator();
  for (int y = 0; y < img.getHeight(); ++y)
  {
    for (int x = 0; x < img.getWidth(); ++x)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>(rng.drawUniform32bit());
    }
  }
}
}  // namespace

TEST(CImagePyramid, BuildPyramid_gray_sizes)
{
  CImage img(64, 64, CH_GRAY);
  fillRandom(img);

  CImagePyramid pyr;
  pyr.buildPyramid(img, 4, true);

  ASSERT_EQ(pyr.images.size(), 4U);
  EXPECT_EQ(pyr.images[0].getWidth(), 64);
  EXPECT_EQ(pyr.images[0].getHeight(), 64);
  EXPECT_EQ(pyr.images[1].getWidth(), 32);
  EXPECT_EQ(pyr.images[1].getHeight(), 32);
  EXPECT_EQ(pyr.images[2].getWidth(), 16);
  EXPECT_EQ(pyr.images[2].getHeight(), 16);
  EXPECT_EQ(pyr.images[3].getWidth(), 8);
  EXPECT_EQ(pyr.images[3].getHeight(), 8);
}

TEST(CImagePyramid, BuildPyramid_gray_noSmooth)
{
  CImage img(128, 64, CH_GRAY);
  fillRandom(img);

  CImagePyramid pyr;
  pyr.buildPyramid(img, 3, false);

  ASSERT_EQ(pyr.images.size(), 3U);
  EXPECT_EQ(pyr.images[0].getWidth(), 128);
  EXPECT_EQ(pyr.images[0].getHeight(), 64);
  EXPECT_EQ(pyr.images[1].getWidth(), 64);
  EXPECT_EQ(pyr.images[1].getHeight(), 32);
  EXPECT_EQ(pyr.images[2].getWidth(), 32);
  EXPECT_EQ(pyr.images[2].getHeight(), 16);
}

TEST(CImagePyramid, BuildPyramid_color)
{
  CImage img(64, 64, CH_RGB);

  CImagePyramid pyr;
  pyr.buildPyramid(img, 3, true);

  ASSERT_EQ(pyr.images.size(), 3U);
  EXPECT_EQ(pyr.images[0].getWidth(), 64);
  EXPECT_EQ(pyr.images[0].getHeight(), 64);
  EXPECT_TRUE(pyr.images[0].isColor());
  EXPECT_EQ(pyr.images[1].getWidth(), 32);
  EXPECT_EQ(pyr.images[1].getHeight(), 32);
}

TEST(CImagePyramid, BuildPyramid_color_toGray)
{
  CImage img(64, 64, CH_RGB);

  CImagePyramid pyr;
  pyr.buildPyramid(img, 3, true, true /* convert_grayscale */);

  ASSERT_EQ(pyr.images.size(), 3U);
  EXPECT_FALSE(pyr.images[0].isColor());
  EXPECT_EQ(pyr.images[0].getWidth(), 64);
  EXPECT_EQ(pyr.images[0].getHeight(), 64);
}

TEST(CImagePyramid, BuildPyramidFast)
{
  CImage img(64, 64, CH_GRAY);
  fillRandom(img);

  CImagePyramid pyr;
  pyr.buildPyramidFast(img, 3, true);

  ASSERT_EQ(pyr.images.size(), 3U);
  EXPECT_EQ(pyr.images[0].getWidth(), 64);
  EXPECT_EQ(pyr.images[0].getHeight(), 64);
  EXPECT_EQ(pyr.images[1].getWidth(), 32);
  EXPECT_EQ(pyr.images[2].getWidth(), 16);
}

TEST(CImagePyramid, BuildPyramid_singleOctave)
{
  CImage img(32, 32, CH_GRAY);
  fillRandom(img);

  CImagePyramid pyr;
  pyr.buildPyramid(img, 1, true);

  ASSERT_EQ(pyr.images.size(), 1U);
  EXPECT_EQ(pyr.images[0].getWidth(), 32);
  EXPECT_EQ(pyr.images[0].getHeight(), 32);
}

TEST(CImagePyramid, BuildPyramid_fromFile)
{
  const auto tstImg =
      mrpt::UNITTEST_BASEDIR() + "/../../mrpt_examples_cpp/img_basic_example/frame_color.jpg"s;

  if (!mrpt::system::fileExists(tstImg))
  {
    GTEST_SKIP() << "Test image not found: " << tstImg;
  }

  CImage img;
  ASSERT_TRUE(img.loadFromFile(tstImg));

  CImagePyramid pyr;
  pyr.buildPyramid(img, 4, true, true);

  ASSERT_EQ(pyr.images.size(), 4U);
  EXPECT_FALSE(pyr.images[0].isColor());
  EXPECT_EQ(pyr.images[0].getWidth(), img.getWidth());
  EXPECT_EQ(pyr.images[0].getHeight(), img.getHeight());

  for (size_t i = 1; i < pyr.images.size(); ++i)
  {
    EXPECT_EQ(pyr.images[i].getWidth(), pyr.images[i - 1].getWidth() / 2);
    EXPECT_EQ(pyr.images[i].getHeight(), pyr.images[i - 1].getHeight() / 2);
  }
}
