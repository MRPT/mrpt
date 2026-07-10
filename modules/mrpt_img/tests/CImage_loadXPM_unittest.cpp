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

using namespace mrpt::img;

namespace
{
// A minimal, valid 2x2 XPM image: 2 colors, 1 char per pixel.
// Pixel layout:
//   R G
//   G R
const char* const validXpm2x2[] = {
    "2 2 2 1", "R c red", "G c green", "RG", "GR",
};

// A 3x2 XPM using the special "None" color for transparency handling
// (exercised via the mask-color substitution codepath).
const char* const xpmWithNone[] = {
    "2 2 2 1", "R c red", ". c None", "R.", ".R",
};
}  // namespace

TEST(CImage_loadXPM, ValidSmallImage)
{
  CImage img;
  ASSERT_TRUE(img.loadFromXPM(validXpm2x2, /*swap_rb=*/false));

  EXPECT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.getHeight(), 2);
  EXPECT_TRUE(img.isColor());

  // Row 0: "RG" -> red, green
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 255);  // R at (0,0) = red.R
  EXPECT_EQ(img.at<uint8_t>(1, 0, 1), 255);  // G at (1,0) = green.G ("green"=0,255,0 per rgbtab)
}

TEST(CImage_loadXPM, SwapRedBlue)
{
  CImage imgNoSwap;
  ASSERT_TRUE(imgNoSwap.loadFromXPM(validXpm2x2, false));

  CImage imgSwap;
  ASSERT_TRUE(imgSwap.loadFromXPM(validXpm2x2, true));

  // swap_rb should exchange the R and B channels of every pixel.
  EXPECT_EQ(imgNoSwap.at<uint8_t>(0, 0, 0), imgSwap.at<uint8_t>(0, 0, 2));
  EXPECT_EQ(imgNoSwap.at<uint8_t>(0, 0, 2), imgSwap.at<uint8_t>(0, 0, 0));
}

TEST(CImage_loadXPM, NoneColorIsSubstitutedWithMaskColor)
{
  CImage img;
  ASSERT_TRUE(img.loadFromXPM(xpmWithNone, false));
  EXPECT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.getHeight(), 2);
}

TEST(CImage_loadXPM, NullDataFails)
{
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(nullptr));
}

TEST(CImage_loadXPM, MalformedHeaderFails)
{
  const char* const badHeader[] = {"not a valid header", "R c red", "R"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(badHeader));
}

TEST(CImage_loadXPM, ZeroSizeHeaderFails)
{
  const char* const zeroSize[] = {"0 0 1 1", "R c red"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(zeroSize));
}

TEST(CImage_loadXPM, TruncatedColorLineFails)
{
  // Declares 1 color but the color definition line is too short.
  const char* const truncated[] = {"1 1 1 1", "R", "R"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(truncated));
}

TEST(CImage_loadXPM, MalformedColorDefinitionFails)
{
  // "c " target present but followed by garbage that isn't a valid color name.
  const char* const badColorDef[] = {"1 1 1 1", "R c not_a_real_color_name", "R"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(badColorDef));
}

TEST(CImage_loadXPM, UnknownColorTargetFails)
{
  // Color spec is missing the required "c "/"g "/... target token entirely.
  const char* const noTarget[] = {"1 1 1 1", "R xyz red", "R"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(noTarget));
}

TEST(CImage_loadXPM, TruncatedImageDataFails)
{
  // Header declares a 4x1 image but the pixel row is shorter than that.
  const char* const truncatedData[] = {"4 1 1 1", "R c red", "RR"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(truncatedData));
}

TEST(CImage_loadXPM, UnknownPixelKeyFails)
{
  // Pixel data references a key ('X') not present in the color table.
  const char* const badPixel[] = {"1 1 1 1", "R c red", "X"};
  CImage img;
  EXPECT_FALSE(img.loadFromXPM(badPixel));
}

TEST(CImage_loadXPM, HexColorSpec)
{
  const char* const hexColor[] = {"1 1 1 1", "R c #112233", "R"};
  CImage img;
  ASSERT_TRUE(img.loadFromXPM(hexColor, false));
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 0x11);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 1), 0x22);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 2), 0x33);
}

TEST(CImage_loadXPM, TwoCharsPerPixel)
{
  const char* const twoChar[] = {
      "2 1 2 2",
      "AA c red",
      "BB c blue",
      "AABB",
  };
  CImage img;
  ASSERT_TRUE(img.loadFromXPM(twoChar, false));
  EXPECT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 255);  // red
  EXPECT_EQ(img.at<uint8_t>(1, 0, 2), 255);  // blue
}

TEST(CImage_loadXPM, GrayscaleColorSpec)
{
  const char* const grayColor[] = {"1 1 1 1", "R g gray50", "R"};
  CImage img;
  ASSERT_TRUE(img.loadFromXPM(grayColor, false));
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 127);
}
