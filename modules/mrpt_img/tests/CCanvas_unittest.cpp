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
#include <mrpt/math/CMatrixFixed.h>

using namespace mrpt::img;

// CCanvas is an abstract interface; CImage is the concrete implementation
// used to exercise its default-implemented drawing primitives.

TEST(CCanvas, Line)
{
  CImage img(20, 20, CH_RGB);
  img.filledRectangle({0, 0}, {19, 19}, TColor::black());

  img.line({0, 0}, {19, 19}, TColor::white());
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 255);
  EXPECT_EQ(img.at<uint8_t>(19, 19, 0), 255);
}

TEST(CCanvas, LineDegenerateOrOutOfBoundsIsNoOp)
{
  CImage img(10, 10, CH_RGB);
  img.filledRectangle({0, 0}, {9, 9}, TColor::black());

  // Zero-length line
  img.line({5, 5}, {5, 5}, TColor::white());
  // Both endpoints with negative X: nothing to draw
  img.line({-5, 0}, {-1, 0}, TColor::white());
  // Both endpoints with negative Y
  img.line({0, -5}, {0, -1}, TColor::white());
  // Both endpoints beyond width
  img.line({100, 0}, {200, 0}, TColor::white());
  // Both endpoints beyond height
  img.line({0, 100}, {0, 200}, TColor::white());

  // Image should remain untouched (all black)
  for (int y = 0; y < img.getHeight(); y++)
  {
    for (int x = 0; x < img.getWidth(); x++)
    {
      EXPECT_EQ(img.at<uint8_t>(x, y, 0), 0);
    }
  }
}

TEST(CCanvas, Rectangle)
{
  CImage img(20, 20, CH_RGB);
  img.filledRectangle({0, 0}, {19, 19}, TColor::black());
  img.rectangle({2, 2}, {10, 10}, TColor::white());

  EXPECT_EQ(img.at<uint8_t>(2, 2, 0), 255);
  EXPECT_EQ(img.at<uint8_t>(10, 2, 0), 255);
}

TEST(CCanvas, TriangleInferiorAndSuperior)
{
  CImage img(40, 40, CH_RGB);
  img.filledRectangle({0, 0}, {39, 39}, TColor::black());

  img.triangle({20, 20}, 5, TColor::white(), true);
  img.triangle({20, 20}, 5, TColor::white(), false);
  // Just check the call doesn't crash and touches some pixels
  SUCCEED();
}

TEST(CCanvas, FilledRectangleClipsToImage)
{
  CImage img(10, 10, CH_RGB);
  img.filledRectangle({0, 0}, {9, 9}, TColor::black());
  img.filledRectangle({-5, -5}, {5, 5}, TColor::white());

  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 255);
  EXPECT_EQ(img.at<uint8_t>(5, 5, 0), 255);
}

TEST(CCanvas, SelectTextFontUnknownWarnsAndKeepsPrevious)
{
  CImage img(10, 10, CH_GRAY);
  img.selectTextFont("this-font-does-not-exist");
  // Should not throw; falls back silently (with a stderr warning).
  SUCCEED();
}

TEST(CCanvas, SelectTextFontKnown)
{
  CImage img(10, 10, CH_GRAY);
  img.selectTextFont("6x13");
  img.selectTextFont("6x13B");
  img.selectTextFont("6x13O");
  img.selectTextFont("10x20");
  SUCCEED();
}

TEST(CCanvas, TextOutDefaultFontDrawsSomething)
{
  CImage img(80, 20, CH_GRAY);
  img.filledRectangle({0, 0}, {79, 19}, TColor::black());
  img.textOut({2, 2}, "Hi", TColor::white());

  bool anyWhite = false;
  for (int y = 0; y < img.getHeight() && !anyWhite; y++)
  {
    for (int x = 0; x < img.getWidth(); x++)
    {
      if (img.at<uint8_t>(x, y) == 255)
      {
        anyWhite = true;
        break;
      }
    }
  }
  EXPECT_TRUE(anyWhite);
}

TEST(CCanvas, TextOutUnicodeCharacterOutsideAnyBlockIsSkipped)
{
  CImage img(20, 20, CH_GRAY);
  // A UNICODE codepoint (encoded as UTF-8, within the uint16_t range used
  // internally) not present in the "9x15" font block ranges: this exercises
  // the "char not in font" skip path.
  img.textOut({0, 0}, "\xE2\x98\x83", TColor::white());  // U+2603 SNOWMAN
  SUCCEED();
}

TEST(CCanvas, DrawImageColorOntoColor)
{
  CImage dst(10, 10, CH_RGB);
  dst.filledRectangle({0, 0}, {9, 9}, TColor::black());

  CImage src(4, 4, CH_RGB);
  src.filledRectangle({0, 0}, {3, 3}, TColor::white());

  dst.drawImage({2, 2}, src);
  EXPECT_EQ(dst.at<uint8_t>(2, 2, 0), 255);
}

TEST(CCanvas, DrawImageGrayOntoColor)
{
  CImage dst(10, 10, CH_GRAY);
  dst.filledRectangle({0, 0}, {9, 9}, TColor::black());

  CImage src(4, 4, CH_GRAY);
  src.filledRectangle({0, 0}, {3, 3}, TColor::white());

  dst.drawImage({2, 2}, src);
  EXPECT_EQ(dst.at<uint8_t>(2, 2, 0), 255);
}

TEST(CCanvas, DrawMarkAllTypes)
{
  CImage img(30, 30, CH_RGB);
  img.filledRectangle({0, 0}, {29, 29}, TColor::black());

  img.drawMark({15, 15}, TColor::white(), '+', 5);
  img.drawMark({15, 15}, TColor::white(), 's', 5);
  img.drawMark({15, 15}, TColor::white(), 'x', 5);
  img.drawMark({15, 15}, TColor::white(), ':', 5);
  SUCCEED();
}

TEST(CCanvas, DrawMarkInvalidTypeThrows)
{
  CImage img(10, 10, CH_RGB);
  EXPECT_THROW(img.drawMark({5, 5}, TColor::white(), 'Q'), std::exception);
}

TEST(CCanvas, DrawCircle)
{
  CImage img(30, 30, CH_RGB);
  img.filledRectangle({0, 0}, {29, 29}, TColor::black());

  img.drawCircle({15, 15}, 10, TColor::white());
  // Negative radius is normalized (mirrored) internally.
  img.drawCircle({15, 15}, -10, TColor::white());
  // Zero radius: a degenerate 2-segment "circle"
  img.drawCircle({15, 15}, 0, TColor::white());
  SUCCEED();
}

TEST(CCanvas, EllipseGaussian)
{
  CImage img(40, 40, CH_RGB);
  img.filledRectangle({0, 0}, {39, 39}, TColor::black());

  mrpt::math::CMatrixFixed<double, 2, 2> cov;
  cov(0, 0) = 4.0;
  cov(1, 1) = 9.0;
  cov(0, 1) = cov(1, 0) = 0.5;

  img.ellipseGaussian(cov, 20.0, 20.0, 2.0, TColor::white());
  SUCCEED();
}
