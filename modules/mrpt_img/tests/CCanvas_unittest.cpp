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

#include <vector>

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

namespace
{
/** A minimal CCanvas implementation that does NOT override drawImage(), so the
 * generic per-pixel base implementation is the one under test (CImage
 * overrides it with a faster, memcpy-based one).
 */
class SimpleCanvas : public mrpt::img::CCanvas
{
 public:
  SimpleCanvas(int32_t w, int32_t h) : m_w(w), m_h(h), m_pixels(static_cast<size_t>(w * h)) {}

  void setPixel(const TPixelCoord& pt, const mrpt::img::TColor& color) override
  {
    if (pt.x < 0 || pt.y < 0 || pt.x >= m_w || pt.y >= m_h)
    {
      return;
    }
    m_pixels[static_cast<size_t>(pt.y * m_w + pt.x)] = color;
  }
  [[nodiscard]] int32_t getWidth() const override { return m_w; }
  [[nodiscard]] int32_t getHeight() const override { return m_h; }

  [[nodiscard]] const mrpt::img::TColor& at(int32_t x, int32_t y) const
  {
    return m_pixels[static_cast<size_t>(y * m_w + x)];
  }

 private:
  int32_t m_w;
  int32_t m_h;
  std::vector<mrpt::img::TColor> m_pixels;
};
}  // namespace

TEST(CCanvas, BaseDrawImageColorSource)
{
  SimpleCanvas canvas(8, 8);

  CImage src(2, 2, CH_RGB);
  src.setPixel({0, 0}, TColor(10, 20, 30));
  src.setPixel({1, 0}, TColor(40, 50, 60));
  src.setPixel({0, 1}, TColor(70, 80, 90));
  src.setPixel({1, 1}, TColor(100, 110, 120));

  canvas.drawImage({3, 4}, src);

  EXPECT_EQ(canvas.at(3, 4).R, 10);
  EXPECT_EQ(canvas.at(3, 4).G, 20);
  EXPECT_EQ(canvas.at(3, 4).B, 30);
  EXPECT_EQ(canvas.at(4, 5).R, 100);
  EXPECT_EQ(canvas.at(4, 5).B, 120);
  // Untouched pixel:
  EXPECT_EQ(canvas.at(0, 0).R, 0);
}

TEST(CCanvas, BaseDrawImageGrayscaleSourceIsReplicatedToRGB)
{
  SimpleCanvas canvas(8, 8);

  CImage src(2, 1, CH_GRAY);
  src.setPixelGray({0, 0}, 77);
  src.setPixelGray({1, 0}, 200);

  canvas.drawImage({1, 1}, src);

  EXPECT_EQ(canvas.at(1, 1).R, 77);
  EXPECT_EQ(canvas.at(1, 1).G, 77);
  EXPECT_EQ(canvas.at(1, 1).B, 77);
  EXPECT_EQ(canvas.at(2, 1).R, 200);
}

TEST(CCanvas, BaseDrawImageClipsOutsideTarget)
{
  SimpleCanvas canvas(4, 4);
  CImage src(3, 3, CH_RGB);
  src.filledRectangle({0, 0}, {2, 2}, TColor::white());

  // Partially off both edges: setPixel() silently drops out-of-range writes.
  canvas.drawImage({-1, -1}, src);
  EXPECT_EQ(canvas.at(0, 0).R, 255);

  canvas.drawImage({3, 3}, src);
  EXPECT_EQ(canvas.at(3, 3).R, 255);
}
