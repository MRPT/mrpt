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
#include <mrpt/imgui/CImGuiImageView.h>

using namespace mrpt::imgui;

TEST(ImageViewTransform, FitCentersAndKeepsAspect)
{
  ImageViewTransform tf;
  tf.fit(200, 100, 400, 400);  // limited by width: zoom 2
  EXPECT_FLOAT_EQ(tf.zoom, 2.0f);
  const auto [x0, y0] = tf.imageToWidget(0, 0);
  const auto [x1, y1] = tf.imageToWidget(200, 100);
  EXPECT_FLOAT_EQ(x0, 0.0f);
  EXPECT_FLOAT_EQ(x1, 400.0f);
  EXPECT_FLOAT_EQ(y0, 100.0f);  // centered vertically
  EXPECT_FLOAT_EQ(y1, 300.0f);
}

TEST(ImageViewTransform, ZoomAtKeepsPointUnderCursor)
{
  ImageViewTransform tf;
  tf.fit(100, 100, 100, 100);
  const auto [u0, v0] = tf.widgetToImage(30, 70);
  tf.zoomAt(30, 70, 3.0f);
  EXPECT_FLOAT_EQ(tf.zoom, 3.0f);
  const auto [u1, v1] = tf.widgetToImage(30, 70);
  EXPECT_NEAR(u0, u1, 1e-4);
  EXPECT_NEAR(v0, v1, 1e-4);

  tf.zoomAt(0, 0, 1e6f);
  EXPECT_FLOAT_EQ(tf.zoom, tf.maxZoom);
}

TEST(ImageViewTransform, Pan)
{
  ImageViewTransform tf;
  tf.zoom = 2;
  tf.pan(10, -4);
  EXPECT_FLOAT_EQ(tf.offsetX, -5.0f);
  EXPECT_FLOAT_EQ(tf.offsetY, 2.0f);
}

TEST(CImGuiImageView, toRGBA8Gray)
{
  mrpt::img::CImage im(3, 2, mrpt::img::CH_GRAY);
  im.setPixel({2, 1}, mrpt::img::TColor(77, 77, 77));
  const auto buf = CImGuiImageView::toRGBA8(im);
  ASSERT_EQ(buf.size(), 3u * 2u * 4u);
  const size_t i = (1 * 3 + 2) * 4;
  EXPECT_EQ(buf[i + 0], 77);
  EXPECT_EQ(buf[i + 1], 77);
  EXPECT_EQ(buf[i + 2], 77);
  EXPECT_EQ(buf[i + 3], 255);
}

TEST(CImGuiImageView, toRGBA8ColorChannelOrder)
{
  mrpt::img::CImage im(2, 2, mrpt::img::CH_RGB);
  im.setPixel({1, 0}, mrpt::img::TColor(10, 20, 30));
  const auto buf = CImGuiImageView::toRGBA8(im);
  const size_t i = 1 * 4;
  EXPECT_EQ(buf[i + 0], 10);  // R, whatever the internal channel order
  EXPECT_EQ(buf[i + 1], 20);
  EXPECT_EQ(buf[i + 2], 30);
  EXPECT_EQ(buf[i + 3], 255);

  const auto info = CImGuiImageView::pixelInfo(im, 1, 0);
  EXPECT_EQ(info, "(1, 0): R=10 G=20 B=30");
  EXPECT_TRUE(CImGuiImageView::pixelInfo(im, 2, 0).empty());
  EXPECT_TRUE(CImGuiImageView::pixelInfo(im, -1, 0).empty());
}

TEST(CImGuiImageView, toRGBA8Depth16Normalized)
{
  mrpt::img::CImage im;
  im.resize(2, 1, mrpt::img::CH_GRAY, mrpt::img::PixelDepth::D16U);
  im.ptrLine<uint16_t>(0)[0] = 1000;
  im.ptrLine<uint16_t>(0)[1] = 3000;
  const auto buf = CImGuiImageView::toRGBA8(im);
  ASSERT_EQ(buf.size(), 8u);
  EXPECT_EQ(buf[0], 0);
  EXPECT_EQ(buf[4], 255);
  EXPECT_EQ(CImGuiImageView::pixelInfo(im, 1, 0), "(1, 0): 3000");
}

TEST(CImGuiImageView, EmptyImage)
{
  CImGuiImageView view;
  EXPECT_FALSE(view.hasImage());
  EXPECT_TRUE(CImGuiImageView::toRGBA8(mrpt::img::CImage()).empty());
  mrpt::img::CImage im(4, 4, mrpt::img::CH_RGB);
  view.setImage(im);
  EXPECT_TRUE(view.hasImage());
  view.clear();
  EXPECT_FALSE(view.hasImage());
}
