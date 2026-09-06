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

/** Unit tests for the CImage <-> wxImage conversion helpers. wxWidgets needs
 *  its image handlers (and hence the app) initialized, so these run under the
 *  same display gate as the window tests.
 */

#include <gtest/gtest.h>
#include <mrpt/gui/config.h>

#include "gui_test_common.h"

#if MRPT_HAS_WXWIDGETS

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/img/CImage.h>

#include <memory>

namespace
{
/** A gradient image, so that a conversion that dropped or swapped channels
 *  cannot go unnoticed. Pixels are written channel by channel: `at<TColor>()`
 *  would reinterpret 4 bytes over a 3-byte RGB pixel. */
mrpt::img::CImage makeGradient(unsigned w, unsigned h)
{
  mrpt::img::CImage img(w, h, mrpt::img::CH_RGB);
  for (int y = 0; y < static_cast<int>(h); y++)
  {
    for (int x = 0; x < static_cast<int>(w); x++)
    {
      img.at<uint8_t>(x, y, 0) = static_cast<uint8_t>(x * 4);
      img.at<uint8_t>(x, y, 1) = static_cast<uint8_t>(y * 4);
      img.at<uint8_t>(x, y, 2) = 128;
    }
  }
  return img;
}

/** wxWidgets must be up and running before any wxImage is created; opening a
 *  window is the supported way to start MRPT's wx subsystem. */
struct WxFixture
{
  std::unique_ptr<mrpt::gui::CDisplayWindow> win;
  WxFixture() { win = std::make_unique<mrpt::gui::CDisplayWindow>("wx init", 64, 64); }
};
}  // namespace

TEST(WxUtils, mrpt_image_to_wx_image_and_back)
{
  SKIP_IF_NO_GUI();
  WxFixture fx;

  const unsigned W = 32, H = 16;
  const auto src = makeGradient(W, H);

  std::unique_ptr<wxImage> wxImg(mrpt::gui::MRPTImage2wxImage(src));
  ASSERT_NE(wxImg, nullptr);
  ASSERT_TRUE(wxImg->IsOk());
  EXPECT_EQ(static_cast<unsigned>(wxImg->GetWidth()), W);
  EXPECT_EQ(static_cast<unsigned>(wxImg->GetHeight()), H);

  const auto back = mrpt::gui::wxImage2MRPTImagePtr(*wxImg);
  ASSERT_NE(back, nullptr);
  EXPECT_EQ(back->getWidth(), W);
  EXPECT_EQ(back->getHeight(), H);

  // The round trip must preserve every pixel, in the same channel order:
  for (int y = 0; y < static_cast<int>(H); y++)
  {
    for (int x = 0; x < static_cast<int>(W); x++)
    {
      for (int8_t ch = 0; ch < 3; ch++)
      {
        ASSERT_EQ(src.at<uint8_t>(x, y, ch), back->at<uint8_t>(x, y, ch))
            << "at (" << x << "," << y << ") channel " << static_cast<int>(ch);
      }
    }
  }
}

TEST(WxUtils, raw_pointer_conversion_overload)
{
  SKIP_IF_NO_GUI();
  WxFixture fx;

  const auto src = makeGradient(8, 8);
  std::unique_ptr<wxImage> wxImg(mrpt::gui::MRPTImage2wxImage(src));
  ASSERT_NE(wxImg, nullptr);

  std::unique_ptr<mrpt::img::CImage> back(mrpt::gui::wxImage2MRPTImage(*wxImg));
  ASSERT_NE(back, nullptr);
  EXPECT_EQ(back->getWidth(), 8U);
  EXPECT_EQ(back->getHeight(), 8U);
}

TEST(WxUtils, mrpt_image_to_wx_bitmap)
{
  SKIP_IF_NO_GUI();
  WxFixture fx;

  const auto src = makeGradient(20, 10);
  std::unique_ptr<wxBitmap> bmp(mrpt::gui::MRPTImage2wxBitmap(src));
  ASSERT_NE(bmp, nullptr);
  EXPECT_TRUE(bmp->IsOk());
  EXPECT_EQ(bmp->GetWidth(), 20);
  EXPECT_EQ(bmp->GetHeight(), 10);
}

TEST(WxUtils, grayscale_images_are_expanded_to_rgb)
{
  SKIP_IF_NO_GUI();
  WxFixture fx;

  mrpt::img::CImage gray(12, 6, mrpt::img::CH_GRAY);
  for (int y = 0; y < 6; y++)
  {
    for (int x = 0; x < 12; x++)
    {
      gray.at<uint8_t>(x, y) = static_cast<uint8_t>(x * 20);
    }
  }

  std::unique_ptr<wxImage> wxImg(mrpt::gui::MRPTImage2wxImage(gray));
  ASSERT_NE(wxImg, nullptr);
  ASSERT_TRUE(wxImg->IsOk());

  const auto back = mrpt::gui::wxImage2MRPTImagePtr(*wxImg);
  ASSERT_NE(back, nullptr);
  ASSERT_TRUE(back->isColor());
  for (int y = 0; y < 6; y++)
  {
    for (int x = 0; x < 12; x++)
    {
      const auto v = gray.at<uint8_t>(x, y);
      for (int8_t ch = 0; ch < 3; ch++)
      {
        ASSERT_EQ(back->at<uint8_t>(x, y, ch), v) << "at (" << x << "," << y << ")";
      }
    }
  }
}

#endif  // MRPT_HAS_WXWIDGETS
