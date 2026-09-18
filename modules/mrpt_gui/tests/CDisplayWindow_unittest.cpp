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

/** Unit tests for the 2D image window and the plots window. They need a window
 *  server; CI provides a virtual one (`xvfb-run`), and the tests self-skip
 *  when there is none.
 */

#include <gtest/gtest.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>

#include <thread>
#include <vector>

#include "gui_test_common.h"

namespace
{
mrpt::img::CImage makeTestImage(unsigned w, unsigned h)
{
  mrpt::img::CImage img(w, h, mrpt::img::CH_RGB);
  img.filledRectangle(
      mrpt::img::TPixelCoord(0, 0),
      mrpt::img::TPixelCoord(static_cast<int>(w) - 1, static_cast<int>(h) - 1),
      mrpt::img::TColor(20, 120, 200));
  return img;
}

/** Give the GUI thread a chance to process the queued requests. */
void pumpGui() { std::this_thread::sleep_for(std::chrono::milliseconds(150)); }
}  // namespace

TEST(CDisplayWindow, create_show_image_and_close)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow win("unit test 2D", 320, 240);
  EXPECT_TRUE(win.isOpen());

  win.showImage(makeTestImage(320, 240));
  pumpGui();
  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow, window_geometry_and_title)
{
  SKIP_IF_NO_GUI();

  auto win = mrpt::gui::CDisplayWindow::Create("unit test resize", 200, 150);
  ASSERT_TRUE(win->isOpen());

  win->setWindowTitle("renamed");
  win->resize(320, 240);
  win->setPos(50, 60);
  win->setCursorCross(true);
  win->setCursorCross(false);
  win->enableCursorCoordinatesVisualization(false);
  pumpGui();

  EXPECT_TRUE(win->isOpen());
}

TEST(CDisplayWindow, plot_helpers)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow win("unit test plot", 320, 240);

  mrpt::math::CVectorFloat x(5), y(5);
  for (int i = 0; i < 5; i++)
  {
    x[i] = static_cast<float>(i);
    y[i] = static_cast<float>(i * i);
  }
  win.plot(x, y);
  pumpGui();
  win.plot(y);
  pumpGui();

  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindow, key_state_starts_clear)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindow win("unit test keys", 160, 120);
  EXPECT_FALSE(win.keyHit());
  EXPECT_EQ(win.getPushedKey(), 0);

  win.clearKeyHitFlag();
  EXPECT_FALSE(win.keyHit());

  // No mouse has entered the window yet, but the query must not throw:
  EXPECT_NO_THROW((void)win.getLastMousePosition());
}

TEST(CDisplayWindowPlots, plot_curves_and_shapes)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindowPlots win("unit test plots", 320, 240);
  ASSERT_TRUE(win.isOpen());

  std::vector<double> x{0.0, 1.0, 2.0, 3.0};
  std::vector<double> y{0.0, 1.0, 4.0, 9.0};

  win.plot(x, y, "b-", "parabola");
  win.hold_on();
  win.plot(x, x, "r.", "identity");
  win.hold_off();
  win.axis(-1, 4, -1, 10);
  win.axis_equal(true);
  win.axis_fit();
  win.enableMousePanZoom(true);
  win.setWindowTitle("plots renamed");
  win.resize(400, 300);
  pumpGui();

  EXPECT_TRUE(win.isOpen());

  win.clf();
  pumpGui();
  EXPECT_TRUE(win.isOpen());
}

TEST(CDisplayWindowPlots, plot_an_ellipse_and_an_image)
{
  SKIP_IF_NO_GUI();

  mrpt::gui::CDisplayWindowPlots win("unit test plots 2", 320, 240);

  mrpt::math::CMatrixFixed<double, 2, 2> cov;
  cov.setIdentity();
  win.plotEllipse(1.0, 2.0, cov, 2.0f, "b-", "ellipse");

  mrpt::math::CMatrixDynamic<double> covDyn(2, 2);
  covDyn.setIdentity();
  win.plotEllipse(0.0, 0.0, covDyn, 1.0f, "r-", "ellipseDyn", true /*showName*/);

  win.image(makeTestImage(64, 48), 0.0f, 0.0f, 1.0f, 1.0f, "img");
  win.axis_fit();
  pumpGui();

  EXPECT_TRUE(win.isOpen());
}
