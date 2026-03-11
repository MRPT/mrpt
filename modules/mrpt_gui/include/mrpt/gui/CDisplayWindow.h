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
#pragma once

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/system/os.h>

#include <vector>

namespace mrpt
{
/** Classes for creating GUI windows for 2D and 3D visualization.   \ingroup
 * mrpt_gui_grp */
namespace gui
{
/** This class creates a window as a graphical user interface (GUI) for
 * displaying images to the user.
 *
 *  For a list of supported events with the observer/observable pattern, see the
 * discussion in mrpt::gui::CBaseGUIWindow.
 *
 * ![mrpt::gui::CDisplayWindow screenshot](preview_CDisplayWindow.jpg)
 *
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindow : public mrpt::gui::CBaseGUIWindow
{
 public:
  using Ptr = std::shared_ptr<CDisplayWindow>;
  using ConstPtr = std::shared_ptr<const CDisplayWindow>;

 protected:
  /** Enables or disables the visualization of cursor coordinates on the
   * window caption.
   */
  bool m_enableCursorCoordinates{true};

 public:
  /** Constructor
   */
  CDisplayWindow(
      const std::string& windowCaption = std::string(),
      unsigned int initWidth = 400,
      unsigned int initHeight = 400);

  /** Class factory returning a smart pointer, equivalent to
   * `std::make_shared<>(...)` */
  static CDisplayWindow::Ptr Create(
      const std::string& windowCaption,
      unsigned int initWidth = 400,
      unsigned int initHeight = 400);

  /** Destructor
   */
  ~CDisplayWindow() override;

  std::optional<mrpt::img::TPixelCoord> getLastMousePosition() const override;

  /** Set cursor style to default (cursorIsCross=false) or to a cross
   * (cursorIsCross=true) */
  void setCursorCross(bool cursorIsCross) override;

  /** Show a given color or grayscale image on the window.
   *  It adapts the size of the window to that of the image.
   */
  void showImage(const mrpt::img::CImage& img);

  /** Plots a graph in MATLAB-like style.
   */
  void plot(const mrpt::math::CVectorFloat& x, const mrpt::math::CVectorFloat& y);

  /** Plots a graph in MATLAB-like style.
   */
  void plot(const mrpt::math::CVectorFloat& y);

  /** Resizes the window, stretching the image to fit into the display area.
   */
  void resize(unsigned int width, unsigned int height) override;

  /** Changes the position of the window on the screen.
   */
  void setPos(int x, int y) override;

  /** Enables or disables the visualization of cursor coordinates on the
   * window caption (default = enabled).
   */
  void enableCursorCoordinatesVisualization(bool enable) { m_enableCursorCoordinates = enable; }

  /** Changes the window title text.
   */
  void setWindowTitle(const std::string& str) override;

};  // End of class def.

}  // namespace gui

}  // namespace mrpt
