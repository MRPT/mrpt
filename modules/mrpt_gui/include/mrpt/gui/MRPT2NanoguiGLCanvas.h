/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>

#include <mutex>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/gui/config.h>
#if MRPT_HAS_NANOGUI

#ifdef None  // X11 headers conflict...
#undef None
#endif
#include <nanogui/nanogui.h>

namespace mrpt::gui
{
/** An extension of nanogui::GLCanvas to render MRPT OpenGL scenes.
 *
 * Directly access `scene` (locking its mutex `scene_mtx` first) to update the
 * scene to be rendered.
 *
 * \sa CDisplayWindowGUI
 * \ingroup mrpt_gui_grp
 */
class MRPT2NanoguiGLCanvas : public nanogui::GLCanvas
{
 public:
  MRPT2NanoguiGLCanvas(nanogui::Widget* parent);
  ~MRPT2NanoguiGLCanvas();

  virtual void drawGL() override;

  /** The scene to render in this control.
   * \note Users must lock the mutex scene_mtx while modifying this variable.
   */
  mrpt::viz::Scene::Ptr scene;
  std::mutex scene_mtx;

  CGlCanvasBase& camera() { return m_headless_canvas; }
  const CGlCanvasBase& camera() const { return m_headless_canvas; }

 protected:
  /** @name Internal virtual functions to handle GUI events
   * @{ */
  virtual bool mouseMotionEvent(
      const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
  virtual bool mouseButtonEvent(
      const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
  virtual bool scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
  /** @} */

  /** Used to keep track of mouse events on the camera */
  internal::NanoGUICanvasHeadless m_headless_canvas;
};

}  // namespace mrpt::gui

#endif  // MRPT_HAS_NANOGUI
