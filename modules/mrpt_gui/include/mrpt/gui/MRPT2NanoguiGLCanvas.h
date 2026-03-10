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

#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/viz/COrbitCameraController.h>

#include <memory>
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

  void drawGL() override;

  /** The scene to render in this control.
   * \note Users must lock the mutex scene_mtx while modifying this variable.
   */
  mrpt::viz::Scene::Ptr scene;
  std::mutex scene_mtx;

  mrpt::viz::COrbitCameraController& camera() { return m_cameraCtrl; }
  const mrpt::viz::COrbitCameraController& camera() const { return m_cameraCtrl; }

 protected:
  /** @name Internal virtual functions to handle GUI events
   * @{ */
  bool mouseMotionEvent(
      const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
  bool mouseButtonEvent(const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
  bool scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
  /** @} */

  mrpt::viz::COrbitCameraController m_cameraCtrl;
  uint8_t m_lastModifiers = 0;  // needed for scrollEvent (no modifiers arg in nanogui)

  std::unique_ptr<mrpt::opengl::CompiledScene> m_compiledScene;
  std::weak_ptr<mrpt::viz::Scene> m_lastCompiledScenePtr;
};

}  // namespace mrpt::gui

#endif  // MRPT_HAS_NANOGUI
