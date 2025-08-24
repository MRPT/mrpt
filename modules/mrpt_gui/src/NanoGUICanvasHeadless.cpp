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

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

using namespace mrpt::gui::internal;

void NanoGUICanvasHeadless::mouseMotionEvent(
    const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers)
{
  m_lastModifiers = modifiers;

  const bool leftIsDown = button & (1 << GLFW_MOUSE_BUTTON_LEFT);
  const bool middleIsDown = button & (1 << GLFW_MOUSE_BUTTON_MIDDLE);
  const bool rightIsDown = button & (1 << GLFW_MOUSE_BUTTON_RIGHT);

  const int X = p.x();
  const int Y = p.y();
  updateLastPos(X, Y);

  if (leftIsDown || rightIsDown || middleIsDown)
  {
    // Proxy variables to cache the changes:
    CGlCanvasBase::CamaraParams params = cameraParams();

    if (leftIsDown)
    {
      if (modifiers & GLFW_MOD_SHIFT)
        updateZoom(params, X, Y);
      else if (modifiers & GLFW_MOD_CONTROL)
        updateRotate(params, X, Y);
      else if (modifiers & GLFW_MOD_ALT)
        updateRoll(params, X, Y, 0.30f);
      else
        updateOrbitCamera(params, X, Y);
    }
    else
      updatePan(params, X, Y);

    setMousePos(X, Y);
    setCameraParams(params);

    // Refresh window: done automatically by parent
  }
}

void NanoGUICanvasHeadless::mouseButtonEvent(
    const nanogui::Vector2i& p, int button, bool down, int modifiers)
{
  m_lastModifiers = modifiers;

  setMousePos(p.x(), p.y());
  setMouseClicked(down);
}
void NanoGUICanvasHeadless::scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel)
{
  CamaraParams params = cameraParams();

  if (!(m_lastModifiers & GLFW_MOD_SHIFT))
  {
    // regular zoom:
    updateZoom(params, 125 * rel.y());
  }
  else
  {
    // Move vertically +-Z:
    params.cameraPointingZ += 125 * rel.y() * params.cameraZoomDistance * 1e-4;
  }
  setCameraParams(params);
}

#endif  // MRPT_HAS_NANOGUI
