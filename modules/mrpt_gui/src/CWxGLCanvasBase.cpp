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

#include <mrpt/core/lock_helper.h>
#include <mrpt/gui/CWxGLCanvasBase.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/gui/config.h>     // MRPT_HAS_WXWIDGETS
#include <mrpt/opengl/config.h>  // MRPT_HAS_OPENGL_GLUT
#include <mrpt/system/CTicTac.h>

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// Windows:
#include <windows.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif

#endif

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace std;

/*----------------------------------------------------------------
  Implementation of Test-GLCanvas
-----------------------------------------------------------------*/

std::unique_ptr<wxGLContext> CWxGLCanvasBase::m_gl_context;
std::recursive_mutex CWxGLCanvasBase::m_gl_context_mtx;

void CWxGLCanvasBase::OnWindowCreation(wxWindowCreateEvent& /*ev*/)
{
  auto lck = mrpt::lockHelper(m_gl_context_mtx);

  if (!m_gl_context)
  {
    m_gl_context = std::make_unique<wxGLContext>(this);
  }
}

void CWxGLCanvasBase::swapBuffers()
{
  auto lck = mrpt::lockHelper(m_gl_context_mtx);

  if (m_gl_context)
  {
    SetCurrent(*m_gl_context);
    SwapBuffers();
  }
}
void CWxGLCanvasBase::preRender() { OnPreRender(); }
void CWxGLCanvasBase::postRender() { OnPostRender(); }
void CWxGLCanvasBase::renderError(const string& err_msg) { OnRenderError(err_msg.c_str()); }

namespace
{
uint8_t wxModsToMrpt(const wxMouseEvent& e)
{
  uint8_t m = 0;
  if (e.ShiftDown())
  {
    m |= COrbitCameraController::ModShift;
  }
  if (e.ControlDown())
  {
    m |= COrbitCameraController::ModControl;
  }
  if (e.AltDown())
  {
    m |= COrbitCameraController::ModAlt;
  }
  return m;
}

uint8_t wxButtonDown(const wxMouseEvent& e)
{
  uint8_t m = 0;
  if (e.LeftDown())
  {
    m |= COrbitCameraController::ButtonLeft;
  }
  if (e.MiddleDown())
  {
    m |= COrbitCameraController::ButtonMiddle;
  }
  if (e.RightDown())
  {
    m |= COrbitCameraController::ButtonRight;
  }
  return m;
}
uint8_t wxButtonUp(const wxMouseEvent& e)
{
  uint8_t m = 0;
  if (e.LeftUp())
  {
    m |= COrbitCameraController::ButtonLeft;
  }
  if (e.MiddleUp())
  {
    m |= COrbitCameraController::ButtonMiddle;
  }
  if (e.RightUp())
  {
    m |= COrbitCameraController::ButtonRight;
  }
  return m;
}

}  // namespace

void CWxGLCanvasBase::OnMouseDown(wxMouseEvent& event)
{
  m_cameraCtrl.onMouseButton(event.GetX(), event.GetY(), wxButtonDown(event), true);
}

void CWxGLCanvasBase::OnMouseUp(wxMouseEvent& event)
{
  m_cameraCtrl.onMouseButton(event.GetX(), event.GetY(), wxButtonUp(event), false);
}

void CWxGLCanvasBase::OnMouseMove(wxMouseEvent& event)
{
  uint8_t buttons = 0;
  if (event.LeftIsDown())
  {
    buttons |= COrbitCameraController::ButtonLeft;
  }
  if (event.MiddleIsDown())
  {
    buttons |= COrbitCameraController::ButtonMiddle;
  }
  if (event.RightIsDown())
  {
    buttons |= COrbitCameraController::ButtonRight;
  }

  if (buttons != 0)
  {
    m_cameraCtrl.onMouseMove(event.GetX(), event.GetY(), buttons, wxModsToMrpt(event));
    Refresh(false);
    Update();
  }
  this->SetFocus();
}

void CWxGLCanvasBase::OnMouseWheel(wxMouseEvent& event)
{
  // wxWidgets gives rotation in multiples of WHEEL_DELTA (120).
  // Normalise to ±1 units like the other backends.
  const float delta =
      static_cast<float>(event.GetWheelRotation()) / static_cast<float>(event.GetWheelDelta());
  m_cameraCtrl.onScroll(delta, wxModsToMrpt(event));
  Refresh(false);
  Update();
  this->SetFocus();
}

// clang-format off
namespace {
int WX_GL_ATTR_LIST[] = {
	WX_GL_DOUBLEBUFFER,    WX_GL_RGBA,
	WX_GL_DEPTH_SIZE,     24,
#if wxCHECK_VERSION(3, 0, 3)
	WX_GL_MAJOR_VERSION,  3,
	WX_GL_MINOR_VERSION,  1,
    WX_GL_CORE_PROFILE, // do not allow using opengl 1.x deprecated stuff
#endif
	0
};
}
// clang-format on

CWxGLCanvasBase::CWxGLCanvasBase(
    wxWindow* parent,
    wxWindowID id,
    const wxPoint& pos,
    const wxSize& size,
    long style,
    const wxString& name) :
    wxGLCanvas(parent, id, WX_GL_ATTR_LIST, pos, size, style | wxFULL_REPAINT_ON_RESIZE, name)
{
  this->Bind(wxEVT_LEFT_DOWN, &CWxGLCanvasBase::OnMouseDown, this);
  this->Bind(wxEVT_MIDDLE_DOWN, &CWxGLCanvasBase::OnMouseDown, this);
  this->Bind(wxEVT_RIGHT_DOWN, &CWxGLCanvasBase::OnMouseDown, this);

  this->Bind(wxEVT_LEFT_UP, &CWxGLCanvasBase::OnMouseUp, this);
  this->Bind(wxEVT_MIDDLE_UP, &CWxGLCanvasBase::OnMouseUp, this);
  this->Bind(wxEVT_RIGHT_UP, &CWxGLCanvasBase::OnMouseUp, this);

  this->Bind(wxEVT_MOTION, &CWxGLCanvasBase::OnMouseMove, this);
  this->Bind(wxEVT_MOUSEWHEEL, &CWxGLCanvasBase::OnMouseWheel, this);
  this->Bind(wxEVT_CHAR, &CWxGLCanvasBase::OnChar, this);
  this->Bind(wxEVT_CHAR_HOOK, &CWxGLCanvasBase::OnChar, this);
  this->Bind(wxEVT_CREATE, &CWxGLCanvasBase::OnWindowCreation, this);

  this->Bind(wxEVT_SIZE, &CWxGLCanvasBase::OnSize, this);
  this->Bind(wxEVT_PAINT, &CWxGLCanvasBase::OnPaint, this);
  this->Bind(wxEVT_ERASE_BACKGROUND, &CWxGLCanvasBase::OnEraseBackground, this);
  this->Bind(wxEVT_ENTER_WINDOW, &CWxGLCanvasBase::OnEnterWindow, this);

// JL: There seems to be a problem in MSW we don't receive this event, but
//      in GTK we do and at the right moment to avoid an X server crash.
#ifdef _WIN32
  wxWindowCreateEvent dum;
  OnWindowCreation(dum);
#endif
}

void CWxGLCanvasBase::OnChar(wxKeyEvent& event) { OnCharCustom(event); }
void CWxGLCanvasBase::Render()
{
  wxPaintDC dc(this);

  auto lck = mrpt::lockHelper(m_gl_context_mtx);

  if (!m_gl_context)
  { /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" <<
     "\n";*/
    return;
  }

  SetCurrent(*m_gl_context);

  // Init OpenGL once, but after SetCurrent
  if (!m_init)
  {
    InitGL();
    m_init = true;
  }

  lck.unlock();

  const auto sz = mrpt::gui::GetScaledClientSize(this);
  double At = renderCanvas(sz.GetWidth(), sz.GetHeight());

  OnPostRenderSwapBuffers(At, dc);
}

void CWxGLCanvasBase::OnEnterWindow(wxMouseEvent& WXUNUSED(event)) { SetFocus(); }

void CWxGLCanvasBase::OnPaint(wxPaintEvent& WXUNUSED(event)) { Render(); }
void CWxGLCanvasBase::OnSize(wxSizeEvent& event)
{
  if (!m_parent->IsShown())
  {
    return;
  }

  // set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
  const auto sz = mrpt::gui::GetScaledClientSize(this);

  if (this->IsShownOnScreen())
  {
    if (!m_gl_context)
    { /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" <<
       "\n";*/
      return;
    }
    {
      auto lck = mrpt::lockHelper(m_gl_context_mtx);
      SetCurrent(*m_gl_context);
    }

    resizeViewport(sz.GetWidth(), sz.GetHeight());
  }
}

void CWxGLCanvasBase::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
  // Do nothing, to avoid flashing.
}

void CWxGLCanvasBase::InitGL()
{
  auto lck = mrpt::lockHelper(m_gl_context_mtx);

  if (!m_gl_context)
  { /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" <<
     "\n";*/
    return;
  }

  SetCurrent(*m_gl_context);

  static bool GLUT_INIT_DONE = false;

  if (!GLUT_INIT_DONE)
  {
    GLUT_INIT_DONE = true;

    int argc = 1;
    char* argv[1] = {nullptr};
    glutInit(&argc, argv);
  }
}

void CWxGLCanvasBase::setCameraPose(const mrpt::poses::CPose3D& camPose)
{
  THROW_EXCEPTION("todo");
  // Convert pose to orbit parameters and feed into the controller
  // (implementation depends on CCamera::setFrom(CPose3D), TBD separately)
}

#endif  // MRPT_HAS_WXWIDGETS
