/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef MYGLCANVAS_H
#define MYGLCANVAS_H

#include <mrpt/gui/CWxGLCanvasBase.h>
#include <mrpt/system/datetime.h>

// This is funny... conflicts with X headers.
#undef Button1
#undef Button2
#undef Button3
// To avoid conflicts between Eigen & X11 headers
#ifdef Success
#undef Success
#endif

using namespace mrpt::gui;

// Allow RawlogViewer to build without 3D:
#define KINECTSTEREO_HAS_3D (wxUSE_GLCANVAS && MRPT_HAS_OPENGL_GLUT)

#if KINECTSTEREO_HAS_3D

class CMyGLCanvas : public CWxGLCanvasBase
{
 public:
  CMyGLCanvas(
      wxWindow* parent,
      wxWindowID id = wxID_ANY,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize,
      long style = 0,
      const wxString& name = _T("CMyGLCanvas"));

  ~CMyGLCanvas() override;

  void OnPreRender() override;
  void OnPostRender() override;
  void OnPostRenderSwapBuffers(double At, wxPaintDC& dc) override;
  void OnRenderError(const wxString& str) override;
};

#else

// Dummy class
class CMyGLCanvas : public wxPanel
{
 public:
  CMyGLCanvas(
      wxWindow* parent,
      wxWindowID id = wxID_ANY,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize,
      long style = 0,
      const wxString& name = _T("CMyGLCanvas"))
  {
  }
};

#endif  // Have glut

#endif  // MYGLCANVAS_H
