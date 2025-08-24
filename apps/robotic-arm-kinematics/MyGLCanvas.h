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

#ifndef MYGLCANVAS_H
#define MYGLCANVAS_H

#include <mrpt/gui/CWxGLCanvasBase.h>
#include <wx/panel.h>

using namespace mrpt::gui;

// Allow RawlogViewer to build without 3D:
#define APP_HAS_3D (wxUSE_GLCANVAS && MRPT_HAS_OPENGL_GLUT)

#if APP_HAS_3D

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

#endif

#endif  // MYGLCANVAS_H
