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

#include "MyGLCanvas.h"

using namespace std;

#if KINECTSTEREO_HAS_3D

CMyGLCanvas::CMyGLCanvas(
    wxWindow* parent,
    wxWindowID id,
    const wxPoint& pos,
    const wxSize& size,
    long style,
    const wxString& name) :
    CWxGLCanvasBase(parent, id, pos, size, style, name)
{
  setCameraPointing(0.0f, 0.0f, 0.0f);
  setZoomDistance(20.0f);
  setElevationDegrees(45.0f);
  setAzimuthDegrees(135.0f);
  setCameraProjective(true);
}

CMyGLCanvas::~CMyGLCanvas() = default;
void CMyGLCanvas::OnRenderError(const wxString& str) {}
void CMyGLCanvas::OnPreRender() {}
void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC& dc) {}
void CMyGLCanvas::OnPostRender() {}

#endif
