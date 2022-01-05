/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "MyGLCanvas.h"

using namespace std;

#if APP_HAS_3D

CMyGLCanvas::CMyGLCanvas(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long style, const wxString& name)
	: CWxGLCanvasBase(parent, id, pos, size, style, name)
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
void CMyGLCanvas::OnPostRender()
{
	// Show credits on the screen? renderTextBitmap(20, 20, "" );
}

#endif
