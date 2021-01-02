/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef MYGLCANVAS_H
#define MYGLCANVAS_H

#include <mrpt/gui/CWxGLCanvasBase.h>

using namespace mrpt::gui;

class CMyGLCanvas : public CWxGLCanvasBase
{
   public:
	CMyGLCanvas(
		wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize, long style = 0,
		const wxString& name = _T("CMyGLCanvas"));

	~CMyGLCanvas() override;

	void OnPreRender() override;
	void OnPostRender() override;
	void OnPostRenderSwapBuffers(double At, wxPaintDC& dc) override;
	void OnRenderError(const wxString& str) override;
};

#endif  // MYGLCANVAS_H
