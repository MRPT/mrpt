/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MYGLCANVAS_H
#define MYGLCANVAS_H

#include <mrpt/gui/CMyGLCanvasBase.h>


// This is funny... conflicts with X headers.
#undef Button1
#undef Button2
#undef Button3
// To avoid conflicts between Eigen & X11 headers
#ifdef Success 
#	undef Success 
#endif

using namespace mrpt::gui;

// Allow RawlogViewer to build without 3D:
#define KINECTSTEREO_HAS_3D (wxUSE_GLCANVAS && MRPT_HAS_OPENGL_GLUT)

#if KINECTSTEREO_HAS_3D

class CMyGLCanvas : public CMyGLCanvasBase
{
public:
    CMyGLCanvas( wxWindow *parent, wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0, const wxString& name = _T("CMyGLCanvas") );

	virtual ~CMyGLCanvas();

	void OnPreRender();
	void OnPostRender();
	void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);
	void OnRenderError( const wxString &str );

	mrpt::system::TTimeStamp last_timestamp;

};

#else

// Dummy class
class CMyGLCanvas : public wxPanel
{
public:
    CMyGLCanvas( wxWindow *parent, wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0, const wxString& name = _T("CMyGLCanvas") )
	{
	}
};

#endif  // Have glut


#endif // MYGLCANVAS_H

