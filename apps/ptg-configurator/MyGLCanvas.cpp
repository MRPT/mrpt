/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "MyGLCanvas.h"

using namespace std;

#if APP_HAS_3D

CMyGLCanvas::CMyGLCanvas( wxWindow *parent, wxWindowID id,
                 const wxPoint& pos, const wxSize& size,
                 long style, const wxString& name )
		: CMyGLCanvasBase(parent,id,pos,size,style,name)
{
	cameraPointingX = 0;
	cameraPointingY = 0;
	cameraPointingZ = 0;
	cameraZoomDistance = 20;
	cameraElevationDeg = 45;
	cameraAzimuthDeg   = 135;
	cameraIsProjective = true;
}

CMyGLCanvas::~CMyGLCanvas()
{
}

void CMyGLCanvas::OnRenderError( const wxString &str )
{
}

void CMyGLCanvas::OnPreRender()
{
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
}

void CMyGLCanvas::OnPostRender()
{
	// If any, draw the 2D text messages:
	int w,h;
	this->GetSize(&w,&h);
	this->render_text_messages(w,h);
}

#endif
