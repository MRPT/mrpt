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
	mrpt::opengl::COpenGLViewportPtr gl_view = m_openGLScene->getViewport("small-view");
	if (gl_view)
	{
		mrpt::opengl::CCamera & view_cam = gl_view->getCamera();

		view_cam.setAzimuthDegrees( this->cameraAzimuthDeg );
		view_cam.setElevationDegrees( this->cameraElevationDeg );
		view_cam.setZoomDistance( 4 );
	}
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
}

void CMyGLCanvas::OnPostRender()
{
	// Show credits on the screen? renderTextBitmap(20, 20, "" );
}

#endif
