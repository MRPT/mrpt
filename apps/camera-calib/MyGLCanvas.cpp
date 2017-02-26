/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "MyGLCanvas.h"

//#include "mono_slamMain.h"

using namespace std;

CMyGLCanvas::CMyGLCanvas( wxWindow *parent, wxWindowID id,
                 const wxPoint& pos, const wxSize& size,
                 long style, const wxString& name )
		: CMyGLCanvasBase(parent,id,pos,size,style,name)
{
	cameraPointingX = 0;
	cameraPointingY = 0;
	cameraPointingZ = 0;
	cameraZoomDistance = 6;
	cameraElevationDeg = 25;
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
	// Do we have to update the scene??
/*	SYNCH::CCriticalSectionLocker   lock( &critSec_UpdateScene );
	if (newOpenGLScene)
	{
		if (m_openGLScene) delete m_openGLScene;
		m_openGLScene = newOpenGLScene;
		newOpenGLScene = NULL;
	}*/
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
}

void CMyGLCanvas::OnPostRender()
{
	// Show credits on the screen??
//	renderTextBitmap(20, 20, "RoadBot GUI, Copyright 2008 UMA" );

}



