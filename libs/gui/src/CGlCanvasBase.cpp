/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/utils/CTicTac.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace std;

float  CGlCanvasBase::SENSIBILITY_DEG_PER_PIXEL = 0.1f;

#if MRPT_HAS_OPENGL_GLUT
	#ifdef MRPT_OS_WINDOWS
	// Windows:
	#include <windows.h>
	#endif

	#ifdef MRPT_OS_APPLE
	#include <OpenGL/gl.h>
	#include <OpenGL/glu.h>
	#include <GLUT/glut.h>
	#else
	#include <GL/gl.h>
	#include <GL/glu.h>
	#include <GL/glut.h>
		#ifdef HAVE_FREEGLUT_EXT_H
		#include <GL/freeglut_ext.h>
		#endif
	#endif

#endif


CGlCanvasBase::CGlCanvasBase()
	: cameraPointingX(0)
	, cameraPointingY(0)
	, cameraPointingZ(0)
	, cameraZoomDistance(40)
	, cameraElevationDeg(45)
	, cameraAzimuthDeg(45)
	, cameraIsProjective(true)
	, cameraFOV(30)
	, clearColorR(0.4f)
	, clearColorG(0.4f)
	, clearColorB(0.4f)
	, useCameraFromScene(false)
	, m_openGLScene(COpenGLScene::Create())
	, m_mouseLastX(0)
	, m_mouseLastY(0)
	, mouseClickX(0)
	, mouseClickY(0)
	, mouseClicked(false)
	, m_minZoom(1.0)
	, m_maxZoom(3200.0)
{
}

CGlCanvasBase::~CGlCanvasBase()
{
	m_openGLScene.reset();
}

void CGlCanvasBase::setMinimumZoom(float zoom)
{
	m_minZoom = zoom;
}

void CGlCanvasBase::setMaximumZoom(float zoom)
{
	m_maxZoom = zoom;
}

void CGlCanvasBase::setMousePos(int x, int y)
{
	mouseClickX = x;
	mouseClickY = y;
}

void CGlCanvasBase::setMouseClicked(bool is)
{
	mouseClicked = is;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateZoom(CamaraParams &params, int x, int y) const
{
	float zoom = params.cameraZoomDistance * exp(0.01*(y - mouseClickY));
	if (zoom <= m_minZoom && m_maxZoom >= zoom)
		return params;
	params.cameraZoomDistance = zoom;
	if (params.cameraZoomDistance < 0.01) params.cameraZoomDistance = 0.01f;

	float	Az = -0.05 * (x - mouseClickX);
	float	D = 0.001 * params.cameraZoomDistance;
	params.cameraPointingZ += D*Az;

	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateZoom(CamaraParams &params, float delta) const
{
	float zoom = params.cameraZoomDistance * (1 - 0.03f*(delta/120.0f));
	if (zoom <= m_minZoom && m_maxZoom >= zoom)
		return params;

	params.cameraZoomDistance = zoom;
	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateRotate(CamaraParams &params, int x, int y) const
{
	const float dis = max(0.01f,(params.cameraZoomDistance));
	float	eye_x = params.cameraPointingX +  dis * cos(DEG2RAD(params.cameraAzimuthDeg))*cos(DEG2RAD(params.cameraElevationDeg));
	float	eye_y = params.cameraPointingY +  dis * sin(DEG2RAD(params.cameraAzimuthDeg))*cos(DEG2RAD(params.cameraElevationDeg));
	float	eye_z = params.cameraPointingZ +  dis * sin(DEG2RAD(params.cameraElevationDeg));

	// Orbit camera:
	float A_AzimuthDeg = -SENSIBILITY_DEG_PER_PIXEL*(x - mouseClickX);
	params.cameraAzimuthDeg += A_AzimuthDeg;

	float A_ElevationDeg = SENSIBILITY_DEG_PER_PIXEL*(y - mouseClickY);
	params.setElevationDeg(params.cameraElevationDeg + A_ElevationDeg);

	// Move cameraPointing pos:
	params.cameraPointingX = eye_x - dis * cos(DEG2RAD(params.cameraAzimuthDeg))*cos(DEG2RAD(params.cameraElevationDeg));
	params.cameraPointingY = eye_y - dis * sin(DEG2RAD(params.cameraAzimuthDeg))*cos(DEG2RAD(params.cameraElevationDeg));
	params.cameraPointingZ = eye_z - dis * sin(DEG2RAD(params.cameraElevationDeg));

	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateOrbitCamera(CamaraParams &params, int x, int y) const
{
	params.cameraAzimuthDeg -= 0.2*(x - mouseClickX);
	params.setElevationDeg(params.cameraElevationDeg + 0.2*(y - mouseClickY));

	return params;
}

void CGlCanvasBase::updateLastPos(int x, int y)
{
	m_mouseLastX = x;
	m_mouseLastY = y;
}

void CGlCanvasBase::resizeViewport(int w, int h)
{
	if (w == -1 || h == -1)
		return;

	glViewport(0,0,(GLint) w, (GLint) h);
}

void CGlCanvasBase::clearColors()
{
	glClearColor(clearColorR,clearColorG,clearColorB,1.0);
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updatePan(CamaraParams &params, int x, int y) const
{
	float	Ay = -(x - mouseClickX);
	float	Ax = -(y - mouseClickY);
	float	D = 0.001 * params.cameraZoomDistance;
	params.cameraPointingX += D*(Ax*cos(DEG2RAD(params.cameraAzimuthDeg)) - Ay*sin(DEG2RAD(params.cameraAzimuthDeg)));
	params.cameraPointingY += D*(Ax*sin(DEG2RAD(params.cameraAzimuthDeg)) + Ay*cos(DEG2RAD(params.cameraAzimuthDeg)));
	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::cameraParams() const
{
	CamaraParams params;
	params.cameraPointingX = cameraPointingX;
	params.cameraPointingY = cameraPointingY;
	params.cameraPointingZ = cameraPointingZ;

	params.cameraZoomDistance = cameraZoomDistance;
	params.cameraElevationDeg = cameraElevationDeg;
	params.cameraAzimuthDeg = cameraAzimuthDeg;

	params.cameraIsProjective = cameraIsProjective;
	params.cameraFOV = cameraFOV;

	return params;
}

void CGlCanvasBase::setCameraParams(const CGlCanvasBase::CamaraParams &params)
{
	cameraPointingX = params.cameraPointingX;
	cameraPointingY = params.cameraPointingY;
	cameraPointingZ = params.cameraPointingZ;

	cameraZoomDistance = params.cameraZoomDistance;
	setElevationDegrees(params.cameraElevationDeg);
	setAzimuthDegrees(params.cameraAzimuthDeg);

	cameraIsProjective = params.cameraIsProjective;
	cameraFOV = params.cameraFOV;
}

float CGlCanvasBase::getZoomDistance() const
{
	return cameraZoomDistance;
}

void CGlCanvasBase::updateCameraParams(CCamera &cam) const
{
	cam.setPointingAt( cameraPointingX, cameraPointingY, cameraPointingZ );
	cam.setZoomDistance(cameraZoomDistance);
	cam.setAzimuthDegrees( cameraAzimuthDeg );
	cam.setElevationDegrees(cameraElevationDeg);
	cam.setProjectiveModel( cameraIsProjective );
	cam.setProjectiveFOVdeg( cameraFOV );
}

void CGlCanvasBase::setUseCameraFromScene(bool is)
{
	useCameraFromScene = is;
}

void CGlCanvasBase::setAzimuthDegrees(float ang)
{
	cameraAzimuthDeg = ang;
}

void CGlCanvasBase::setElevationDegrees(float ang)
{
	cameraElevationDeg = ang;
}

float CGlCanvasBase::getAzimuthDegrees() const
{
	return cameraAzimuthDeg;
}

float CGlCanvasBase::getElevationDegrees() const
{
	return cameraElevationDeg;
}

double CGlCanvasBase::renderCanvas(int width, int height)
{
	CTicTac tictac;
	double	At = 0.1;

	try
	{
		// Call PreRender user code:
		preRender();


		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set static configs:
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_ALPHA_TEST);
		glEnable(GL_TEXTURE_2D);


		// PART 1a: Set the viewport
		// --------------------------------------
		resizeViewport( (GLsizei)width, (GLsizei)height);

		// Set the background color:
		clearColors();

		if (m_openGLScene)
		{
			// Set the camera params in the scene:
			if (!useCameraFromScene)
			{
				COpenGLViewport::Ptr view= m_openGLScene->getViewport("main");
				if (!view)
				{
					THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");
				}

				mrpt::opengl::CCamera & cam = view->getCamera();
				updateCameraParams(cam);
			}

			// PART 2: Set the MODELVIEW matrix
			// --------------------------------------
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			tictac.Tic();

			// PART 3: Draw primitives:
			// --------------------------------------
			m_openGLScene->render();

		} // end if "m_openGLScene!=nullptr"

		postRender();

		// Flush & swap buffers to disply new image:
		glFlush();
		swapBuffers();

		At = tictac.Tac();

		glPopAttrib();
	}
	catch (std::exception &e)
	{
		glPopAttrib();
		const std::string err_msg = std::string("[CMyGLCanvasBase::Render] Exception!: ") +std::string(e.what());
		std::cerr << err_msg;
		renderError(err_msg);
	}
	catch (...)
	{
		glPopAttrib();
		std::cerr << "Runtime error!" << std::endl;
	}

	return At;
}


CGlCanvasBase::CamaraParams::CamaraParams()
	: cameraPointingX(0)
	, cameraPointingY(0)
	, cameraPointingZ(0)
	, cameraZoomDistance(40)
	, cameraElevationDeg(45)
	, cameraAzimuthDeg(45)
	, cameraIsProjective(true)
	, cameraFOV(30)
{
}

void CGlCanvasBase::CamaraParams::setElevationDeg(float deg)
{
	cameraElevationDeg = deg;

	if (cameraElevationDeg < -90)
		cameraElevationDeg = -90;
	else if (cameraElevationDeg > 90)
		cameraElevationDeg = 90;
}
