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
{
}

CGlCanvasBase::~CGlCanvasBase()
{
	m_openGLScene.reset();
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

CGlCanvasBase::CamaraParams CGlCanvasBase::updateZoom(CamaraParams &params, int x, int y)
{
	params.cameraZoomDistance *= exp(0.01*(y - mouseClickY));
	if (params.cameraZoomDistance < 0.01) params.cameraZoomDistance = 0.01f;

	float	Az = -0.05 * (x - mouseClickX);
	float	D = 0.001 * params.cameraZoomDistance;
	params.cameraPointingZ += D*Az;

	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateZoom(CamaraParams &params, float delta)
{
	params.cameraZoomDistance *= 1 - 0.03f*(delta/120.0f);
	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::updateRotate(CamaraParams &params, int x, int y)
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

CGlCanvasBase::CamaraParams CGlCanvasBase::updateOrbitCamera(CamaraParams &params, int x, int y)
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

CGlCanvasBase::CamaraParams CGlCanvasBase::updatePan(CamaraParams &params, int x, int y)
{
	float	Ay = -(x - mouseClickX);
	float	Ax = -(y - mouseClickY);
	float	D = 0.001 * params.cameraZoomDistance;
	params.cameraPointingX += D*(Ax*cos(DEG2RAD(params.cameraAzimuthDeg)) - Ay*sin(DEG2RAD(params.cameraAzimuthDeg)));
	params.cameraPointingY += D*(Ax*sin(DEG2RAD(params.cameraAzimuthDeg)) + Ay*cos(DEG2RAD(params.cameraAzimuthDeg)));
	return params;
}

CGlCanvasBase::CamaraParams CGlCanvasBase::cameraParams()
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
	cameraElevationDeg = params.cameraElevationDeg;
	cameraAzimuthDeg = params.cameraAzimuthDeg;

	cameraIsProjective = params.cameraIsProjective;
	cameraFOV = params.cameraFOV;
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
