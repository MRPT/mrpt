/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/system/CTicTac.h>

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// Windows:
#include <windows.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif
#endif  // MRPT_HAS_OPENGL_GLUT

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace std;
using mrpt::system::CTicTac;

float CGlCanvasBase::SENSIBILITY_DEG_PER_PIXEL = 0.1f;

void CGlCanvasBase::setMinimumZoom(float zoom) { m_minZoom = zoom; }
void CGlCanvasBase::setMaximumZoom(float zoom) { m_maxZoom = zoom; }
void CGlCanvasBase::setMousePos(int x, int y)
{
	m_mouseClickX = x;
	m_mouseClickY = y;
}

void CGlCanvasBase::setMouseClicked(bool is) { mouseClicked = is; }
void CGlCanvasBase::updateZoom(CamaraParams& params, int x, int y) const
{
	float zoom = params.cameraZoomDistance * exp(0.01f * (y - m_mouseClickY));
	if (zoom <= m_minZoom || (m_maxZoom != -1.0f && m_maxZoom <= zoom)) return;
	params.cameraZoomDistance = zoom;
	if (params.cameraZoomDistance < 0.01f) params.cameraZoomDistance = 0.01f;

	float Az = -0.05f * (x - m_mouseClickX);
	float D = 0.001f * params.cameraZoomDistance;
	params.cameraPointingZ += D * Az;
}

void CGlCanvasBase::updateZoom(CamaraParams& params, float delta) const
{
	float zoom = params.cameraZoomDistance * (1 - 0.03f * (delta / 120.0f));
	if (zoom <= m_minZoom || (m_maxZoom != -1.0f && m_maxZoom <= zoom)) return;

	params.cameraZoomDistance = zoom;
}

void CGlCanvasBase::updateRotate(CamaraParams& params, int x, int y) const
{
	const float dis = max(0.01f, (params.cameraZoomDistance));
	float eye_x =
		params.cameraPointingX + dis * cos(DEG2RAD(params.cameraAzimuthDeg)) *
									 cos(DEG2RAD(params.cameraElevationDeg));
	float eye_y =
		params.cameraPointingY + dis * sin(DEG2RAD(params.cameraAzimuthDeg)) *
									 cos(DEG2RAD(params.cameraElevationDeg));
	float eye_z =
		params.cameraPointingZ + dis * sin(DEG2RAD(params.cameraElevationDeg));

	// Orbit camera:
	float A_AzimuthDeg = -SENSIBILITY_DEG_PER_PIXEL * (x - m_mouseClickX);
	params.cameraAzimuthDeg += A_AzimuthDeg;

	float A_ElevationDeg = SENSIBILITY_DEG_PER_PIXEL * (y - m_mouseClickY);
	params.setElevationDeg(params.cameraElevationDeg + A_ElevationDeg);

	// Move cameraPointing pos:
	params.cameraPointingX =
		eye_x - dis * cos(DEG2RAD(params.cameraAzimuthDeg)) *
					cos(DEG2RAD(params.cameraElevationDeg));
	params.cameraPointingY =
		eye_y - dis * sin(DEG2RAD(params.cameraAzimuthDeg)) *
					cos(DEG2RAD(params.cameraElevationDeg));
	params.cameraPointingZ =
		eye_z - dis * sin(DEG2RAD(params.cameraElevationDeg));
}

void CGlCanvasBase::updateOrbitCamera(CamaraParams& params, int x, int y) const
{
	params.cameraAzimuthDeg -= 0.2f * (x - m_mouseClickX);
	params.setElevationDeg(
		params.cameraElevationDeg + 0.2f * (y - m_mouseClickY));
}

void CGlCanvasBase::updateLastPos(int x, int y)
{
	m_mouseLastX = x;
	m_mouseLastY = y;
}

void CGlCanvasBase::resizeViewport(int w, int h)
{
#if MRPT_HAS_OPENGL_GLUT
	if (w == -1 || h == -1) return;

	glViewport(0, 0, (GLint)w, (GLint)h);
#endif
}

void CGlCanvasBase::clearColors()
{
#if MRPT_HAS_OPENGL_GLUT
	glClearColor(clearColorR, clearColorG, clearColorB, clearColorA);
#endif
}

void CGlCanvasBase::updatePan(CamaraParams& params, int x, int y) const
{
	float Ay = -(x - m_mouseClickX);
	float Ax = -(y - m_mouseClickY);
	float D = 0.001f * params.cameraZoomDistance;
	params.cameraPointingX += D * (Ax * cos(DEG2RAD(params.cameraAzimuthDeg)) -
								   Ay * sin(DEG2RAD(params.cameraAzimuthDeg)));
	params.cameraPointingY += D * (Ax * sin(DEG2RAD(params.cameraAzimuthDeg)) +
								   Ay * cos(DEG2RAD(params.cameraAzimuthDeg)));
}

CGlCanvasBase::CamaraParams CGlCanvasBase::cameraParams() const
{
	return m_cameraParams;
}

const CGlCanvasBase::CamaraParams& CGlCanvasBase::getRefCameraParams() const
{
	return m_cameraParams;
}

void CGlCanvasBase::setCameraParams(const CGlCanvasBase::CamaraParams& params)
{
	m_cameraParams = params;
}

float CGlCanvasBase::getZoomDistance() const
{
	return m_cameraParams.cameraZoomDistance;
}

void CGlCanvasBase::setZoomDistance(float zoom)
{
	m_cameraParams.cameraZoomDistance = zoom;
}

CCamera& CGlCanvasBase::updateCameraParams(CCamera& cam) const
{
	cam.setPointingAt(
		m_cameraParams.cameraPointingX, m_cameraParams.cameraPointingY,
		m_cameraParams.cameraPointingZ);
	cam.setZoomDistance(m_cameraParams.cameraZoomDistance);
	cam.setAzimuthDegrees(m_cameraParams.cameraAzimuthDeg);
	cam.setElevationDegrees(m_cameraParams.cameraElevationDeg);
	cam.setProjectiveModel(m_cameraParams.cameraIsProjective);
	cam.setProjectiveFOVdeg(m_cameraParams.cameraFOV);

	return cam;
}

void CGlCanvasBase::setUseCameraFromScene(bool is) { useCameraFromScene = is; }
bool CGlCanvasBase::getUseCameraFromScene() const { return useCameraFromScene; }
void CGlCanvasBase::setAzimuthDegrees(float ang)
{
	m_cameraParams.cameraAzimuthDeg = ang;
}

void CGlCanvasBase::setElevationDegrees(float ang)
{
	m_cameraParams.cameraElevationDeg = ang;
}

float CGlCanvasBase::getAzimuthDegrees() const
{
	return m_cameraParams.cameraAzimuthDeg;
}

float CGlCanvasBase::getElevationDegrees() const
{
	return m_cameraParams.cameraElevationDeg;
}

void CGlCanvasBase::setCameraProjective(bool is)
{
	m_cameraParams.cameraIsProjective = is;
}

bool CGlCanvasBase::isCameraProjective() const
{
	return m_cameraParams.cameraIsProjective;
}

void CGlCanvasBase::setCameraFOV(float FOV) { m_cameraParams.cameraFOV = FOV; }
float CGlCanvasBase::cameraFOV() const { return m_cameraParams.cameraFOV; }
void CGlCanvasBase::setClearColors(float r, float g, float b, float a)
{
	clearColorR = r;
	clearColorG = g;
	clearColorB = b;
	clearColorA = a;
}

float CGlCanvasBase::getClearColorR() const { return clearColorR; }
float CGlCanvasBase::getClearColorG() const { return clearColorG; }
float CGlCanvasBase::getClearColorB() const { return clearColorB; }
float CGlCanvasBase::getClearColorA() const { return clearColorA; }
void CGlCanvasBase::setOpenGLSceneRef(COpenGLScene::Ptr scene)
{
	m_openGLScene = scene;
}

void CGlCanvasBase::setCameraPointing(float pointX, float pointY, float pointZ)
{
	m_cameraParams.cameraPointingX = pointX;
	m_cameraParams.cameraPointingY = pointY;
	m_cameraParams.cameraPointingZ = pointZ;
}

float CGlCanvasBase::getCameraPointingX() const
{
	return m_cameraParams.cameraPointingX;
}

float CGlCanvasBase::getCameraPointingY() const
{
	return m_cameraParams.cameraPointingY;
}

float CGlCanvasBase::getCameraPointingZ() const
{
	return m_cameraParams.cameraPointingZ;
}

double CGlCanvasBase::renderCanvas(int width, int height)
{
#if MRPT_HAS_OPENGL_GLUT
	CTicTac tictac;
	double At = 0.1;

	try
	{
		CHECK_OPENGL_ERROR();

		// Call PreRender user code:
		preRender();
		CHECK_OPENGL_ERROR();

		// Set static configs:
		glEnable(GL_DEPTH_TEST);
		CHECK_OPENGL_ERROR();

		// Set the viewport
		resizeViewport((GLsizei)width, (GLsizei)height);

		// Set the background color:
		clearColors();

		if (m_openGLScene)
		{
			// Set the camera params in the scene:
			if (!useCameraFromScene)
			{
				COpenGLViewport::Ptr view = m_openGLScene->getViewport("main");
				if (!view)
				{
					THROW_EXCEPTION(
						"Fatal error: there is no 'main' viewport in the 3D "
						"scene!");
				}

				mrpt::opengl::CCamera& cam = view->getCamera();
				updateCameraParams(cam);
			}

			tictac.Tic();

			// Draw primitives:
			m_openGLScene->render();

		}  // end if "m_openGLScene!=nullptr"

		postRender();

		// Flush & swap buffers to disply new image:
		glFlush();
		swapBuffers();

		At = tictac.Tac();
	}
	catch (const std::exception& e)
	{
		const std::string err_msg =
			std::string("[CGLCanvasBase::Render] Exception:\n") +
			mrpt::exception_to_str(e);
		std::cerr << err_msg;
		renderError(err_msg);
	}

	return At;
#else
	THROW_EXCEPTION("Cant render: MRPT was built without OpenGL");
#endif
}

void CGlCanvasBase::CamaraParams::setElevationDeg(float deg)
{
	cameraElevationDeg = deg;

	if (cameraElevationDeg < -90.0f)
		cameraElevationDeg = -90.0f;
	else if (cameraElevationDeg > 90.0f)
		cameraElevationDeg = 90.0f;
}
