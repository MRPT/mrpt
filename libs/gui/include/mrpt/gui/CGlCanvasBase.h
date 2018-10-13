/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/opengl/COpenGLScene.h>

namespace mrpt
{
namespace opengl
{
class CCamera;
}
}  // namespace mrpt

namespace mrpt
{
namespace gui
{
/** This base class implements a working with opengl::Camera and a OpenGL
 * canvas, and it's used in gui::CWxGLCanvasBase and gui::CQtGlCanvasBase.
 */
class CGlCanvasBase
{
   public:
	struct CamaraParams
	{
		CamaraParams() = default;
		void setElevationDeg(float deg);

		float cameraPointingX = .0f, cameraPointingY = .0f,
			  cameraPointingZ = .0f;
		float cameraZoomDistance = 40.f;
		float cameraElevationDeg = 45.f, cameraAzimuthDeg = 45.f;
		bool cameraIsProjective = true;
		float cameraFOV = 30.f;
	};

	CGlCanvasBase() = default;
	virtual ~CGlCanvasBase() = default;
	/** Sets the minimum of the zoom
	 * See also setMaximumZoom(float) */
	void setMinimumZoom(float zoom);

	/** Sets the maximum of the zoom
	 * See also setMinimumZoom(float) */
	void setMaximumZoom(float zoom);

	/** Saves the click position of the mouse
	 * See also setMouseClicked(bool) */
	void setMousePos(int x, int y);

	/** Sets the property mouseClicked
	 * By default, this property is false.
	 * See also setMousePos(int, int) */
	void setMouseClicked(bool is);

	/** Sets the last mouse position */
	void updateLastPos(int x, int y);
	/** Calls the glViewport function*/
	void resizeViewport(int w, int h);
	/** Calls the glClearColor function
	 * See also setClearColors(float, float, float, float)*/
	void clearColors();

	/** This function for the mouse event
	 * It gets a reference to CamaraParams, x, y
	 * and updates the zoom of the CameraParams.
	 * See also updateZoom(CamaraParams &, float)*/
	void updateZoom(CamaraParams& params, int x, int y) const;
	/** This function for the wheel event
	 * It gets a reference to CamaraParams, delta
	 * and updates the zoom of the CameraParams.
	 * See also updateZoom(CamaraParams &, int, int)*/
	void updateZoom(CamaraParams& params, float delta) const;
	/** This function for the mouse event
	 * It gets a reference to CamaraParams, x, y
	 * and updates the elevation and azimuth.
	 * See also getElevationDegrees(), getAzimuthDegrees()*/
	void updateRotate(CamaraParams& params, int x, int y) const;
	/** This function for the mouse event
	 * It gets a reference to CamaraParams, x, y
	 * and updates the elevation and azimuth.
	 * See also getElevationDegrees(), getAzimuthDegrees()*/
	void updateOrbitCamera(CamaraParams& params, int x, int y) const;
	/** This function for the mouse event
	 * It gets a reference to CamaraParams, x, y
	 * and updates the pointing of the camera.
	 * See also getCameraPointingX(), getCameraPointingY(),
	 * getCameraPointingZ()*/
	void updatePan(CamaraParams& params, int x, int y) const;

	/** Returns a copy of CamaraParams
	 * See also getRefCameraParams(), setCameraParams(const CamaraParams &)*/
	CamaraParams cameraParams() const;
	/** Returns a reference to CamaraParams
	 * See also cameraParams(), setCameraParams(const CamaraParams &) */
	const CamaraParams& getRefCameraParams() const;
	/** Sets the CamaraParams
	 * See also cameraParams(), getRefCameraParams()*/
	virtual void setCameraParams(const CamaraParams& params);

	/** This function gets a reference to mrpt::opengl::CCamera and
	 * updates the camera parameters(pointing, zoom, azimuth, elevation,
	 * IsProjective, FOV)
	 */
	mrpt::opengl::CCamera& updateCameraParams(mrpt::opengl::CCamera& cam) const;

	/** If set to true (default=false), the cameraPointingX,... parameters are
	 * ignored and the camera stored in the 3D scene is used instead.
	 * See also void bool getUseCameraFromScene()
	 */
	void setUseCameraFromScene(bool is);

	/** See also void setUseCameraFromScene(bool)
	 */
	bool getUseCameraFromScene() const;

	// Visualization params:
	/** Saves the pointing of the camera
	 * See also getCameraPointingX(), getCameraPointingY(), getCameraPointingZ()
	 */
	virtual void setCameraPointing(float pointX, float pointY, float pointZ);

	/** Returns the x pointing of the camera
	 * See also setCameraPointing(float, float, float)
	 */
	float getCameraPointingX() const;

	/** Returns the y pointing of the camera
	 * See also setCameraPointing(float, float, float)
	 */
	float getCameraPointingY() const;

	/** Returns the z pointing of the camera
	 * See also setCameraPointing(float, float, float)
	 */
	float getCameraPointingZ() const;

	/** Saves camera zooming
	 * See also getZoomDistance()
	 */
	virtual void setZoomDistance(float zoom);

	/** Returns a zoom
	 * See also setZoomDistance(float)
	 */
	float getZoomDistance() const;

	/** Saves the degrees of the azimuth camera
	 * See also getAzimuthDegrees()
	 */
	virtual void setAzimuthDegrees(float ang);

	/** Returns a azimuth degrees
	 * See also setAzimuthDegrees(float)
	 */
	float getAzimuthDegrees() const;

	/** Saves the degrees of the elevation camera
	 * See also getElevationDegrees()
	 */
	virtual void setElevationDegrees(float ang);

	/** Returns a elevation degrees
	 * See also setElevationDegrees(float)
	 */
	float getElevationDegrees() const;

	virtual void setCameraProjective(bool is);
	bool isCameraProjective() const;

	virtual void setCameraFOV(float FOV);
	float cameraFOV() const;

	/** Sets the RGBA colors for glClearColor
	 * See also clearColors(), getClearColorR(),
	 * getClearColorG(),getClearColorB(), getClearColorA()
	 */
	void setClearColors(float r, float g, float b, float a = 1.0f);
	float getClearColorR() const;
	float getClearColorG() const;
	float getClearColorB() const;
	float getClearColorA() const;

	static float SENSIBILITY_DEG_PER_PIXEL;  // Default = 0.1

	/** Overload this method to limit the capabilities of the user to move the
	 * camera using the mouse.
	 *  For all these variables:
	 *  - cameraPointingX
	 *  - cameraPointingY
	 *  - cameraPointingZ
	 *  - cameraZoomDistance
	 *  - cameraElevationDeg
	 *  - cameraAzimuthDeg
	 *
	 *  A "new_NAME" variable will be passed with the temptative new
	 * value after the user action.
	 *   The default behavior should be to copy all the new variables
	 * to the variables listed above
	 *   but in the middle any find of user-defined filter can be
	 * implemented.
	 */
	virtual void OnUserManuallyMovesCamera(
		float new_cameraPointingX, float new_cameraPointingY,
		float new_cameraPointingZ, float new_cameraZoomDistance,
		float new_cameraElevationDeg, float new_cameraAzimuthDeg)
	{
		m_cameraParams.cameraPointingX = new_cameraPointingX;
		m_cameraParams.cameraPointingY = new_cameraPointingY;
		m_cameraParams.cameraPointingZ = new_cameraPointingZ;
		m_cameraParams.cameraZoomDistance = new_cameraZoomDistance;
		m_cameraParams.cameraElevationDeg = new_cameraElevationDeg;
		m_cameraParams.cameraAzimuthDeg = new_cameraAzimuthDeg;
	}

	inline void getLastMousePosition(int& x, int& y) const
	{
		x = m_mouseLastX;
		y = m_mouseLastY;
	}

	/**  At constructor an empty scene is created. The object is freed at GL
	canvas destructor.
	 * This function returns a smart pointer to the opengl scene
	getOpenGLSceneRef		  */
	mrpt::opengl::COpenGLScene::Ptr& getOpenGLSceneRef()
	{
		return m_openGLScene;
	}
	void setOpenGLSceneRef(mrpt::opengl::COpenGLScene::Ptr scene);

   protected:
	virtual void swapBuffers() = 0;
	virtual void preRender() = 0;
	virtual void postRender() = 0;
	virtual void renderError(const std::string& err_msg) = 0;

	virtual double renderCanvas(int width = -1, int height = -1);

   private:
	float clearColorR = .4f, clearColorG = .4f, clearColorB = .4f,
		  clearColorA = 1.f;
	bool useCameraFromScene = false;
	mrpt::opengl::COpenGLScene::Ptr m_openGLScene =
		mrpt::make_aligned_shared<mrpt::opengl::COpenGLScene>();
	int m_mouseLastX = 0, m_mouseLastY = 0;
	int m_mouseClickX = 0, m_mouseClickY = 0;
	bool mouseClicked = false;
	float m_minZoom = 1.f;
	float m_maxZoom = 3200.f;
	CamaraParams m_cameraParams;
};  // end of class
}  // namespace gui
}  // namespace mrpt
