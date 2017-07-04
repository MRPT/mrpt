/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once
#include "mrpt/opengl/COpenGLScene.h"

namespace mrpt {
namespace opengl {
class CCamera;
}
}

namespace mrpt
{
namespace gui
{
/** This base class implements a working with opengl::Camera and a OpenGL canvas, and it's used in gui::CMyGLCanvasBase and gui::CQGLCanvasBase.
*/
class CGlCanvasBase
{
public:
	struct CamaraParams
	{
		CamaraParams();
		void setElevationDeg(float deg);

		float	cameraPointingX, cameraPointingY, cameraPointingZ;
		float	cameraZoomDistance;
		float	cameraElevationDeg,cameraAzimuthDeg;
		bool	cameraIsProjective;
		float	cameraFOV;
	};

	CGlCanvasBase();
	virtual ~CGlCanvasBase();

	void setMousePos(int x, int y);
	void setMouseClicked(bool is);
	void updateLastPos(int x, int y);

	CamaraParams updateZoom(CamaraParams &params, int x, int y);
	CamaraParams updateZoom(CamaraParams &params, float delta);
	CamaraParams updateRotate(CamaraParams &params, int x, int y);
	CamaraParams updateOrbitCamera(CamaraParams &params, int x, int y);
	CamaraParams updatePan(CamaraParams &params, int x, int y);

	CamaraParams cameraParams();
	void setCameraParams(const CamaraParams &params);

	void updateCameraParams(mrpt::opengl::CCamera &cam) const;
	void setUseCameraFromScene(bool is);

	// Visualization params:
	float	cameraPointingX, cameraPointingY, cameraPointingZ;
	float	cameraZoomDistance;
	float	cameraElevationDeg,cameraAzimuthDeg;
	bool	cameraIsProjective;
	float	cameraFOV;
	float	clearColorR,clearColorG,clearColorB;

	/** If set to true (default=false), the cameraPointingX,... parameters are ignored and the camera stored in the 3D scene is used instead.
	  */
	bool	useCameraFromScene;

	static float  SENSIBILITY_DEG_PER_PIXEL;		// Default = 0.1

	/** Overload this method to limit the capabilities of the user to move the camera using the mouse.
			  *  For all these variables:
			  *  - cameraPointingX
			  *  - cameraPointingY
			  *  - cameraPointingZ
			  *  - cameraZoomDistance
			  *  - cameraElevationDeg
			  *  - cameraAzimuthDeg
			  *
			  *  A "new_NAME" variable will be passed with the temptative new value after the user action.
			  *   The default behavior should be to copy all the new variables to the variables listed above
			  *   but in the middle any find of user-defined filter can be implemented.
			  */
	virtual void OnUserManuallyMovesCamera(
			float	new_cameraPointingX,
			float 	new_cameraPointingY,
			float 	new_cameraPointingZ,
			float	new_cameraZoomDistance,
			float	new_cameraElevationDeg,
			float	new_cameraAzimuthDeg )
	{
		cameraPointingX 	= new_cameraPointingX;
		cameraPointingY 	= new_cameraPointingY;
		cameraPointingZ 	= new_cameraPointingZ;
		cameraZoomDistance	= new_cameraZoomDistance;
		cameraElevationDeg	= new_cameraElevationDeg;
		cameraAzimuthDeg 	= new_cameraAzimuthDeg;
	}

	inline void getLastMousePosition(int &x,int& y) const
	{
		x =m_mouseLastX;
		y =m_mouseLastY;
	}

	/**  At constructor an empty scene is created. The object is freed at GL canvas destructor.
			  */
	mrpt::opengl::COpenGLScene::Ptr		m_openGLScene;

private:
	int		m_mouseLastX, m_mouseLastY;
	int		mouseClickX, mouseClickY;
	bool	mouseClicked;

};  // end of class

}	// end namespace
}	// end namespace

