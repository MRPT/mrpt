/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef  CDisplayWindow3D_H
#define  CDisplayWindow3D_H

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/utils/CImage.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		class CImage;
		class CImageFloat;
	}

	namespace gui
	{
		using namespace mrpt::utils;

		class C3DWindowDialog;
		class CMyGLCanvas_DisplayWindow3D;

		DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE(CDisplayWindow3D, mrpt::gui::CBaseGUIWindow, GUI_IMPEXP)

		/** A graphical user interface (GUI) for efficiently rendering 3D scenes in real-time.
		  *  This class always contains internally an instance of opengl::COpenGLScene, which
		  *   the objects, viewports, etc. to be rendered.
		  *
		  *  Images can be grabbed automatically to disk for easy creation of videos.
		  *  See CDisplayWindow3D::grabImagesStart  (and for creating videos, mrpt::utils::CVideoFileWriter).
		  *
		  *
		  *  Since the 3D rendering is performed in a detached thread, especial care must be taken
		  *   when updating the 3D scene to be rendered. The process involves an internal critical section
		  *   and it must always consist of these steps:
		  *
		  * \code
		  *   CDisplayWindow3D	win("My window");
		  *   
		  *   // Adquire the scene:
		  *   opengl::COpenGLScenePtr &ptrScene = win.get3DSceneAndLock();
		  *   
		  *   // Modify the scene:
		  *   ptrScene->...
		  *   // or replace by another scene:
		  *   ptrScene = otherScene;
		  *   
		  *   // Unlock it, so the window can use it for redraw:
		  *   win.unlockAccess3DScene();
		  *   
		  *   // Update window, if required
		  *   win.forceRepaint();
		  * \endcode
		  *
		  * An alternative way of updating the scene is by creating, before locking the 3D window, a new object
		  *  of class COpenGLScene, then locking the window only for replacing the smart pointer. This may be
		  *  advantageous is generating the 3D scene takes a long time, since while the window
		  *  is locked it will not be responsive to the user input or window redraw.
		  *
		  * The window can also display a set of 2D text messages overlapped to the 3D scene.
		  *  See CDisplayWindow3D::add2DTextMessage
		  *
		  *  For a list of supported events with the observer/observable pattern, see the discussion in mrpt::gui::CBaseGUIWindow.
		  *
		  *
		  * \sa  The example /samples/display3D, the <a href="http://www.mrpt.org/Tutorial_3D_Scenes" > tutorial only</a>.
		  */
		class GUI_IMPEXP CDisplayWindow3D : public mrpt::gui::CBaseGUIWindow
		{
			// This must be added to any CObject derived class:
			DEFINE_MRPT_OBJECT( CDisplayWindow3D )

			friend class C3DWindowDialog;
			friend class CMyGLCanvas_DisplayWindow3D;


			float m_minRange,m_maxRange,m_FOV;


			/** Internal OpenGL object (see general discussion in about usage of this object)
			  */
			opengl::COpenGLScenePtr			m_3Dscene;

			/** Critical section for accesing m_3Dscene
			  */
			synch::CCriticalSection		m_csAccess3DScene;

			/** Throws an exception on initialization error
			  */
			void  createOpenGLContext();

			void_ptr_noncopy	m_DisplayDeviceContext;
			void_ptr_noncopy	m_GLRenderingContext;

			std::string 		m_grab_imgs_prefix;
			unsigned int		m_grab_imgs_idx;

			bool				m_is_capturing_imgs;
			CImagePtr		m_last_captured_img;
			synch::CCriticalSection		m_last_captured_img_cs;

			void  doRender();

			mrpt::system::TTimeStamp 	m_lastFullScreen;

		public:
			/** Constructor
			 */
			CDisplayWindow3D(
				const std::string	&windowCaption = std::string(),
				unsigned int		initialWindowWidth = 400,
				unsigned int		initialWindowHeight = 300 );

			/** Class factory returning a smart pointer */
			static CDisplayWindow3DPtr Create(
				const std::string	&windowCaption = std::string(),
				unsigned int		initialWindowWidth = 400,
				unsigned int		initialWindowHeight = 300 )
			{
				return CDisplayWindow3DPtr(new CDisplayWindow3D(windowCaption,initialWindowWidth,initialWindowHeight));
			}

			/** Destructor
			 */
			virtual ~CDisplayWindow3D();

			/** Gets a reference to the smart shared pointer that holds the internal scene (carefuly read introduction in gui::CDisplayWindow3D before use!)
			  *  This also locks the critical section for accesing the scene, thus the window will not be repainted until it is unlocked.
			  */
			opengl::COpenGLScenePtr & get3DSceneAndLock( );

			/** Unlocks the access to the internal 3D scene.
			  *  Typically user will want to call forceRepaint after updating the scene.
			  */
			void  unlockAccess3DScene();

			/** Repaints the window.
			  * forceRepaint, repaint and updateWindow are all aliases of the same method.
			  */
			void  forceRepaint();

			/** Repaints the window.
			  * forceRepaint, repaint and updateWindow are all aliases of the same method.
			  */
			void  repaint() { forceRepaint(); }

			/** Repaints the window.
			  * forceRepaint, repaint and updateWindow are all aliases of the same method.
			  */
			void  updateWindow() { forceRepaint(); }

			/** Return the camera min range (z) (used for gluPerspective).
			  */
			float getMinRange() const { return m_minRange; };

			/** Return the camera max range (z) (used for gluPerspective).
			  */
			float getMaxRange() const { return m_maxRange; };

			/** Return the camera field of view (in degrees) (used for gluPerspective).
			  */
			float getFOV() const { return m_FOV; };

			/** Changes the camera min range (z) (used for gluPerspective).
			  *  The window is not updated with this method, call "forceRepaint" to update the 3D view.
			  */
			void setMinRange(float v) { m_minRange=v; };

			/** Changes the camera max range (z) (used for gluPerspective).
			  *  The window is not updated with this method, call "forceRepaint" to update the 3D view.
			  */
			void setMaxRange(float v) { m_maxRange=v; };

			/** Changes the camera field of view (in degrees) (used for gluPerspective).
			  *  The window is not updated with this method, call "forceRepaint" to update the 3D view.
			  */
			void setFOV(float v)  { m_FOV=v; };

			/** Resizes the window, stretching the image to fit into the display area.
			 */
			void  resize( unsigned int width, unsigned int height );

			/** Changes the position of the window on the screen.
			 */
			void  setPos( int x, int y );

			/** Changes the window title.
			  */
			void  setWindowTitle( const std::string &str );

			/** Changes the camera parameters programatically
			  */
			void setCameraElevationDeg( float deg );

			/** Changes the camera parameters programatically
			  */
			void setCameraAzimuthDeg( float deg );

			/** Changes the camera parameters programatically
			  */
			void setCameraPointingToPoint( float x,float y, float z );

			/** Changes the camera parameters programatically
			  */
			void setCameraZoom( float zoom );

			/** Sets the camera as projective, or orthogonal. */
			void setCameraProjective( bool isProjective );


			/** Get camera parameters programatically */
			float getCameraElevationDeg() const;

			/** Get camera parameters programatically  */
			float getCameraAzimuthDeg() const;

			/** Get camera parameters programatically  */
			void getCameraPointingToPoint( float &x,float &y, float &z ) const;

			/** Get camera parameters programatically */
			float getCameraZoom() const;

			/** Sets the camera as projective, or orthogonal. */
			bool isCameraProjective() const;

			/**  If set to true (default = false), the mouse-based scene navigation will be disabled and the camera position will be determined by the opengl viewports in the 3D scene.
			  */
			void useCameraFromScene(bool useIt = true);

			/** Gets the last x,y pixel coordinates of the mouse. \return False if the window is closed. */
			bool getLastMousePosition(int &x, int &y) const;

			/** Set cursor style to default (cursorIsCross=false) or to a cross (cursorIsCross=true) */
			void setCursorCross(bool cursorIsCross);

			/** Start to save rendered images to disk.
			  *  Images will be saved independently as png files, depending on
			  *   the template path passed to this method. For example:
			  *
			  *  path_prefix: "./video_"
			  *
			  *  Will generate "./video_000001.png", etc.
			  *
			  *  \sa grabImagesStop
			  */
			void grabImagesStart( const std::string &grab_imgs_prefix = std::string("video_") );

			/** Stops image grabbing started by grabImagesStart
			  * \sa grabImagesStart
			  */
			void grabImagesStop();

			/** Enables the grabbing of CImage objects from screenshots of the window.
			  *  \sa getLastWindowImage
			  */
			void captureImagesStart();

			/** Stop image grabbing
			  * \sa captureImagesStart
			  */
			void captureImagesStop();

			/** Retrieve the last captured image from the window.
			  *  You MUST CALL FIRST captureImagesStart to enable image grabbing.
			  * \sa captureImagesStart, getLastWindowImagePtr
			  */
			void getLastWindowImage( mrpt::utils::CImage &out_img) const;

			/** Retrieve the last captured image from the window, as a smart pointer.
			  *  This method is more efficient than getLastWindowImage since only a copy of the pointer is performed, while
			  *   getLastWindowImage would copy the entire image.
			  *
			  *  You MUST CALL FIRST captureImagesStart to enable image grabbing.
			  * \sa captureImagesStart, getLastWindowImage
			  */
			mrpt::utils::CImagePtr getLastWindowImagePtr() const;

			/** Increments by one the image counter and return the next image file name (Users normally don't want to call this method).
			  * \sa grabImagesStart
			  */
			std::string grabImageGetNextFile();

			bool isCapturingImgs() const {  return m_is_capturing_imgs; }


			/** Add 2D text messages overlapped to the 3D rendered scene. The string will remain displayed in the 3D window
			  *   until it's changed with subsequent calls to this same method, or all the texts are cleared with clearTextMessages().
			  *
			  *  \param x The X position, interpreted as absolute pixels from the left if X>=1, absolute pixels from the left if X<0 or as a width factor if in the range [0,1[.
			  *  \param y The Y position, interpreted as absolute pixels from the bottom if Y>=1, absolute pixels from the top if Y<0 or as a height factor if in the range [0,1[.
			  *  \param text The text string to display.
			  *  \param color The text color. For example: TColorf(1.0,1.0,1.0)
			  *  \param unique_index An "index" for this text message, so that subsequent calls with the same index will overwrite this text message instead of creating new ones.
			  *
			  *  You'll need to refresh the display manually with forceRepaint().
			  *
			  * \sa clearTextMessages
			  */
			void addTextMessage(
				const double x,
				const double y,
				const std::string &text,
				const mrpt::utils::TColorf &color = mrpt::utils::TColorf(1.0,1.0,1.0),
				const size_t unique_index = 0,
				const TOpenGLFont font = MRPT_GLUT_BITMAP_TIMES_ROMAN_24
				);

			/**  Clear all text messages created with addTextMessage().
			  *  You'll need to refresh the display manually with forceRepaint().
			  * \sa addTextMessage
			  */
			void clearTextMessages();

		}; // End of class def.
	} // End of namespace
} // End of namespace

#endif
