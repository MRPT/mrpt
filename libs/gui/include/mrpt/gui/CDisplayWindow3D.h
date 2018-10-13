/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/datetime.h>

#include <mutex>

namespace mrpt::gui
{
class C3DWindowDialog;
class CMyGLCanvas_DisplayWindow3D;

/** A graphical user interface (GUI) for efficiently rendering 3D scenes in
 * real-time.
 *  This class always contains internally an instance of opengl::COpenGLScene,
 * which
 *   the objects, viewports, etc. to be rendered.
 *
 *  Images can be grabbed automatically to disk for easy creation of videos.
 *  See CDisplayWindow3D::grabImagesStart  (and for creating videos,
 * mrpt::vision::CVideoFileWriter).
 *
 *  A short-cut for displaying 2D images (using the OpenGL rendering hardware)
 * is available
 *  through \a setImageView() and \a setImageView_fast(). Internally, these
 * methods call methods
 *  in the "main" viewport of the window (see \a COpenGLViewport).
 *
 *  Since the 3D rendering is performed in a detached thread, especial care
 * must be taken
 *   when updating the 3D scene to be rendered. The process involves an
 * internal critical section
 *   and it must always consist of these steps:
 *
 * \code
 *   mrpt::gui::CDisplayWindow3D	win("My window");
 *
 *   // Adquire the scene:
 *   mrpt::opengl::COpenGLScene::Ptr &ptrScene = win.get3DSceneAndLock();
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
 * An alternative way of updating the scene is by creating, before locking the
 * 3D window, a new object
 *  of class COpenGLScene, then locking the window only for replacing the smart
 * pointer. This may be
 *  advantageous is generating the 3D scene takes a long time, since while the
 * window
 *  is locked it will not be responsive to the user input or window redraw.
 *
 * It is safer against exceptions to use the auxiliary class
 * CDisplayWindow3DLocker.
 * \code
 *   mrpt::gui::CDisplayWindow3D	win("My window");
 *   // ...
 *   { // The scene is adquired in this scope
 *      mrpt::opengl::COpenGLScene::Ptr ptrScene;
 *      mrpt::gui::CDisplayWindow3DLocker  locker(win,ptrScene);
 *      //...
 *
 *   } // scene is unlocked upon dtor of `locker`
 * \endcode
 *
 * Notice however that a copy of the smart pointer is made, so replacement of
 * the entire scene
 * via `operator =` is not possible if using this method. Still, in general it
 * should be preferred because
 * the mutexes are automatically released in case of unexpected exceptions.
 *
 * The window can also display a set of 2D text messages overlapped to the 3D
 * scene.
 *  See CDisplayWindow3D::addTextMessage
 *
 *  For a list of supported events with the observer/observable pattern, see
 * the discussion in mrpt::gui::CBaseGUIWindow.
 *  In addition to those events, this class introduces
 * mrpt::gui::mrptEvent3DWindowGrabImageFile
 *
 * ** CDisplayWindow3D mouse view navigation cheatsheet **
 *  - <b>Orbit camera</b>: Left-button pressed + move
 *  - <b>Zoom in / out</b>:
 *    - Mouse scroll wheel, or
 *    - SHIFT+Left-button pressed + move up/down
 *  - <b>Look around (pivot camera)</b>: CTRL+Left-button pressed + move
 * up/down
 *  - <b>Pan (XY plane)</b>: Right-button pressed + move
 *  - <b>Move camera along Z axis</b>: SHIFT+Left-button pressed + move
 * left/right
 *
 * \sa  The example /samples/display3D, the <a
 * href="http://www.mrpt.org/Tutorial_3D_Scenes" > tutorial only</a>.
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindow3D : public mrpt::gui::CBaseGUIWindow
{
   public:
	using Ptr = std::shared_ptr<CDisplayWindow3D>;
	using ConstPtr = std::shared_ptr<const CDisplayWindow3D>;

   protected:
	friend class C3DWindowDialog;
	friend class CMyGLCanvas_DisplayWindow3D;

	/** Internal OpenGL object (see general discussion in about usage of this
	 * object) */
	mrpt::opengl::COpenGLScene::Ptr m_3Dscene;
	/** Critical section for accesing m_3Dscene */
	mutable std::recursive_mutex m_csAccess3DScene;

	/** Throws an exception on initialization error */
	void createOpenGLContext();

	mrpt::void_ptr_noncopy m_DisplayDeviceContext;
	mrpt::void_ptr_noncopy m_GLRenderingContext;

	std::string m_grab_imgs_prefix;
	unsigned int m_grab_imgs_idx{0};

	bool m_is_capturing_imgs{false};
	mrpt::img::CImage::Ptr m_last_captured_img;
	mutable std::mutex m_last_captured_img_cs;

	void doRender();

	mrpt::system::TTimeStamp m_lastFullScreen;

	/** \sa getRenderingFPS */
	double m_last_FPS{10};

	void internalSetMinMaxRange();

   public:
	/** Constructor */
	CDisplayWindow3D(
		const std::string& windowCaption = std::string(),
		unsigned int initialWindowWidth = 400,
		unsigned int initialWindowHeight = 300);

	/** Class factory returning a smart pointer */
	static CDisplayWindow3D::Ptr Create(
		const std::string& windowCaption, unsigned int initialWindowWidth = 400,
		unsigned int initialWindowHeight = 300);

	/** Destructor */
	~CDisplayWindow3D() override;

	/** Gets a reference to the smart shared pointer that holds the internal
	 * scene (carefuly read introduction in gui::CDisplayWindow3D before use!)
	 *  This also locks the critical section for accesing the scene, thus the
	 * window will not be repainted until it is unlocked.
	 * \note It is safer to use mrpt::gui::CDisplayWindow3DLocker instead.*/
	mrpt::opengl::COpenGLScene::Ptr& get3DSceneAndLock();

	/** Unlocks the access to the internal 3D scene. It is safer to use
	 * mrpt::gui::CDisplayWindow3DLocker instead.
	 *  Typically user will want to call forceRepaint after updating the scene.
	 */
	void unlockAccess3DScene();

	/** Repaints the window. forceRepaint, repaint and updateWindow are all
	 * aliases of the same method */
	void forceRepaint();
	/** Repaints the window. forceRepaint, repaint and updateWindow are all
	 * aliases of the same method */
	void repaint() { forceRepaint(); }
	/** Repaints the window. forceRepaint, repaint and updateWindow are all
	 * aliases of the same method */
	void updateWindow() { forceRepaint(); }
	/** Return the camera field of view (in degrees) (used for gluPerspective)
	 */
	float getFOV() const;
	/** Changes the camera min clip range (z) (used for gluPerspective). The
	 * window is not updated with this method, call "forceRepaint" to update the
	 * 3D view. */
	void setMinRange(double new_min);
	/** Changes the camera max clip range (z) (used for gluPerspective. The
	 * window is not updated with this method, call "forceRepaint" to update the
	 * 3D view. */
	void setMaxRange(double new_max);
	/** Changes the camera field of view (in degrees) (used for gluPerspective).
	 * The window is not updated with this method, call "forceRepaint" to update
	 * the 3D view. */
	void setFOV(float v);
	/** Resizes the window, stretching the image to fit into the display area.
	 */
	void resize(unsigned int width, unsigned int height) override;
	/** Changes the position of the window on the screen. */
	void setPos(int x, int y) override;
	/**  Changes the window title. */
	void setWindowTitle(const std::string& str) override;
	/** Changes the camera parameters programmatically */
	void setCameraElevationDeg(float deg);
	/** Changes the camera parameters programmatically */
	void setCameraAzimuthDeg(float deg);
	/** Changes the camera parameters programmatically */
	void setCameraPointingToPoint(float x, float y, float z);
	/** Changes the camera parameters programmatically */
	void setCameraZoom(float zoom);
	/** Sets the camera as projective, or orthogonal. */
	void setCameraProjective(bool isProjective);
	/** Get camera parameters programmatically */
	float getCameraElevationDeg() const;
	/** Get camera parameters programmatically */
	float getCameraAzimuthDeg() const;
	/** Get camera parameters programmatically */
	void getCameraPointingToPoint(float& x, float& y, float& z) const;
	/** Get camera parameters programmatically */
	float getCameraZoom() const;
	/** Sets the camera as projective, or orthogonal */
	bool isCameraProjective() const;
	/** If set to true (default = false), the mouse-based scene navigation will
	 * be disabled and the camera position will be determined by the opengl
	 * viewports in the 3D scene */
	void useCameraFromScene(bool useIt = true);
	/** Gets the 3D ray for the direction line of the pixel where the mouse
	 * cursor is at. \return False if the window is closed. \sa
	 * getLastMousePosition */
	bool getLastMousePositionRay(mrpt::math::TLine3D& ray) const;
	/** Gets the last x,y pixel coordinates of the mouse. \return False if the
	 * window is closed. \sa getLastMousePositionRay */
	bool getLastMousePosition(int& x, int& y) const override;
	/** Set cursor style to default (cursorIsCross=false) or to a cross
	 * (cursorIsCross=true) \sa getLastMousePositionRay */
	void setCursorCross(bool cursorIsCross) override;

	/** Start to save rendered images to disk.
	 *  Images will be saved independently as png files, depending on
	 *   the template path passed to this method. For example:
	 *
	 *  path_prefix: "./video_"
	 *
	 *  Will generate "./video_000001.png", etc.
	 *
	 *  If this feature is enabled, the window will emit events of the type
	 * mrpt::gui::mrptEvent3DWindowGrabImageFile() which you can subscribe to.
	 *
	 *  \sa grabImagesStop
	 */
	void grabImagesStart(
		const std::string& grab_imgs_prefix = std::string("video_"));

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
	 * \return false if there was no time yet for grabbing any image (then, the
	 * output image is undefined).
	 * \sa captureImagesStart, getLastWindowImagePtr
	 */
	bool getLastWindowImage(mrpt::img::CImage& out_img) const;

	/** Retrieve the last captured image from the window, as a smart pointer.
	 *  This method is more efficient than getLastWindowImage since only a copy
	 * of the pointer is performed, while
	 *   getLastWindowImage would copy the entire image.
	 *
	 *  You MUST CALL FIRST captureImagesStart to enable image grabbing.
	 * \Note If there was no time yet for grabbing any image, an empty smart
	 * pointer will be returned.
	 * \sa captureImagesStart, getLastWindowImage
	 */
	mrpt::img::CImage::Ptr getLastWindowImagePtr() const;

	/** Increments by one the image counter and return the next image file name
	 * (Users normally don't want to call this method).
	 * \sa grabImagesStart
	 */
	std::string grabImageGetNextFile();

	bool isCapturingImgs() const { return m_is_capturing_imgs; }
	/** Add 2D text messages overlapped to the 3D rendered scene. The string
	 * will remain displayed in the 3D window
	 *   until it's changed with subsequent calls to this same method, or all
	 * the texts are cleared with clearTextMessages().
	 *
	 *  \param x The X position, interpreted as absolute pixels from the left
	 * if X>=1, absolute pixels from the left if X<0 or as a width factor if in
	 * the range [0,1[.
	 *  \param y The Y position, interpreted as absolute pixels from the bottom
	 * if Y>=1, absolute pixels from the top if Y<0 or as a height factor if in
	 * the range [0,1[.
	 *  \param text The text string to display.
	 *  \param color The text color. For example: TColorf(1.0,1.0,1.0)
	 *  \param unique_index An "index" for this text message, so that
	 * subsequent calls with the same index will overwrite this text message
	 * instead of creating new ones.
	 *
	 *  You'll need to refresh the display manually with forceRepaint().
	 *
	 * \sa clearTextMessages
	 */
	void addTextMessage(
		const double x, const double y, const std::string& text,
		const mrpt::img::TColorf& color = mrpt::img::TColorf(1.0, 1.0, 1.0),
		const size_t unique_index = 0,
		const mrpt::opengl::TOpenGLFont font =
			mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24);

	/** overload with more font parameters - refer to
	 * mrpt::opengl::gl_utils::glDrawText()
	 *  Available fonts are enumerated at mrpt::opengl::gl_utils::glSetFont() */
	void addTextMessage(
		const double x_frac, const double y_frac, const std::string& text,
		const mrpt::img::TColorf& color, const std::string& font_name,
		const double font_size,
		const mrpt::opengl::TOpenGLFontStyle font_style = mrpt::opengl::NICE,
		const size_t unique_index = 0, const double font_spacing = 1.5,
		const double font_kerning = 0.1, const bool draw_shadow = false,
		const mrpt::img::TColorf& shadow_color = mrpt::img::TColorf(0, 0, 0));

	/**  Clear all text messages created with addTextMessage().
	 *  You'll need to refresh the display manually with forceRepaint().
	 * \sa addTextMessage
	 */
	void clearTextMessages();

	/** Get the average Frames Per Second (FPS) value from the last 250
	 * rendering events */
	double getRenderingFPS() const { return m_last_FPS; }
	/** A short cut for getting the "main" viewport of the scene object, it is
	 * equivalent to:
	 *  \code
	 *    mrpt::opengl::COpenGLScene::Ptr &scene = win3D.get3DSceneAndLock();
	 *    viewport = scene->getViewport("main");
	 *    win3D.unlockAccess3DScene();
	 *  \endcode
	 */
	mrpt::opengl::COpenGLViewport::Ptr getDefaultViewport();

	/** Set the "main" viewport into "image view"-mode, where an image is
	 * efficiently drawn (fitting the viewport area) using an OpenGL textured
	 * quad.
	 *  Call this method with the new image to update the displayed image (but
	 * recall to first lock the parent openglscene's critical section, then do
	 * the update, then release the lock, and then issue a window repaint).
	 *  Internally, the texture is drawn using a mrpt::opengl::CTexturedPlane
	 *  The viewport can be reverted to behave like a normal viewport by
	 * calling setNormalMode()
	 * \sa setImageView_fast, COpenGLViewport
	 * \note This method already locks/unlocks the 3D scene of the window, so
	 * the user must NOT call get3DSceneAndLock() / unlockAccess3DScene()
	 * before/after calling it.
	 */
	void setImageView(const mrpt::img::CImage& img);

	/** Just like \a setImageView but moves the internal image memory instead of
	 * making a copy, so it's faster but empties the input image.
	 * \sa setImageView, COpenGLViewport
	 * \note This method already locks/unlocks the 3D scene of the window, so
	 * the user must NOT call get3DSceneAndLock() / unlockAccess3DScene()
	 * before/after calling it.
	 */
	void setImageView_fast(mrpt::img::CImage& img);

   protected:
	/** Set the rendering FPS (users don't call this, the method is for internal
	 * MRPT objects only) \sa getRenderingFPS */
	void internal_setRenderingFPS(double FPS);
	/** called by CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers */
	void internal_emitGrabImageEvent(const std::string& fil);

};  // End of class def.

/** @name Events specific to CDisplayWindow3D
	@{  */

/**  An event sent by a CDisplayWindow3D window when an image is saved after
 * enabling this feature with CDisplayWindow3D::grabImagesStart()
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will be invoked
 * from the wxWidgets internal MRPT thread,
 *    so all your code in the handler must be thread safe.
 */
class mrptEvent3DWindowGrabImageFile : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEvent3DWindowGrabImageFile(
		CDisplayWindow3D* obj, const std::string& _img_file)
		: source_object(obj), img_file(_img_file)
	{
	}

	CDisplayWindow3D* source_object;
	/** The absolute path of the file that has been just saved. */
	const std::string& img_file;
};  // End of class def.

/** @} */

/** Auxiliary class for safely claiming the 3DScene of a
 * mrpt::gui::CDisplayWindow3D.
 * The mutex will be hold between ctor and dtor calls of objects of this class,
 * safely releasing
 * the lock upon exceptions. See example usage code in docs of
 * mrpt::gui::CDisplayWindow3D
 *
 * \ingroup mrpt_gui_grp
 * \note New in MRPT 1.5.0
 */
class CDisplayWindow3DLocker
{
   public:
	/** Acquires the lock of the 3D scene of the referenced window, and returns
	 * a copy of the smart pointer to it. */
	CDisplayWindow3DLocker(
		CDisplayWindow3D& win, mrpt::opengl::COpenGLScene::Ptr& out_scene_ptr);
	/** Acquires the lock of the 3D scene of the referenced window. Use this
	 * signature when the scene object is not required. */
	CDisplayWindow3DLocker(CDisplayWindow3D& win);
	~CDisplayWindow3DLocker();

   private:
	CDisplayWindow3D& m_win;
};

}  // namespace mrpt::gui
