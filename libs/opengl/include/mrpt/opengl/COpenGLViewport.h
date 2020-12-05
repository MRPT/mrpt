/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTextMessageCapable.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/CObservable.h>
#include <mrpt/system/mrptEvent.h>
#include <map>

namespace mrpt::img
{
class CImage;
}
namespace mrpt::opengl
{
/** A viewport within a COpenGLScene, containing a set of OpenGL objects to
 *render.
 *   This class has protected constuctor, thus it cannot be created by users.
 *Use COpenGLScene::createViewport instead.
 *  A viewport has these "operation modes":
 *		- Normal (default): It renders the contained objects.
 *		- Cloned: It clones the objects from another viewport. See \a
 *setCloneView()
 *		- Image mode: It renders an image (e.g. from a video stream) efficiently
 *using a textued quad. See \a setImageView().
 *
 * In any case, the viewport can be resized to only fit a part of the entire
 *parent viewport.
 *  There will be always at least one viewport in a COpenGLScene named "main".
 *
 * This class can be observed (see mrpt::system::CObserver) for the following
 *events (see mrpt::system::mrptEvent):
 *   - mrpt::opengl::mrptEventGLPreRender
 *   - mrpt::opengl::mrptEventGLPostRender
 *
 * Two directional light sources at infinity are created by default, with
 *directions (-1,-1,-1) and (1,2,1), respectively.
 *
 * Lighting parameters are accessible via lightParameters().
 *
 *  Refer to mrpt::opengl::COpenGLScene for further details.
 * \ingroup mrpt_opengl_grp
 */
class COpenGLViewport : public mrpt::serialization::CSerializable,
						public mrpt::system::CObservable,
						public mrpt::opengl::CTextMessageCapable
{
	DEFINE_SERIALIZABLE(COpenGLViewport, mrpt::opengl)
	friend class COpenGLScene;

   public:
	/** @name Viewport "modes"
		@{ */

	/** Set this viewport as a clone of some other viewport, given its name - as
	 * a side effect, current list of internal OpenGL objects is cleared.
	 *  By default, only the objects are cloned, not the camera. See
	 * \sa resetCloneView
	 */
	void setCloneView(const std::string& clonedViewport);

	/** Set this viewport into "image view"-mode, where an image is efficiently
	 * drawn (fitting the viewport area) using an OpenGL textured quad.
	 *  Call this method with the new image to update the displayed image (but
	 * recall to first lock the parent openglscene's critical section, then do
	 * the update, then release the lock, and then issue a window repaint).
	 *  Internally, the texture is drawn using a mrpt::opengl::CTexturedPlane
	 *  The viewport can be reverted to behave like a normal viewport by
	 * calling setNormalMode()
	 */
	void setImageView(const mrpt::img::CImage& img);

	/** Just like \a setImageView but moves the internal image memory instead of
	 * making a copy, so it's faster but empties the input image.
	 * \sa setImageView
	 */
	void setImageView(mrpt::img::CImage&& img);

	/** Returns true if setImageView() has been called on this viewport */
	bool isImageViewMode() const { return !!m_imageViewPlane; }

	/** Reset the viewport to normal mode: rendering its own objects.
	 * \sa setCloneView, setNormalMode
	 */
	inline void resetCloneView() { setNormalMode(); }
	/** If set to true, and setCloneView() has been called, this viewport will
	 * be rendered using the camera of the cloned viewport.
	 */
	inline void setCloneCamera(bool enable) { m_isClonedCamera = enable; }
	/** Resets the viewport to a normal 3D viewport \sa setCloneView,
	 * setImageView */
	void setNormalMode();

	/** @} */
	// end of Set the "viewport mode"

	/** @name OpenGL global settings that affect rendering all objects in the
	   scene/viewport
		@{ */

	/** Sets glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST) is enabled, or GL_FASTEST
	 * otherwise. */
	void enablePolygonNicest(bool enable = true)
	{
		m_OpenGL_enablePolygonNicest = enable;
	}
	bool isPolygonNicestEnabled() const { return m_OpenGL_enablePolygonNicest; }

	const TLightParameters& lightParameters() const { return m_lights; }
	TLightParameters& lightParameters() { return m_lights; }

	/** @} */

	/** @name Change or read viewport properties (except "viewport modes")
		@{ */

	/** Returns the name of the viewport */
	inline std::string getName() { return m_name; }
	/** Change the viewport position and dimension on the rendering window.
	 *  X & Y coordinates here can have two interpretations:
	 *    - If in the range [0,1], they are factors with respect to the actual
	 *window sizes (i.e. width=1 means the entire width of the rendering
	 *window).
	 *    - If >1, they are interpreted as pixels.
	 *
	 *  width & height can be interpreted as:
	 *		- If >1, they are the size of the viewport in that dimension, in
	 *pixels.
	 *		- If in [0,1], they are the size of the viewport in that dimension,
	 *in
	 *a factor of the width/height.
	 *		- If in [-1,0[, the size is computed such as the right/top border
	 *ends
	 *up in the given coordinate, interpreted as a factor (e.g. -1: up to the
	 *end of the viewport, -0.5: up to the middle of it).
	 *		- If <-1 the size is computed such as the right/top border ends up
	 *in
	 *the given absolute coordinate (e.g. -200: up to the row/column 200px).
	 *
	 * \note (x,y) specify the lower left corner of the viewport rectangle.
	 * \sa getViewportPosition
	 */
	void setViewportPosition(
		const double x, const double y, const double width,
		const double height);

	/** Get the current viewport position and dimension on the rendering window.
	 *  X & Y coordinates here can have two interpretations:
	 *    - If in the range [0,1], they are factors with respect to the actual
	 * window sizes (i.e. width=1 means the entire width of the rendering
	 * window).
	 *    - If >1, they are interpreted as pixels.
	 * \note (x,y) specify the lower left corner of the viewport rectangle.
	 * \sa setViewportPosition
	 */
	void getViewportPosition(
		double& x, double& y, double& width, double& height);

	/** Set the min/max clip depth distances of the rendering frustum (default:
	 * 0.1 - 10000)
	 * \sa getViewportClipDistances
	 */
	void setViewportClipDistances(const float clip_min, const float clip_max);

	/** Get the current min/max clip depth distances of the rendering frustum
	 * (default: 0.1 - 10000)
	 * \sa setViewportClipDistances
	 */
	void getViewportClipDistances(float& clip_min, float& clip_max) const;

	/** Set the border size ("frame") of the viewport (default=0) */
	inline void setBorderSize(unsigned int lineWidth)
	{
		m_borderWidth = lineWidth;
	}
	inline unsigned int getBorderSize() const { return m_borderWidth; }

	void setBorderColor(const mrpt::img::TColor& c) { m_borderColor = c; }
	const mrpt::img::TColor& getBorderColor() const { return m_borderColor; }

	/** Return whether the viewport will be rendered transparent over previous
	 * viewports.
	 */
	inline bool isTransparent() { return m_isTransparent; }
	/** Set the transparency, that is, whether the viewport will be rendered
	 * transparent over previous viewports (default=false).
	 */
	inline void setTransparent(bool trans) { m_isTransparent = trans; }
	/** Set a background color different from that of the parent GUI window */
	inline void setCustomBackgroundColor(const mrpt::img::TColorf& color)
	{
		m_custom_backgb_color = true;
		m_background_color = color;
	}

	inline mrpt::img::TColorf getCustomBackgroundColor() const
	{
		return m_background_color;
	}

	/** Compute the 3D ray corresponding to a given pixel; this can be used to
	 * allow the user to pick and select 3D objects by clicking onto the 2D
	 * image.
	 *  \param x_coord Horizontal coordinate with the usual meaning (0:left of
	 * the viewport, W-1: right border).
	 *  \param y_coord Horizontal coordinate with the usual meaning (0:top of
	 * the viewport, H-1: right border).
	 * \param out_cameraPose If not nullptr, will have the camera 3D pose as a
	 * mrpt::poses::CPose3D. See also
	 * \note (x,y) refer to VIEWPORT coordinates. Take into account this when
	 * viewports do not extend to the whole window size.
	 * \note x and y are double instead of integers to allow sub-pixel
	 * precision.
	 * \sa getCurrentCameraPose
	 */
	void get3DRayForPixelCoord(
		const double x_coord, const double y_coord,
		mrpt::math::TLine3D& out_ray,
		mrpt::poses::CPose3D* out_cameraPose = nullptr) const;

	/** @} */  // end of Change or read viewport properties

	/** @name Contained objects set/get/search
		@{ */

	using const_iterator = CListOpenGLObjects::const_iterator;
	using iterator = CListOpenGLObjects::iterator;

	inline const_iterator begin() const { return m_objects.begin(); }
	inline const_iterator end() const { return m_objects.end(); }
	inline iterator begin() { return m_objects.begin(); }
	inline iterator end() { return m_objects.end(); }
	/** Delete all internal obejcts
	 * \sa insert */
	void clear();

	/** Insert a new object into the list.
	 *  The object MUST NOT be deleted, it will be deleted automatically by
	 * this object when not required anymore.
	 */
	void insert(const CRenderizable::Ptr& newObject);

	/** Compute the current 3D camera pose.
	 * \sa get3DRayForPixelCoord
	 */
	void getCurrentCameraPose(mrpt::poses::CPose3D& out_cameraPose) const;

	/** Changes the point of view of the camera, from a given pose.
	 * \sa getCurrentCameraPose
	 */
	void setCurrentCameraFromPose(mrpt::poses::CPose3D& p);

	/** Returns the first object with a given name, or nullptr if not found.
	 */
	CRenderizable::Ptr getByName(const std::string& str);

	/** Returns the i'th object of a given class (or of a descendant class), or
	  nullptr (an empty smart pointer) if not found.
	  *  Example:
	  * \code
		   CSphere::Ptr obs = view.getByClass<CSphere>();
	  * \endcode
	  * By default (ith=0), the first observation is returned.
	  */
	template <typename T>
	typename T::Ptr getByClass(size_t ith = 0) const
	{
		MRPT_START
		size_t foundCount = 0;
		const auto* class_ID = &T::GetRuntimeClassIdStatic();
		for (const auto& o : m_objects)
			if (o && o->GetRuntimeClass()->derivedFrom(class_ID))
				if (foundCount++ == ith) return std::dynamic_pointer_cast<T>(o);

		// If not found directly, search recursively:
		for (const auto& o : m_objects)
		{
			if (o && o->GetRuntimeClass() ==
						 CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
			{
				typename T::Ptr obj = std::dynamic_pointer_cast<T>(
					std::dynamic_pointer_cast<CSetOfObjects>(o)
						->template getByClass<T>(ith));
				if (obj) return obj;
			}
		}
		return typename T::Ptr();  // Not found: return empty smart pointer
		MRPT_END
	}

	/** Removes the given object from the scene (it also deletes the object to
	 * free its memory).
	 */
	void removeObject(const CRenderizable::Ptr& obj);

	/** Number of objects contained. */
	inline size_t size() const { return m_objects.size(); }
	inline bool empty() const { return m_objects.empty(); }
	/** Get a reference to the camera associated with this viewport. */
	opengl::CCamera& getCamera() { return m_camera; }
	/** Get a reference to the camera associated with this viewport. */
	const opengl::CCamera& getCamera() const { return m_camera; }
	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const;

	/** @} */  // end of Contained objects set/get/search

	/** Destructor: clears all objects. */
	~COpenGLViewport() override;

	/** Constructor, invoked from COpenGLScene only.
	 */
	COpenGLViewport(
		COpenGLScene* parent = nullptr,
		const std::string& name = std::string(""));

	/** Render the objects in this viewport (called from COpenGLScene) */
	void render(
		const int render_width, const int render_height,
		const int render_offset_x = 0, const int render_offset_y = 0) const;

   protected:
	/** Initializes all textures in the scene (See
	 * opengl::CTexturedPlane::initializeTextures)
	 */
	void initializeTextures();

	/** Retrieves a list of all objects in text form.
	 */
	void dumpListOfObjects(std::vector<std::string>& lst);

	/** Render in image mode */
	void renderImageMode() const;

	/** Render a normal scene with 3D objects */
	void renderNormalSceneMode() const;

	/** Render the viewport border, if enabled */
	void renderViewportBorder() const;

	/** The camera associated to the viewport */
	opengl::CCamera m_camera;
	/** The scene that contains this viewport. */
	mrpt::safe_ptr<COpenGLScene> m_parent;
	/** Set by setCloneView */
	bool m_isCloned{false};
	/** Set by setCloneCamera */
	bool m_isClonedCamera{false};
	/** Only if m_isCloned=true */
	std::string m_clonedViewport;
	/** The viewport's name */
	std::string m_name;
	/** Whether to clear color buffer. */
	bool m_isTransparent{false};
	/** Default=0, the border around the viewport. */
	uint32_t m_borderWidth{0};

	mrpt::img::TColor m_borderColor{255, 255, 255, 255};

	/** The viewport position [0,1] */
	double m_view_x{0}, m_view_y{0}, m_view_width{1}, m_view_height{1};
	/** The min/max clip depth distances (default: 0.1 - 10000) */
	float m_clip_min = 0.1f, m_clip_max = 10000.0f;
	bool m_custom_backgb_color{false};
	/** used only if m_custom_backgb_color */
	mrpt::img::TColorf m_background_color = {0.6f, 0.6f, 0.6f};

	/** The image to display, after calling \a setImageView() */
	mrpt::opengl::CTexturedPlane::Ptr m_imageViewPlane;

	mutable mrpt::opengl::CSetOfLines::Ptr m_borderLines;

	/** Info updated with each "render()" and used in "get3DRayForPixelCoord" */
	mutable TRenderMatrices m_state;

	/** Default shader program */
	mutable std::map<shader_id_t, mrpt::opengl::Program::Ptr> m_shaders;

	/** Load all MPRT predefined shader programs into m_shaders */
	void loadDefaultShaders() const;

	/** Unload shader programs in m_shaders */
	void unloadShaders();

	/** The list of objects that comprise the 3D scene.
	 *  Objects are automatically deleted when calling "clear" or in the
	 * destructor.
	 */
	opengl::CListOpenGLObjects m_objects;

	void internal_enableImageView();

	// OpenGL global settings:
	bool m_OpenGL_enablePolygonNicest{true};

	TLightParameters m_lights;

	/** Renders all messages in the underlying class CTextMessageCapable */
	void renderTextMessages() const;
};

/**
 * Inserts an openGL object into a viewport. Allows call chaining.
 * \sa mrpt::opengl::COpenGLViewport::insert
 */
inline COpenGLViewport::Ptr& operator<<(
	COpenGLViewport::Ptr& s, const CRenderizable::Ptr& r)
{
	s->insert(r);
	return s;
}
/**
 * Inserts any iterable set of openGL objects into a viewport. Allows call
 * chaining.
 * \sa mrpt::opengl::COpenGLViewport::insert
 */
inline COpenGLViewport::Ptr& operator<<(
	COpenGLViewport::Ptr& s, const std::vector<CRenderizable::Ptr>& v)
{
	for (const auto& it : v) s->insert(it);
	return s;
}

/** @name Events emitted by COpenGLViewport
	@{  */

/**  An event sent by an mrpt::opengl::COpenGLViewport just after clearing the
 * viewport and setting the GL_PROJECTION matrix, and before calling the scene
 * OpenGL drawing primitives.
 *
 * While handling this event you can call OpenGL glDraw(), etc.
 *
 * IMPORTANTE NOTICE: Event handlers in your observer class will most likely be
 * invoked from an internal GUI thread of MRPT, so all your code in the handler
 * must be thread safe.
 */
class mrptEventGLPreRender : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventGLPreRender(const COpenGLViewport* obj)
		: source_viewport(obj)
	{
	}
	const COpenGLViewport* const source_viewport;
};  // End of class def.

/**  An event sent by an mrpt::opengl::COpenGLViewport after calling the scene
 * OpenGL drawing primitives and before doing a glSwapBuffers
 *
 *  While handling this event you can call OpenGL glBegin(),glEnd(),gl*
 * functions or those in mrpt::opengl::gl_utils to draw stuff *on the top* of
 * the normal
 *   objects contained in the COpenGLScene.
 *
 *  IMPORTANTE NOTICE: Event handlers in your observer class will most likely
 * be invoked from an internal GUI thread of MRPT,
 *    so all your code in the handler must be thread safe.
 */
class mrptEventGLPostRender : public mrpt::system::mrptEvent
{
   protected:
	/** Just to allow this class to be polymorphic */
	void do_nothing() override {}

   public:
	inline mrptEventGLPostRender(const COpenGLViewport* obj)
		: source_viewport(obj)
	{
	}
	const COpenGLViewport* const source_viewport;
};  // End of class def.

/** @} */

}  // namespace mrpt::opengl
