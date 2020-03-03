/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/opengl/CRenderizable.h>

/** The namespace for 3D scene representation and rendering. See also the <a
 * href="mrpt-opengl.html" > summary page</a> of the mrpt-opengl library for
 * more info and thumbnails of many of the render primitive.
 */
namespace mrpt::opengl
{
/** This class allows the user to create, load, save, and render 3D scenes using
 * OpenGL primitives.
 *  The class can be understood as a program to be run over OpenGL, containing
 * a sequence of viewport definitions,
 *   rendering primitives, etc...
 *
 *  It can contain from 1 up to any number of <b>Viewports</b>, each one
 *   associated a set of OpenGL objects and, optionally, a preferred camera
 * position. Both orthogonal (2D/3D) and projection
 *   camera models can be used for each viewport independently, greatly
 * increasing the possibilities of rendered scenes.
 *
 *  An object of COpenGLScene always contains at least one viewport
 * (utils::COpenGLViewport), named "main". Optionally, any
 *   number of other viewports may exist. Viewports are referenced by their
 * names, case-sensitive strings. Each viewport contains
 *   a different 3D scene (i.e. they render different objects), though a
 * mechanism exist to share the same 3D scene by a number of
 *   viewports so memory is not wasted replicating the same objects (see
 * COpenGLViewport::setCloneView ).
 *
 *  The main rendering method, COpenGLScene::render(), assumes a viewport has
 * been set-up for the entire target window. That
 *   method will internally make the required calls to opengl for creating the
 * additional viewports. Note that only the depth
 *   buffer is cleared by default for each (non-main) viewport, to allow
 * transparencies. This can be disabled by the approppriate
 *   member in COpenGLViewport.
 *
 *   An object COpenGLScene can be saved to a ".3Dscene" file using
 * CFileOutputStream, for posterior visualization from
 *    the standalone application <a
 * href="http://www.mrpt.org/Application:SceneViewer" >SceneViewer</a>.
 *    It can be also displayed in real-time using gui::CDisplayWindow3D.
 * \ingroup mrpt_opengl_grp
 */
class COpenGLScene : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(COpenGLScene, mrpt::opengl)
   public:
	/** Constructor
	 */
	COpenGLScene();

	/** Destructor:
	 */
	~COpenGLScene() override;

	/** Copy operator:
	 */
	COpenGLScene& operator=(const COpenGLScene& obj);

	/** Copy constructor:
	 */
	COpenGLScene(const COpenGLScene& obj);

	/**
	 * Inserts a set of objects into the scene, in the given viewport ("main"
	 * by default). Any iterable object will be accepted.
	 * \sa createViewport,getViewport
	 */
	template <class T>
	inline void insertCollection(
		const T& objs, const std::string& vpn = std::string("main"))
	{
		insert(objs.begin(), objs.end(), vpn);
	}
	/** Insert a new object into the scene, in the given viewport (by default,
	 * into the "main" viewport).
	 *  The viewport must be created previously, an exception will be raised if
	 * the given name does not correspond to
	 *   an existing viewport.
	 * \sa createViewport, getViewport
	 */
	void insert(
		const CRenderizable::Ptr& newObject,
		const std::string& viewportName = std::string("main"));

	/**
	 * Inserts a set of objects into the scene, in the given viewport ("main"
	 * by default).
	 * \sa createViewport,getViewport
	 */
	template <class T_it>
	inline void insert(
		const T_it& begin, const T_it& end,
		const std::string& vpn = std::string("main"))
	{
		for (T_it it = begin; it != end; it++) insert(*it, vpn);
	}

	/**Creates a new viewport, adding it to the scene and returning a pointer to
	 * the new object.
	 *  Names (case-sensitive) cannot be duplicated: if the name provided
	 * coincides with an already existing viewport, a pointer to the existing
	 * object will be returned.
	 *  The first, default viewport, is named "main".
	 */
	COpenGLViewport::Ptr createViewport(const std::string& viewportName);

	/** Returns the viewport with the given name, or nullptr if it does not
	 * exist; note that the default viewport is named "main" and initially
	 * occupies the entire rendering area.
	 */
	COpenGLViewport::Ptr getViewport(
		const std::string& viewportName = std::string("main")) const;

	/** Render this scene */
	void render() const;

	size_t viewportsCount() const { return m_viewports.size(); }
	/** Clear the list of objects and viewports in the scene, deleting objects'
	 * memory, and leaving just the default viewport with the default values.
	 */
	void clear(bool createMainViewport = true);

	/** If disabled (default), the SceneViewer application will ignore the
	 * camera of the "main" viewport and keep the viewport selected by the user
	 * by hand; otherwise, the camera in the "main" viewport prevails.
	 * \sa followCamera
	 */
	void enableFollowCamera(bool enabled) { m_followCamera = enabled; }
	/** Return the value of "followCamera"
	 * \sa enableFollowCamera
	 */
	bool followCamera() const { return m_followCamera; }
	/** Returns the first object with a given name, or nullptr (an empty smart
	 * pointer) if not found.
	 */
	CRenderizable::Ptr getByName(
		const std::string& str,
		const std::string& viewportName = std::string("main"));

	/** Returns the i'th object of a given class (or of a descendant class), or
	  nullptr (an empty smart pointer) if not found.
	  *  Example:
	  * \code
		   CSphere::Ptr obs = myscene.getByClass<CSphere>();
	  * \endcode
	  * By default (ith=0), the first observation is returned.
	  */
	template <typename T>
	typename T::Ptr getByClass(size_t ith = 0) const
	{
		MRPT_START
		for (const auto& m_viewport : m_viewports)
		{
			typename T::Ptr o = m_viewport->getByClass<T>(ith);
			if (o) return o;
		}
		return typename T::Ptr();  // Not found: return empty smart pointer
		MRPT_END
	}

	/** Removes the given object from the scene (it also deletes the object to
	 * free its memory).
	 */
	void removeObject(
		const CRenderizable::Ptr& obj,
		const std::string& viewportName = std::string("main"));

	/** Initializes all textures in the scene (See
	 * opengl::CTexturedPlane::loadTextureInOpenGL)
	 */
	void initializeAllTextures();

	/** Retrieves a list of all objects in text form.
	 */
	void dumpListOfObjects(std::vector<std::string>& lst);

	/** Saves the scene to a 3Dscene file, loadable by the application
	 * SceneViewer3D
	 * \sa loadFromFile
	 * \return false on any error.
	 */
	bool saveToFile(const std::string& fil) const;

	/** Loads the scene from a 3Dscene file, the format used by the application
	 * SceneViewer3D.
	 * \sa saveToFile
	 * \return false on any error.
	 */
	bool loadFromFile(const std::string& fil);

	/** Traces a ray
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const;

	/** Evaluates the bounding box of the scene in the given viewport (default:
	 * "main"). */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max,
		const std::string& vpn = std::string("main")) const;

	/** Recursive depth-first visit all objects in all viewports of the scene,
	 * calling the user-supplied function
	 *  The passed function must accept only one argument of type "const
	 * mrpt::opengl::CRenderizable::Ptr &"
	 */
	template <typename FUNCTOR>
	void visitAllObjects(FUNCTOR functor) const
	{
		MRPT_START
		for (const auto& m_viewport : m_viewports)
			for (auto itO = m_viewport->begin(); itO != m_viewport->end();
				 ++itO)
				internal_visitAllObjects(functor, *itO);
		MRPT_END
	}

	/** Ensure all shaders are unloaded in all viewports */
	void unloadShaders();

   protected:
	bool m_followCamera{false};

	using TListViewports = std::vector<COpenGLViewport::Ptr>;

	/** The list of viewports, indexed by name. */
	TListViewports m_viewports;

	template <typename FUNCTOR>
	static void internal_visitAllObjects(
		FUNCTOR functor, const CRenderizable::Ptr& o)
	{
		functor(o);
		if (IS_CLASS(*o, CSetOfObjects))
		{
			CSetOfObjects::Ptr obj =
				std::dynamic_pointer_cast<CSetOfObjects>(o);
			for (auto it = obj->begin(); it != obj->end(); ++it)
				internal_visitAllObjects(functor, *it);
		}
	}
};

/** Inserts an openGL object into a scene. Allows call chaining. \sa
 * mrpt::opengl::COpenGLScene::insert  */
inline COpenGLScene::Ptr& operator<<(
	COpenGLScene::Ptr& s, const CRenderizable::Ptr& r)
{
	s->insert(r);
	return s;
}
/**Inserts any iterable collection of openGL objects into a scene, allowing call
 * chaining.  \sa mrpt::opengl::COpenGLScene::insert */
template <class T>
inline COpenGLScene::Ptr& operator<<(
	COpenGLScene::Ptr& s, const std::vector<T>& v)
{
	s->insert(v.begin(), v.end());
	return s;
}
}  // namespace mrpt::opengl
