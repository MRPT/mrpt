/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_COpenGLScene_H
#define opengl_COpenGLScene_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/utils/CStringList.h>

namespace mrpt
{
	/** The namespace for 3D scene representation and rendering. See also the <a href="mrpt-opengl.html" > summary page</a> of the mrpt-opengl library for more info and thumbnails of many of the render primitive.
	  */
	namespace opengl
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COpenGLScene, mrpt::utils::CSerializable, OPENGL_IMPEXP )


		/** This class allows the user to create, load, save, and render 3D scenes using OpenGL primitives.
		  *  The class can be understood as a program to be run over OpenGL, containing a sequence of viewport definitions,
		  *   rendering primitives, etc...
		  *
		  *  It can contain from 1 up to any number of <b>Viewports</b>, each one
		  *   associated a set of OpenGL objects and, optionally, a preferred camera position. Both orthogonal (2D/3D) and projection
		  *   camera models can be used for each viewport independently, greatly increasing the possibilities of rendered scenes.
		  *
		  *  An object of COpenGLScene always contains at least one viewport (utils::COpenGLViewport), named "main". Optionally, any
		  *   number of other viewports may exist. Viewports are referenced by their names, case-sensitive strings. Each viewport contains
		  *   a different 3D scene (i.e. they render different objects), though a mechanism exist to share the same 3D scene by a number of
		  *   viewports so memory is not wasted replicating the same objects (see COpenGLViewport::setCloneView ).
		  *
		  *  The main rendering method, COpenGLScene::render(), assumes a viewport has been set-up for the entire target window. That
		  *   method will internally make the required calls to opengl for creating the additional viewports. Note that only the depth
		  *   buffer is cleared by default for each (non-main) viewport, to allow transparencies. This can be disabled by the approppriate
		  *   member in COpenGLViewport.
		  *
		  *   An object COpenGLScene can be saved to a ".3Dscene" file using CFileOutputStream, for posterior visualization from
		  *    the standalone application <a href="http://www.mrpt.org/Application:SceneViewer" >SceneViewer</a>.
		  *    It can be also displayed in real-time using gui::CDisplayWindow3D.
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP COpenGLScene : public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( COpenGLScene )
		public:
			/** Constructor
			  */
			COpenGLScene();

			/** Destructor:
			 */
			virtual ~COpenGLScene();

			/** Copy operator:
			  */
			COpenGLScene & operator =( const COpenGLScene &obj );

			/** Copy constructor:
			  */
			COpenGLScene( const COpenGLScene &obj );

			/**
			  * Inserts a set of objects into the scene, in the given viewport ("main" by default). Any iterable object will be accepted.
			  * \sa createViewport,getViewport
			  */
			template<class T> inline void insertCollection(const T &objs,const std::string &vpn=std::string("main"))	{
				insert(objs.begin(),objs.end(),vpn);
			}
			/** Insert a new object into the scene, in the given viewport (by default, into the "main" viewport).
			  *  The viewport must be created previously, an exception will be raised if the given name does not correspond to
			  *   an existing viewport.
			  * \sa createViewport, getViewport
			  */
			void insert( const CRenderizablePtr &newObject, const std::string &viewportName=std::string("main"));

			/**
			  * Inserts a set of objects into the scene, in the given viewport ("main" by default).
			  * \sa createViewport,getViewport
			  */
			template<class T_it> inline void insert(const T_it &begin,const T_it &end,const std::string &vpn=std::string("main"))	{
				for (T_it it=begin;it!=end;it++) insert(*it,vpn);
			}

			/**Creates a new viewport, adding it to the scene and returning a pointer to the new object.
			  *  Names (case-sensitive) cannot be duplicated: if the name provided coincides with an already existing viewport, a pointer to the existing object will be returned.
			  *  The first, default viewport, is named "main".
			  */
			COpenGLViewportPtr createViewport( const std::string &viewportName );

			/** Returns the viewport with the given name, or NULL if it does not exist; note that the default viewport is named "main" and initially occupies the entire rendering area.
			  */
			COpenGLViewportPtr getViewport( const std::string &viewportName = std::string("main") ) const;

			/** Render this scene */
			void render() const;

			size_t  viewportsCount() const { return m_viewports.size(); }

			/** Clear the list of objects and viewports in the scene, deleting objects' memory, and leaving just the default viewport with the default values.
			  */
			void  clear( bool createMainViewport = true );

			/** If disabled (default), the SceneViewer application will ignore the camera of the "main" viewport and keep the viewport selected by the user by hand; otherwise, the camera in the "main" viewport prevails.
			  * \sa followCamera
			  */
			void enableFollowCamera( bool enabled ) { m_followCamera = enabled; }

			/** Return the value of "followCamera"
			  * \sa enableFollowCamera
			  */
			bool followCamera() const { return m_followCamera; }

			/** Returns the first object with a given name, or NULL (an empty smart pointer) if not found.
			  */
			CRenderizablePtr	getByName( const std::string &str, const std::string &viewportName = std::string("main") );

			 /** Returns the i'th object of a given class (or of a descendant class), or NULL (an empty smart pointer) if not found.
			   *  Example:
			   * \code
					CSpherePtr obs = myscene.getByClass<CSphere>();
			   * \endcode
			   * By default (ith=0), the first observation is returned.
			   */
			 template <typename T>
			 typename T::SmartPtr getByClass( const size_t &ith = 0 ) const
			 {
				MRPT_START
				for (TListViewports::const_iterator it = m_viewports.begin();it!=m_viewports.end();++it)
				{
					typename T::SmartPtr o = (*it)->getByClass<T>(ith);
					if (o.present()) return o;
				}
				return typename T::SmartPtr();	// Not found: return empty smart pointer
				MRPT_END
			 }


			/** Removes the given object from the scene (it also deletes the object to free its memory).
			  */
			void removeObject( const CRenderizablePtr &obj, const std::string &viewportName = std::string("main") );

			/** Initializes all textures in the scene (See opengl::CTexturedPlane::loadTextureInOpenGL)
			  */
			void  initializeAllTextures();

			/** Retrieves a list of all objects in text form.
			  */
			void dumpListOfObjects( mrpt::utils::CStringList  &lst );

			/** Saves the scene to a 3Dscene file, loadable by the application SceneViewer3D
			  * \sa loadFromFile
			  * \return false on any error.
			  */
			bool saveToFile(const std::string &fil) const;

			/** Loads the scene from a 3Dscene file, the format used by the application SceneViewer3D.
			  * \sa saveToFile
			  * \return false on any error.
			  */
			bool loadFromFile(const std::string &fil);

			/** Traces a ray
			  */
			bool traceRay(const mrpt::poses::CPose3D&o,double &dist) const;

			/** Evaluates the bounding box of the scene in the given viewport (default: "main"). */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max, const std::string &vpn=std::string("main") ) const;

			/** Recursive depth-first visit all objects in all viewports of the scene, calling the user-supplied function
			  *  The passed function must accept only one argument of type "const mrpt::opengl::CRenderizablePtr &"
			  */
			template <typename FUNCTOR>
			void visitAllObjects( FUNCTOR functor) const
			{
				MRPT_START
				for (TListViewports::const_iterator it = m_viewports.begin();it!=m_viewports.end();++it)
					for (COpenGLViewport::const_iterator itO = (*it)->begin();itO!=(*it)->end();++itO)
						internal_visitAllObjects(functor, *itO);
				MRPT_END
			}

			/** Recursive depth-first visit all objects in all viewports of the scene, calling the user-supplied function
			  *  The passed function must accept a first argument of type "const mrpt::opengl::CRenderizablePtr &"
			  *  and a second one of type EXTRA_PARAM
			  */
			template <typename FUNCTOR,typename EXTRA_PARAM>
			inline void visitAllObjects( FUNCTOR functor, const EXTRA_PARAM &userParam) const {
				visitAllObjects( std::bind2nd(functor,userParam) );
			}

		protected:
			bool		m_followCamera;

			typedef std::vector<COpenGLViewportPtr> TListViewports;

			TListViewports		m_viewports;	//!< The list of viewports, indexed by name.


			template <typename FUNCTOR>
			static void internal_visitAllObjects(FUNCTOR functor, const CRenderizablePtr &o)
			{
				functor(o);
				if (IS_CLASS(o,CSetOfObjects))
				{
					CSetOfObjectsPtr obj = CSetOfObjectsPtr(o);
					for (CSetOfObjects::const_iterator it=obj->begin();it!=obj->end();++it)
						internal_visitAllObjects(functor,*it);
				}
			}

		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( COpenGLScene, mrpt::utils::CSerializable, OPENGL_IMPEXP )
		
			/** Inserts an openGL object into a scene. Allows call chaining. \sa mrpt::opengl::COpenGLScene::insert  */
		inline COpenGLScenePtr &operator<<(COpenGLScenePtr &s,const CRenderizablePtr &r)	{
			s->insert(r);
			return s;
		}
		/**Inserts any iterable collection of openGL objects into a scene, allowing call chaining.  \sa mrpt::opengl::COpenGLScene::insert */
		template <class T> inline COpenGLScenePtr &operator<<(COpenGLScenePtr &s,const std::vector<T> &v)	{
			s->insert(v.begin(),v.end());
			return s;
		}
	} // end namespace

} // End of namespace


#endif
