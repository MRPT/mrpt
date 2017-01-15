/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_COpenGLViewport_H
#define opengl_COpenGLViewport_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CLight.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CObservable.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/mrptEvent.h>

namespace mrpt
{
	namespace utils { class CImage; }

	namespace opengl
	{
		class COpenGLScene;
		class CRenderizable;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COpenGLViewport, mrpt::utils::CSerializable, OPENGL_IMPEXP )

		/** A viewport within a COpenGLScene, containing a set of OpenGL objects to render.
		  *   This class has protected constuctor, thus it cannot be created by users. Use COpenGLScene::createViewport instead.
		  *  A viewport has these "operation modes":
		  *		- Normal (default): It renders the contained objects.
		  *		- Cloned: It clones the objects from another viewport. See \a setCloneView()
		  *		- Image mode: It renders an image (e.g. from a video stream) efficiently using a textued quad. See \a setImageView().
		  *
		  * In any case, the viewport can be resized to only fit a part of the entire parent viewport.
		  *  There will be always at least one viewport in a COpenGLScene named "main".
		  *
		  * This class can be observed (see mrpt::utils::CObserver) for the following events (see mrpt::utils::mrptEvent):
		  *   - mrpt::opengl::mrptEventGLPreRender
		  *   - mrpt::opengl::mrptEventGLPostRender
		  *
		  * Two directional light sources at infinity are created by default, with directions (-1,-1,-1) and (1,2,1), respectively.
		  * All OpenGL properties of light sources are accesible via the methods: setNumberOfLights(), lightsClearAll(), addLight(), and getLight().
		  * Please, refer to mrpt::opengl::CLight and the standard OpenGL documentation for the meaning of all light properties.
		  *
		  *  Refer to mrpt::opengl::COpenGLScene for further details.
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP COpenGLViewport :
			public mrpt::utils::CSerializable,
			public mrpt::utils::CObservable
		{
			DEFINE_SERIALIZABLE( COpenGLViewport )
			friend class COpenGLScene;
		public:
			// -------------------------------------------------------------------
			/** @name Set the viewport "modes"
			    @{ */

			/** Set this viewport as a clone of some other viewport, given its name - as a side effect, current list of internal OpenGL objects is cleared.
			  *  By default, only the objects are cloned, not the camera. See
			  * \sa resetCloneView
			  */
			void setCloneView( const std::string &clonedViewport );

			/** Set this viewport into "image view"-mode, where an image is efficiently drawn (fitting the viewport area) using an OpenGL textured quad.
			  *  Call this method with the new image to update the displayed image (but recall to first lock the parent openglscene's critical section, then do the update, then release the lock, and then issue a window repaint).
			  *  Internally, the texture is drawn using a mrpt::opengl::CTexturedPlane
			  *  The viewport can be reverted to behave like a normal viewport by calling setNormalMode()
			  * \sa setImageView_fast
			  */
			void setImageView(const mrpt::utils::CImage &img);

			/** Just like \a setImageView but moves the internal image memory instead of making a copy, so it's faster but empties the input image.
			  * \sa setImageView
			  */
			void setImageView_fast(mrpt::utils::CImage &img);

			/** Reset the viewport to normal mode: rendering its own objects.
			  * \sa setCloneView, setNormalMode
			  */
			inline void resetCloneView() { setNormalMode(); }

			/** If set to true, and setCloneView() has been called, this viewport will be rendered using the camera of the cloned viewport.
			  */
			inline void setCloneCamera(bool enable) { m_isClonedCamera = enable; }

			/** Resets the viewport to a normal 3D viewport \sa setCloneView, setImageView */
			void setNormalMode();

			/** @} */ // end of Set the "viewport mode"
			// ------------------------------------------------------


			/** @name OpenGL global settings that affect rendering all objects in the scene/viewport
			    @{ */

			/** Sets glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST) is enabled, or GL_FASTEST otherwise. */
			void enablePolygonNicest(bool enable=true) { m_OpenGL_enablePolygonNicest=enable; }
			bool isPolygonNicestEnabled() const { return m_OpenGL_enablePolygonNicest; }

			/** Removes all lights (and disables the global "GL_LIGHTING") */
			void lightsClearAll() { m_lights.clear(); }

			/** Append a new light to the scene. By default there are two lights. "GL_LIGHTING" is disabled if all lights are removed */
			void addLight(const CLight &l) { m_lights.push_back(l); }

			/** Allocates a number of lights, which must be correctly defined via getLight(i), etc. */
			void setNumberOfLights(const size_t N) { m_lights.resize(N); }

			CLight & getLight(const size_t i) { ASSERT_BELOW_(i,m_lights.size()) return m_lights[i]; }
			const CLight & getLight(const size_t i) const  { ASSERT_BELOW_(i,m_lights.size()) return m_lights[i]; }

			/** @} */ // end of OpenGL settings

			// -------------------------------------------------------------------
			/** @name Change or read viewport properties (except "viewport modes")
			    @{ */

			/** Returns the name of the viewport */
			inline std::string getName() { return m_name; }

			/** Change the viewport position and dimension on the rendering window.
			  *  X & Y coordinates here can have two interpretations:
			  *    - If in the range [0,1], they are factors with respect to the actual window sizes (i.e. width=1 means the entire width of the rendering window).
			  *    - If >1, they are interpreted as pixels.
			  *
			  *  width & height can be interpreted as:
			  *		- If >1, they are the size of the viewport in that dimension, in pixels.
			  *		- If in [0,1], they are the size of the viewport in that dimension, in a factor of the width/height.
			  *		- If in [-1,0[, the size is computed such as the right/top border ends up in the given coordinate, interpreted as a factor (e.g. -1: up to the end of the viewport, -0.5: up to the middle of it).
			  *		- If <-1 the size is computed such as the right/top border ends up in the given absolute coordinate (e.g. -200: up to the row/column 200px).
			  *
			  * \note (x,y) specify the lower left corner of the viewport rectangle.
			  * \sa getViewportPosition
			  */
			void setViewportPosition(
				const double x,
				const double y,
				const double width,
				const double height );

			/** Get the current viewport position and dimension on the rendering window.
			  *  X & Y coordinates here can have two interpretations:
			  *    - If in the range [0,1], they are factors with respect to the actual window sizes (i.e. width=1 means the entire width of the rendering window).
			  *    - If >1, they are interpreted as pixels.
			  * \note (x,y) specify the lower left corner of the viewport rectangle.
			  * \sa setViewportPosition
			  */
			void getViewportPosition(
				double &x,
				double &y,
				double &width,
				double &height );

			/** Set the min/max clip depth distances of the rendering frustum (default: 0.1 - 10000)
			  * \sa getViewportClipDistances
			  */
			void setViewportClipDistances(const double clip_min, const double clip_max);

			/** Get the current min/max clip depth distances of the rendering frustum (default: 0.1 - 10000)
			  * \sa setViewportClipDistances
			  */
			void getViewportClipDistances(double &clip_min, double &clip_max) const;

			/** Set the border size ("frame") of the viewport (default=0).
			  */
			inline void setBorderSize( unsigned int lineWidth ) { m_borderWidth = lineWidth; }

			/** Return whether the viewport will be rendered transparent over previous viewports.
			  */
			inline bool isTransparent() { return m_isTransparent; }

			/** Set the transparency, that is, whether the viewport will be rendered transparent over previous viewports (default=false).
			  */
			inline void setTransparent( bool trans ) { m_isTransparent=trans; }

			/** Set a background color different from that of the parent GUI window */
			inline void setCustomBackgroundColor( const mrpt::utils::TColorf &color ) { m_custom_backgb_color = true; m_background_color = color; }

			inline mrpt::utils::TColorf getCustomBackgroundColor() const { return m_background_color; }

			/** Compute the 3D ray corresponding to a given pixel; this can be used to allow the user to pick and select 3D objects by clicking onto the 2D image.
			  *  \param x_coord Horizontal coordinate with the usual meaning (0:left of the viewport, W-1: right border).
			  *  \param y_coord Horizontal coordinate with the usual meaning (0:top of the viewport, H-1: right border).
			  * \param out_cameraPose If not NULL, will have the camera 3D pose as a mrpt::poses::CPose3D. See also
			  * \note (x,y) refer to VIEWPORT coordinates. Take into account this when viewports do not extend to the whole window size.
			  * \note x and y are double instead of integers to allow sub-pixel precision.
			  * \sa getCurrentCameraPose
			  */
			void get3DRayForPixelCoord( const double x_coord, const double y_coord, mrpt::math::TLine3D &out_ray, mrpt::poses::CPose3D *out_cameraPose=NULL ) const;

			/** @} */ // end of Change or read viewport properties
			// ------------------------------------------------------


			// -------------------------------------------------------------------
			/** @name Contained objects set/get/search
			    @{ */

			typedef CListOpenGLObjects::const_iterator 	const_iterator;
			typedef CListOpenGLObjects::iterator 		iterator;

			inline const_iterator begin() const { return  m_objects.begin(); }
			inline const_iterator end() const { return  m_objects.end(); }
			inline iterator begin() { return  m_objects.begin(); }
			inline iterator end() { return  m_objects.end(); }

			/** Delete all internal obejcts
			  * \sa insert */
			void clear();

			/** Insert a new object into the list.
			  *  The object MUST NOT be deleted, it will be deleted automatically by this object when not required anymore.
			  */
			void insert( const CRenderizablePtr &newObject );

			/** Compute the current 3D camera pose.
			  * \sa get3DRayForPixelCoord
			  */
			void getCurrentCameraPose( mrpt::poses::CPose3D &out_cameraPose ) const;

			/** Returns the first object with a given name, or NULL if not found.
			  */
			CRenderizablePtr getByName( const std::string &str );

			 /** Returns the i'th object of a given class (or of a descendant class), or NULL (an empty smart pointer) if not found.
			   *  Example:
			   * \code
					CSpherePtr obs = view.getByClass<CSphere>();
			   * \endcode
			   * By default (ith=0), the first observation is returned.
			   */
			 template <typename T>
			 typename T::SmartPtr getByClass( const size_t &ith = 0 ) const
			 {
				MRPT_START
				size_t  foundCount = 0;
				const mrpt::utils::TRuntimeClassId*	class_ID = T::classinfo;
				for (CListOpenGLObjects::const_iterator it = m_objects.begin();it!=m_objects.end();++it)
					if ( (*it).present() &&  (*it)->GetRuntimeClass()->derivedFrom( class_ID ) )
						if (foundCount++ == ith)
							return typename T::SmartPtr(*it);

				// If not found directly, search recursively:
				for (CListOpenGLObjects::const_iterator it=m_objects.begin();it!=m_objects.end();++it)
				{
					if ( (*it).present() && (*it)->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects,mrpt::opengl))
					{
						typename T::SmartPtr  o = CSetOfObjectsPtr(*it)->getByClass<T>(ith);
						if (o.present()) return o;
					}
				}
				return typename T::SmartPtr();	// Not found: return empty smart pointer
				MRPT_END
			 }

			/** Removes the given object from the scene (it also deletes the object to free its memory).
			  */
			void removeObject( const CRenderizablePtr & obj );

			/** Number of objects contained. */
			inline size_t  size() const { return m_objects.size(); }

			inline bool empty() const { return m_objects.empty(); }

			opengl::CCamera& getCamera() { return m_camera;} //!< Get a reference to the camera associated with this viewport.

			const opengl::CCamera & getCamera() const { return m_camera;} //!< Get a reference to the camera associated with this viewport.

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** @} */ // end of Contained objects set/get/search
			// ------------------------------------------------------


			virtual ~COpenGLViewport();  //!< Destructor: clears all objects.


		protected:
			/** Constructor, invoked from COpenGLScene only.
			  */
			COpenGLViewport( COpenGLScene *parent=NULL, const std::string &name=std::string("") );

			/** Initializes all textures in the scene (See opengl::CTexturedPlane::loadTextureInOpenGL)
			  */
			void  initializeAllTextures();

			/** Retrieves a list of all objects in text form.
			  */
			void dumpListOfObjects( mrpt::utils::CStringList  &lst );

			/** Render the objects in this viewport (called from COpenGLScene only) */
			void  render( const int render_width, const int render_height ) const;

			/** The camera associated to the viewport */
			opengl::CCamera		m_camera;
			utils::safe_ptr<COpenGLScene>  m_parent;   //!< The scene that contains this viewport.
			bool			m_isCloned; //!< Set by setCloneView
			bool			m_isClonedCamera; //!< Set by setCloneCamera
			std::string		m_clonedViewport; //!< Only if m_isCloned=true
			std::string		m_name; //!< The viewport's name
			bool			m_isTransparent; //!< Whether to clear color buffer.
			uint32_t        m_borderWidth;  //!< Default=0, the border around the viewport.
			double			m_view_x, m_view_y,m_view_width,m_view_height; //!< The viewport position [0,1]
			double			m_clip_min,m_clip_max; //!< The min/max clip depth distances (default: 0.1 - 10000)
			bool			m_custom_backgb_color;
			mrpt::utils::TColorf m_background_color;  //!< used only if m_custom_backgb_color
			bool			m_isImageView; //!< Set by setImageView
			//CRenderizablePtr m_imageview_quad ; //!< A mrpt::opengl::CTexturedPlane used after setImageView() is called
			mrpt::utils::CImagePtr  m_imageview_img; //!< The image to display, after calling \a setImageView()

			struct TLastProjectiveMatrixInfo
			{
				TLastProjectiveMatrixInfo() : eye(0,0,0),pointing(0,0,0),up(0,0,0), viewport_width(640), viewport_height(480), FOV(30), azimuth(0), elev(0), zoom(1),is_projective(true)
				{}
			 mrpt::math::TPoint3D 	eye;		//!< The camera is here.
			 mrpt::math::TPoint3D 	pointing; 	//!< The camera points to here
			 mrpt::math::TPoint3D	up; 		//!< Up vector of the camera.
				size_t viewport_width, viewport_height; //!< In pixels. This may be smaller than the total render window.
				float FOV; //!< FOV in degrees.
				float azimuth, elev; //!< Camera elev & azimuth, in radians.
				float zoom;
				bool is_projective;  // true: projective, false: ortho
			};
			mutable TLastProjectiveMatrixInfo m_lastProjMat;  //!< Info updated with each "render()" and used in "get3DRayForPixelCoord"

			/** The list of objects that comprise the 3D scene.
			  *  Objects are automatically deleted when calling "clear" or in the destructor.
			  */
			opengl::CListOpenGLObjects		m_objects;

			void internal_setImageView_fast(const mrpt::utils::CImage &img, bool is_fast);

			// OpenGL global settings:
			bool  m_OpenGL_enablePolygonNicest;

			std::vector<CLight>  m_lights;
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( COpenGLViewport, mrpt::utils::CSerializable, OPENGL_IMPEXP )
		/**
		  * Inserts an openGL object into a viewport. Allows call chaining.
		  * \sa mrpt::opengl::COpenGLViewport::insert
		  */
		inline COpenGLViewportPtr &operator<<(COpenGLViewportPtr &s,const CRenderizablePtr &r)	{
			s->insert(r);
			return s;
		}
		/**
		  * Inserts any iterable set of openGL objects into a viewport. Allows call chaining.
		  * \sa mrpt::opengl::COpenGLViewport::insert
		  */
		inline COpenGLViewportPtr &operator<<(COpenGLViewportPtr &s,const std::vector<CRenderizablePtr> &v)	{
			for (std::vector<CRenderizablePtr>::const_iterator it=v.begin();it!=v.end();++it) s->insert(*it);
			return s;
		}



		/** @name Events emitted by COpenGLViewport
			@{  */

		/**  An event sent by an mrpt::opengl::COpenGLViewport just after clearing the viewport and setting the GL_PROJECTION matrix, and before calling the scene OpenGL drawing primitives.
		  *
		  *  While handling this event you can call OpenGL glBegin(),glEnd(),gl* functions or those in mrpt::opengl::gl_utils to draw stuff *in the back* of the normal
		  *   objects contained in the COpenGLScene.
		  *
		  *  After processing this event, COpenGLViewport will change the OpenGL matrix mode into "GL_MODELVIEW" and load an identity matrix to continue
		  *   rendering the scene objects as usual. Any change done to the GL_PROJECTION will have effects, so do a glPushMatrix()/glPopMatrix() if that is not your intention.
		  *
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will most likely be invoked from an internal GUI thread of MRPT,
		  *    so all your code in the handler must be thread safe.
		  */
		class OPENGL_IMPEXP mrptEventGLPreRender : public mrpt::utils::mrptEvent
		{
		protected:
			void do_nothing() MRPT_OVERRIDE { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventGLPreRender(const COpenGLViewport* obj) : source_viewport(obj) { }
			const COpenGLViewport * const source_viewport;
		}; // End of class def.

		/**  An event sent by an mrpt::opengl::COpenGLViewport after calling the scene OpenGL drawing primitives and before doing a glSwapBuffers
		  *
		  *  While handling this event you can call OpenGL glBegin(),glEnd(),gl* functions or those in mrpt::opengl::gl_utils to draw stuff *on the top* of the normal
		  *   objects contained in the COpenGLScene.
		  *
		  *  IMPORTANTE NOTICE: Event handlers in your observer class will most likely be invoked from an internal GUI thread of MRPT,
		  *    so all your code in the handler must be thread safe.
		  */
		class OPENGL_IMPEXP mrptEventGLPostRender : public mrpt::utils::mrptEvent
		{
		protected:
			void do_nothing()  MRPT_OVERRIDE { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventGLPostRender(const COpenGLViewport* obj) : source_viewport(obj) { }
			const COpenGLViewport * const source_viewport;
		}; // End of class def.


		/** @} */



	} // end namespace

} // End of namespace


#endif
