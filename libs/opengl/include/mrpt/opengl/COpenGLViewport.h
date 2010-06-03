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
#ifndef opengl_COpenGLViewport_H
#define opengl_COpenGLViewport_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace utils { class CStringList; }

	using namespace mrpt::math;

	/** The namespace for 3D scene representation and rendering.
	  */
	namespace opengl
	{
		class COpenGLScene;
		class CRenderizable;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COpenGLViewport, mrpt::utils::CSerializable, OPENGL_IMPEXP )

		/** A viewport within a COpenGLScene, containing a set of OpenGL objects to render.
		  *   This class has protected constuctor, thus it cannot be created by users. Use COpenGLScene::createViewport instead.
		  *  Refer to opengl::COpenGLScene for further details.
		  */
		class OPENGL_IMPEXP COpenGLViewport : public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( COpenGLViewport )

			friend class COpenGLScene;
		public:

			typedef CListOpenGLObjects::const_iterator 	const_iterator;
			typedef CListOpenGLObjects::iterator 		iterator;

			inline const_iterator begin() const { return  m_objects.begin(); }
			inline const_iterator end() const { return  m_objects.end(); }
			inline iterator begin() { return  m_objects.begin(); }
			inline iterator end() { return  m_objects.end(); }


			/** Set this view as a clone of some other viewport, given its name - as a side effect, current list of internal OpenGL objects is cleared.
			  *  By default, only the objects are cloned, not the camera. See
			  * \sa resetCloneView
			  */
			void setCloneView( const std::string &clonedViewport );

			/** Reset the viewport to normal mode: rendering its own objects.
			  * \sa setCloneView
			  */
			inline void resetCloneView() { m_isCloned=false;m_isClonedCamera=false; }

			/** If set to true, and setCloneView() has been called, this viewport will be rendered using the camera of the cloned viewport.
			  */
			inline void setCloneCamera(bool enable) { m_isClonedCamera = enable; }


			/** Delete all internal obejcts
			  * \sa insert
			  */
			void clear();

			/** Insert a new object into the list.
			  *  The object MUST NOT be deleted, it will be deleted automatically by this object when not required anymore.
			  */
			void insert( const CRenderizablePtr &newObject );

			/** Returns the name of the viewport */
			inline std::string getName() { return m_name; }

			/** Change the viewport position and dimension on the rendering window.
			  *  Coordinates here are alwasys in the range [0,1], relative to the actual
			  *   window sizes (i.e. width=1 means the entire width of the rendering window).
			  * \note (x,y) specify the lower left corner of the viewport rectangle.
			  * \sa getViewportPosition
			  */
			void setViewportPosition(
				const double x,
				const double y,
				const double width,
				const double height );

			/** Get the current viewport position and dimension on the rendering window.
			  *  Coordinates here are alwasys in the range [0,1], relative to the actual
			  *   window sizes (i.e. width=1 means the entire width of the rendering window).
			  * \note (x,y) specify the lower left corner of the viewport rectangle.
			  * \sa setViewportPosition
			  */
			void getViewportPosition(
				double &x,
				double &y,
				double &width,
				double &height );

			/** Set the min/max clip depth distances for rendering (default: 0.1 - 10000)
			  * \sa getViewportClipDistances
			  */
			void setViewportClipDistances(const double clip_min, const double clip_max);

			/** Get the current min/max clip depth distances for rendering (default: 0.1 - 10000)
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
			inline void setCustomBackgroundColor( const TColorf &color ) { m_custom_backgb_color = true; m_background_color = color; }

			inline bool hasCustomBackgroundColor() const { return m_custom_backgb_color; }

			inline TColorf getCustomBackgroundColor() const { return m_background_color; }


			virtual ~COpenGLViewport();  //!< Destructor: clears all objects.

			/** Compute the 3D ray corresponding to a given pixel; this can be used to allow the user to pick and select 3D objects by clicking onto the 2D image.
			  *  \param x_coord Horizontal coordinate with the usual meaning (0:left of the viewport, W-1: right border).
			  *  \param y_coord Horizontal coordinate with the usual meaning (0:top of the viewport, H-1: right border).
			  * \param out_cameraPose If not NULL, will have the camera 3D pose as a mrpt::poses::CPose3D. See also
			  * \note (x,y) refer to VIEWPORT coordinates. Take into account this when viewports do not extend to the whole window size.
			  * \note x and y are double instead of integers to allow sub-pixel precision.
			  * \sa getCurrentCameraPose
			  */
			void get3DRayForPixelCoord( const double x_coord, const double y_coord, mrpt::math::TLine3D &out_ray, mrpt::poses::CPose3D *out_cameraPose=NULL ) const;

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
				MRPT_START;
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
				MRPT_END;
			 }



			/** Removes the given object from the scene (it also deletes the object to free its memory).
			  */
			void removeObject( const CRenderizablePtr & obj );

			/** Number of objects contained. */
			inline size_t  size() const { return m_objects.size(); }

			inline bool empty() const { return m_objects.empty(); }


			opengl::CCamera& getCamera() { return m_camera;} //!< Get a reference to the camera associated with this viewport.

			const opengl::CCamera & getCamera() const { return m_camera;} //!< Get a reference to the camera associated with this viewport.

		protected:
			/** Constructor, invoked from COpenGLScene only.
			  */
			COpenGLViewport( COpenGLScene *parent=NULL, const std::string &name=std::string("") );

			/** Initializes all textures in the scene (See opengl::CTexturedPlane::loadTextureInOpenGL)
			  */
			void  initializeAllTextures();

			/** Retrieves a list of all objects in text form.
			  */
			void dumpListOfObjects( utils::CStringList  &lst );

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
			TColorf			m_background_color;  //!< used only if m_custom_backgb_color

			struct TLastProjectiveMatrixInfo
			{
				TLastProjectiveMatrixInfo() : is_projective(true),eye(0,0,0),pointing(0,0,0),up(0,0,0), FOV(30), viewport_width(640), viewport_height(480), azimuth(0), elev(0), zoom(1)
				{}
				bool is_projective;  // true: projective, false: ortho
				TPoint3D 	eye;		//!< The camera is here.
				TPoint3D 	pointing; 	//!< The camera points to here
				TPoint3D	up; 		//!< Up vector of the camera.
				float FOV; //!< FOV in degrees.
				size_t viewport_width, viewport_height; //!< In pixels. This may be smaller than the total render window.
				float azimuth, elev; //!< Camera elev & azimuth, in radians.
				float zoom;
			};
			mutable TLastProjectiveMatrixInfo m_lastProjMat;  //!< Info updated with each "render()" and used in "get3DRayForPixelCoord"

			/** The list of objects that comprise the 3D scene.
			  *  Objects are automatically deleted when calling "clear" or in the destructor.
			  */
			opengl::CListOpenGLObjects		m_objects;

		};
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

	} // end namespace

} // End of namespace


#endif
