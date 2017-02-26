/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_COpenGLStandardObject_H
#define opengl_COpenGLStandardObject_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/geometry.h>

namespace mrpt	{
	namespace opengl	{
		typedef uint32_t _GLENUM;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(COpenGLStandardObject,CRenderizableDisplayList, OPENGL_IMPEXP)
		/**
		  * Objects of this class represent a generic openGL object without specific geometric properties.
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP COpenGLStandardObject:public CRenderizableDisplayList	{
			DEFINE_SERIALIZABLE(COpenGLStandardObject)
		protected:
			/**
			  * OpenGL identifier of the object type.
			  */
			_GLENUM type;
			/**
			  * Set of points in which consists this object.
			  */
			std::vector<mrpt::math::TPoint3D> vertices;
			/**
			  * Granularity of the openGL elements. 3 for GL_TRIANGLES, 4 for GL_QUADS, and so on. Setting it to 0 will generate a single openGL object.
			  */
			uint32_t chunkSize;
			/**
			  * Set of openGL properties enabled in the rendering of this object.
			  */
			std::vector<_GLENUM> enabled;
			float normal[3];
		public:
			/**
			  * Render.
			  * \sa mrpt::opengl::CRenderizable
			  */
			virtual void render_dl() const MRPT_OVERRIDE;
			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;
			/**
			  * Ray Tracing. Will always return false, since objects of this class are not intended to have geometric properties.
			  * \sa mrpt::opengl::CRenderizable
			  */
			bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;
			/**
			  * Creation of object from type, vertices, chunk size and a list of enabled openGL flags.
			  * \throw std::logic_error if the number of vertices is not an exact multiple of the chunk size.
			  */
			static COpenGLStandardObjectPtr Create(_GLENUM t,const std::vector<mrpt::math::TPoint3D> &v,uint32_t cs=0,const std::vector<_GLENUM> &en=std::vector<_GLENUM>());

			/**
			  * Enable some openGL flag.
			  */
			inline void enable(_GLENUM flag)	{
				if (find(enabled.begin(),enabled.end(),flag)==enabled.end()) enabled.push_back(flag);
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Disable some openGL flag.
			  */
			void disable(_GLENUM flag);
			/**
			  * Check whether an openGL will be enabled during the rendering of this object.
			  */
			inline bool isEnabled(_GLENUM flag) const	{
				return find(enabled.begin(),enabled.end(),flag)!=enabled.end();
			}
			/**
			  * Get a list of all currently enabled openGL flags.
			  */
			inline void getEnabledFlags(std::vector<_GLENUM> &v) const	{
				v=enabled;
			}
			/**
			  * Set the list of all openGL flags.
			  */
			inline void setFlags(const std::vector<_GLENUM> &v)	{
				enabled=v;
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Set the normal vector to this object.
			  */
			inline void setNormal(const float (&n)[3])	{
				for (size_t i=0;i<3;i++) normal[i]=n[i];
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Gets the normal vector to this object.
			  */
			inline void getNormal(float (&n)[3]) const	{
				for (size_t i=0;i<3;i++) n[i]=normal[i];
			}
		private:
			/**
			  * Constructor with all the information.
			  */
			COpenGLStandardObject(_GLENUM t,const std::vector<mrpt::math::TPoint3D> &v,uint32_t cs,const std::vector<_GLENUM> &en):type(t),vertices(v),chunkSize(cs),enabled(en)	{
				for (size_t i=0;i<3;i++) normal[i]=0.0;
			}
			/**
			  * Baic empty constructor, initializes to default.
			  */
			COpenGLStandardObject():type(0),vertices(std::vector<mrpt::math::TPoint3D>(0)),chunkSize(0),enabled(std::vector<_GLENUM>())	{
				for (size_t i=0;i<3;i++) normal[i]=0.0;
			}
			/**
			  * Destructor.
			  */
			virtual ~COpenGLStandardObject()	{}
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(COpenGLStandardObject,CRenderizableDisplayList, OPENGL_IMPEXP)
	} // end namespace
} // End of namespace
#endif
