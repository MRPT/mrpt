/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CDisk_H
#define opengl_CDisk_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace opengl
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDisk, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A planar disk in the XY plane.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CDisk : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CDisk )

		protected:
			float		m_radiusIn,m_radiusOut;
			uint32_t	m_nSlices, m_nLoops;

		public:
			void setDiskRadius(float outRadius, float inRadius=0) { m_radiusIn=inRadius; m_radiusOut=outRadius; CRenderizableDisplayList::notifyChange(); }

			float getInRadius() const { return m_radiusIn; }
			float getOutRadius() const { return m_radiusOut; }

			void setSlicesCount(uint32_t N) { m_nSlices=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=50
			void setLoopsCount(uint32_t N) { m_nLoops=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=4


			/** Render
			  */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Ray tracing
			  */
			bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;

			static CDiskPtr Create(float radiusOut,float radiusIn,uint32_t slices=50,uint32_t loops=4);

		private:
			/** Constructor
			  */
			CDisk():m_radiusIn(0),m_radiusOut(1),m_nSlices(50),m_nLoops(4) {}

			CDisk(float rOut,float rIn,uint32_t slices,uint32_t loops):m_radiusIn(rIn),m_radiusOut(rOut),m_nSlices(slices),m_nLoops(loops)	{}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CDisk() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CDisk, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace


#endif
