/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CGridPlaneXZ_H
#define opengl_CGridPlaneXZ_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{


		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGridPlaneXZ, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A grid of lines over the XZ plane.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CGridPlaneXZ </td> <td> \image html preview_CGridPlaneXZ.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CGridPlaneXZ : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CGridPlaneXZ )

		protected:
			float	m_xMin, m_xMax;
			float	m_zMin, m_zMax;
			float	m_plane_y;
			float	m_frequency;
            float	m_lineWidth;
			bool    m_antiAliasing;

		public:
			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); }
			float getLineWidth() const { return  m_lineWidth;}

			void enableAntiAliasing(bool enable=true) { m_antiAliasing =enable; CRenderizableDisplayList::notifyChange(); }
			bool isAntiAliasingEnabled() const { return m_antiAliasing; }

			void setPlaneLimits(float xmin,float xmax, float zmin, float zmax)
			{
				m_xMin=xmin; m_xMax = xmax;
				m_zMin=zmin; m_zMax = zmax;
				CRenderizableDisplayList::notifyChange();
			}

			void getPlaneLimits(float &xmin,float &xmax, float &zmin, float &zmax) const
			{
				xmin=m_xMin; xmax=m_xMax;
				zmin=m_zMin; zmax=m_zMax;
			}

			void setPlaneYcoord(float y) { m_plane_y=y; CRenderizableDisplayList::notifyChange(); }
			float getPlaneYcoord() const { return m_plane_y; }

			void setGridFrequency(float freq) { ASSERT_(freq>0); m_frequency=freq; CRenderizableDisplayList::notifyChange(); }
			float getGridFrequency() const { return m_frequency; }



			/** Class factory  */
			static CGridPlaneXZPtr Create(
				float xMin = -10,
				float xMax = 10,
				float zMin = -10,
				float zMax = 10,
				float y = 0,
				float frequency = 1,
				float lineWidth = 1.3f,
				bool  antiAliasing = true
				);

			/** Render
			  */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;
		private:
			/** Constructor */
			CGridPlaneXZ(
				float xMin = -10,
				float xMax = 10,
				float zMin = -10,
				float zMax = 10,
				float y = 0,
				float frequency = 1,
				float lineWidth = 1.3f,
				bool  antiAliasing = true
				);
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CGridPlaneXZ() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CGridPlaneXZ, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace


#endif
