/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CArrow_H
#define opengl_CArrow_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{


		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CArrow, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A 3D arrow
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CArrow </td> <td> \image html preview_CArrow.png </td> </tr>
		  *  </table>
		  *  </div>
		  * \ingroup mrpt_opengl_grp
		  *  
		  */
		class OPENGL_IMPEXP CArrow : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CArrow )
		protected:
			mutable float	m_x0,m_y0,m_z0;
			mutable float	m_x1,m_y1,m_z1;
			float	m_headRatio;
			float	m_smallRadius, m_largeRadius;
			//For version 2 in stream
			float	m_arrow_roll;
			float	m_arrow_pitch;
			float	m_arrow_yaw;

		public:

			void setArrowEnds(float x0,float y0, float z0, float x1,float y1, float z1)
			{
				m_x0=x0;  m_y0 = y0;  m_z0=z0;
				m_x1=x1;  m_y1 = y1;  m_z1=z1;
				CRenderizableDisplayList::notifyChange();
			}
			void setHeadRatio(float rat) { m_headRatio=rat; CRenderizableDisplayList::notifyChange(); }
			void setSmallRadius(float rat) { m_smallRadius=rat; CRenderizableDisplayList::notifyChange(); }
			void setLargeRadius(float rat) { m_largeRadius=rat; CRenderizableDisplayList::notifyChange(); }
			void setArrowYawPitchRoll(float yaw,float pitch, float roll ) { m_arrow_yaw=yaw; m_arrow_pitch=pitch; m_arrow_roll=roll; CRenderizableDisplayList::notifyChange(); }

			/** Render
			  */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Class factory  */
			static CArrowPtr Create(
				float x0,float y0,float z0,
				float x1,float y1,float z1,
				float headRatio = 0.2f,float smallRadius = 0.05f,float largeRadius = 0.2f,
				float arrow_roll = -1.0f,float arrow_pitch = -1.0f,float arrow_yaw = -1.0f);

		private:
			/** Constructor
			  */
			CArrow(
				float	x0 = 0,
				float	y0 = 0,
				float	z0 = 0,
				float	x1 = 1,
				float	y1 = 1,
				float	z1 = 1,
				float	headRatio = 0.2f,
				float	smallRadius = 0.05f,
				float	largeRadius = 0.2f,
				float	arrow_roll = -1.0f,
				float	arrow_pitch = -1.0f,
				float	arrow_yaw = -1.0f
				) :
				m_x0(x0),m_y0(y0),m_z0(z0),
				m_x1(x1),m_y1(y1),m_z1(z1),
				m_headRatio(headRatio),
				m_smallRadius(smallRadius),
				m_largeRadius(largeRadius),
				m_arrow_roll(arrow_roll),
				m_arrow_pitch(arrow_pitch),
				m_arrow_yaw(arrow_yaw)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CArrow() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CArrow, CRenderizableDisplayList, OPENGL_IMPEXP )


	} // end namespace
} // End of namespace

#endif
