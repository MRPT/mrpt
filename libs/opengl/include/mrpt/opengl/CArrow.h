/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef opengl_CArrow_H
#define opengl_CArrow_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CArrow;

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
			void  render_dl() const;

			/** Class factory  */
			static CArrowPtr Create(
				float	x0,
				float	y0,
				float	z0,
				float	x1,
				float	y1,
				float	z1,
				float	headRatio = 0.2f,
				float	smallRadius = 0.05f,
				float	largeRadius = 0.2f,
				float	arrow_roll = -1.0f,
				float	arrow_pitch = -1.0f,
				float	arrow_yaw = -1.0f
				)
			{
				return CArrowPtr(new CArrow(x0,y0,z0, x1,y1,z1, headRatio, smallRadius, largeRadius, arrow_roll, arrow_pitch, arrow_yaw ));
			}

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


	} // end namespace
} // End of namespace

#endif
