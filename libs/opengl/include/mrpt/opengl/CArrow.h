/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

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
