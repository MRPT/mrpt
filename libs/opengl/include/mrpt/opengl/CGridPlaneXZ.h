/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef opengl_CGridPlaneXZ_H
#define opengl_CGridPlaneXZ_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CGridPlaneXZ;

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
				)
			{
				return CGridPlaneXZPtr( new CGridPlaneXZ( xMin,xMax, zMin, zMax, y, frequency,lineWidth,antiAliasing ) );
			}

			/** Render
			  */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;
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

	} // end namespace

} // End of namespace


#endif
