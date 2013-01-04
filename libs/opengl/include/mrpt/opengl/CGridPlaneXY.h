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

#ifndef opengl_CGridPlaneXY_H
#define opengl_CGridPlaneXY_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CGridPlaneXY;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGridPlaneXY , CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A grid of lines over the XY plane.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CGridPlaneXY </td> <td> \image html preview_CGridPlaneXY.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CGridPlaneXY : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CGridPlaneXY )

		protected:
			float	m_xMin, m_xMax;
			float	m_yMin, m_yMax;
			float	m_plane_z;
			float	m_frequency;


		public:
			void setPlaneLimits(float xmin,float xmax, float ymin, float ymax)
			{
				m_xMin=xmin; m_xMax = xmax;
				m_yMin=ymin; m_yMax = ymax;
				CRenderizableDisplayList::notifyChange();
			}

			void getPlaneLimits(float &xmin,float &xmax, float &ymin, float &ymax) const
			{
				xmin=m_xMin; xmax=m_xMax;
				ymin=m_yMin; ymax=m_yMax;
			}

			void setPlaneZcoord(float z) { CRenderizableDisplayList::notifyChange(); m_plane_z=z;  }
			float getPlaneZcoord() const { return m_plane_z; }

			void setGridFrequency(float freq) { ASSERT_(freq>0); m_frequency=freq; CRenderizableDisplayList::notifyChange(); }
			float getGridFrequency() const { return m_frequency; }


			/** Render */
			virtual void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** Class factory  */
			static CGridPlaneXYPtr Create(
				float				xMin,
				float				xMax,
				float				yMin,
				float				yMax,
				float				z    = 0,
				float				frequency = 1 )
			{
				return CGridPlaneXYPtr( new CGridPlaneXY(
					xMin,
					xMax,
					yMin,
					yMax,
					z,
					frequency ) );
			}


		private:
			/** Constructor
			  */
			CGridPlaneXY(
				float				xMin = -10,
				float				xMax = 10 ,
				float				yMin = -10,
				float				yMax = 10,
				float				z    = 0,
				float				frequency = 1
				) :
				m_xMin(xMin),
				m_xMax(xMax),
				m_yMin(yMin),
				m_yMax(yMax),
				m_plane_z(z),
				m_frequency(frequency)
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CGridPlaneXY() { }
		};

	} // end namespace

} // End of namespace


#endif
