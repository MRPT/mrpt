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
#ifndef opengl_CAxis_H
#define opengl_CAxis_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CAxis;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAxis, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** Draw a 3D world axis, with coordinate marks at some regular interval
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *  <tr> <td> mrpt::opengl::CAxis </td> <td> \image html preview_CAxis.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CAxis : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CAxis )
		protected:
			float	m_xmin,m_ymin,m_zmin;
			float	m_xmax,m_ymax,m_zmax;
			float	m_frecuency;
            float	m_lineWidth;
			bool	m_marks;

		public:
			void setAxisLimits(float xmin,float ymin, float zmin, float xmax,float ymax, float zmax)
			{
				m_xmin=xmin; m_ymin=ymin; m_zmin=zmin;
				m_xmax=xmax; m_ymax=ymax; m_zmax=zmax;
				CRenderizableDisplayList::notifyChange();
			}

			void setFrequency(float f) { ASSERT_(f>0); m_frecuency=f; CRenderizableDisplayList::notifyChange(); } //!< Changes the frequency of the "ticks"

			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); }
			float getLineWidth() const { return  m_lineWidth;}

			void enableTickMarks(bool v=true) { m_marks=v; CRenderizableDisplayList::notifyChange(); }


			/** Class factory  */
			static CAxisPtr Create(
				float xmin,float ymin, float zmin,
				float xmax, float ymax,  float zmax,
				float frecuency = 1, float lineWidth = 3, bool marks=false)
			{
				return CAxisPtr( new CAxis( xmin,ymin, zmin, xmax,ymax,zmax,frecuency,lineWidth,marks  ) );
			}

			/** Render
			  */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

	private:
			/** Constructor
			  */
			CAxis(
				float xmin=-1.0f,float ymin=-1.0f, float zmin=-1.0f,
				float xmax=1.0f, float ymax=1.0f,  float zmax=1.0f,
				float frecuency = 0.25f, float lineWidth = 3.0f, bool marks=false) :
				m_xmin(xmin),m_ymin(ymin),m_zmin(zmin),
				m_xmax(xmax),m_ymax(ymax),m_zmax(zmax),
				m_frecuency(frecuency),
                m_lineWidth(lineWidth),
				m_marks(marks)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CAxis() { }
		};

	} // end namespace
} // End of namespace

#endif
