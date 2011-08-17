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
