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
#ifndef opengl_CSimpleLine_H
#define opengl_CSimpleLine_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSimpleLine;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSimpleLine, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A line segment
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSimpleLine : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CSimpleLine )

		protected:
			float	m_x0,m_y0,m_z0;
			float	m_x1,m_y1,m_z1;
            float	m_lineWidth;
		public:
			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); }
			float getLineWidth() const { return  m_lineWidth;}

			void setLineCoords(float x0,float y0,float z0, float x1, float y1, float z1)
			{
				m_x0=x0; m_y0=y0; m_z0=z0;
				m_x1=x1; m_y1=y1; m_z1=z1;
				CRenderizableDisplayList::notifyChange();
			}

			void getLineCoords(float &x0,float &y0,float &z0, float &x1, float &y1, float &z1) const
			{
				x0=m_x0; y0=m_y0; z0=m_z0;
				x1=m_x1; y1=m_y1; z1=m_z1;
			}

			/** Render
			  */
			void  render_dl() const;

			/** Class factory */
			static CSimpleLinePtr Create(
				float x0,float y0, float z0,
				float x1,float y1, float z1, float lineWidth = 1 )
			{
				return CSimpleLinePtr(new CSimpleLine(x0,y0,z0,x1,y1,z1,lineWidth));
			}

		private:
			/** Constructor
			  */
			CSimpleLine(
				float x0=0,float y0=0, float z0=0,
				float x1=0,float y1=0, float z1=0, float lineWidth = 1 ) :
					m_x0(x0),m_y0(y0),m_z0(z0),
					m_x1(x1),m_y1(y1),m_z1(z1),
					m_lineWidth(lineWidth)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSimpleLine() { }
		};

	} // end namespace

} // End of namespace


#endif
