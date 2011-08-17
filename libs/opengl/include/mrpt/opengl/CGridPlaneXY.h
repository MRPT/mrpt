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
