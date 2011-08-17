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

		public:

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
				float				xMin = -10,
				float				xMax = 10,
				float				zMin = -10,
				float				zMax = 10,
				float				y = 0,
				float				frequency = 1
				)
			{
				return CGridPlaneXZPtr( new CGridPlaneXZ( xMin,xMax, zMin, zMax, y, frequency ) );
			}

			/** Render
			  */
			void  render_dl() const;

		private:
			/** Constructor
			  */
			CGridPlaneXZ(
				float				xMin = -10,
				float				xMax = 10,
				float				zMin = -10,
				float				zMax = 10,
				float				y = 0,
				float				frequency = 1
				) :
				m_xMin(xMin),m_xMax(xMax),
				m_zMin(zMin),m_zMax(zMax),
				m_plane_y(y),
				m_frequency(frequency)
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CGridPlaneXZ() { }
		};

	} // end namespace

} // End of namespace


#endif
