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
#ifndef opengl_CDisk_H
#define opengl_CDisk_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CDisk;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CDisk, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A planar disk in the XY plane.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CDisk </td> <td> \image html preview_CDisk.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CDisk : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CDisk )

		protected:
			float		m_radiusIn,m_radiusOut;
			uint32_t	m_nSlices, m_nLoops;

		public:
			void setDiskRadius(float outRadius, float inRadius=0) { m_radiusIn=inRadius; m_radiusOut=outRadius; CRenderizableDisplayList::notifyChange(); }

			float getInRadius() const { return m_radiusIn; }
			float getOutRadius() const { return m_radiusOut; }

			void setSlicesCount(uint32_t N) { m_nSlices=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=50
			void setLoopsCount(uint32_t N) { m_nLoops=N; CRenderizableDisplayList::notifyChange(); }  //!< Default=4


			/** Render
			  */
			void  render_dl() const;

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			static CDiskPtr Create(float radiusOut,float radiusIn,uint32_t slices=50,uint32_t loops=4)	{
				return CDiskPtr(new CDisk(radiusOut,radiusIn,slices,loops));
			}

		private:
			/** Constructor
			  */
			CDisk():m_radiusIn(0),m_radiusOut(1),m_nSlices(50),m_nLoops(4) {}

			CDisk(float rOut,float rIn,uint32_t slices,uint32_t loops):m_radiusIn(rIn),m_radiusOut(rOut),m_nSlices(slices),m_nLoops(loops)	{}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CDisk() { }
		};

	} // end namespace

} // End of namespace


#endif
