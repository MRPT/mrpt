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
#ifndef opengl_CSphere_H
#define opengl_CSphere_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CSphere;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSphere, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A solid or wire-frame sphere.
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CSphere </td> <td> \image html preview_CSphere.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CSphere : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CSphere )

		protected:
			float			m_radius;
			int				m_nDivsLongitude,m_nDivsLatitude;
			bool			m_keepRadiusIndependentEyeDistance;

		public:
			void setRadius(float r) { m_radius=r; CRenderizableDisplayList::notifyChange(); }
			float getRadius() const {return m_radius; }

			void setNumberDivsLongitude(int N) { m_nDivsLongitude=N; CRenderizableDisplayList::notifyChange(); }
			void setNumberDivsLatitude(int N) { m_nDivsLatitude=N;  CRenderizableDisplayList::notifyChange();}
			void enableRadiusIndependentOfEyeDistance(bool v=true)  { m_keepRadiusIndependentEyeDistance=v; CRenderizableDisplayList::notifyChange(); }

			/** \sa CRenderizableDisplayList */
			virtual bool should_skip_display_list_cache() const { return m_keepRadiusIndependentEyeDistance; }

			/** Class factory  */
			static CSpherePtr Create(
				float				radius,
				int					nDivsLongitude = 20,
				int					nDivsLatitude = 20 )
			{
				return CSpherePtr( new CSphere(radius,nDivsLongitude,nDivsLatitude) );
			}

			/** Render */
			void  render_dl() const;

			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

		private:
			/** Constructor
			  */
			CSphere(
				float				radius = 1.0f,
				int					nDivsLongitude = 20,
				int					nDivsLatitude = 20
				) :
				m_radius(radius),
				m_nDivsLongitude(nDivsLongitude),
				m_nDivsLatitude(nDivsLatitude),
				m_keepRadiusIndependentEyeDistance(false)
			{
			}

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CSphere() { }
		};

	} // end namespace

} // End of namespace

#endif
