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
#ifndef opengl_CTexturedPlane_H
#define opengl_CTexturedPlane_H

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CTexturedPlane;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CTexturedPlane, CTexturedObject, OPENGL_IMPEXP )

		/** A 2D plane in the XY plane with a texture image.
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CTexturedPlane : public CTexturedObject
		{
			DEFINE_SERIALIZABLE( CTexturedPlane )
		protected:
			mutable float				m_tex_x_min,m_tex_x_max;
			mutable float				m_tex_y_min,m_tex_y_max;

			float			m_xMin, m_xMax;
			float			m_yMin, m_yMax;

			mutable bool polygonUpToDate;
			mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPoly;   //!< Used for ray-tracing
			void updatePoly() const;
			void unloadTexture();

			void  render_texturedobj() const;

		public:
			/** Set the texture coordinates of the four corners (in the range 0-1). */
			void setTextureCornerCoords( float tex_x_min, float tex_x_max, float tex_y_min, float tex_y_max)
			{
				m_tex_x_min=tex_x_min;
				m_tex_x_max=tex_x_max;
				m_tex_y_min=tex_y_min;
				m_tex_y_max=tex_y_max;
				CRenderizableDisplayList::notifyChange();
			}

			/** Set the coordinates of the four corners that define the plane on the XY plane. */
			void setPlaneCorners(float xMin, float xMax, float yMin, float yMax)
			{
				m_xMin = xMin; m_xMax = xMax;
				m_yMin = yMin; m_yMax = yMax;
				polygonUpToDate=false;
				CRenderizableDisplayList::notifyChange();
			}

			/** Get the coordinates of the four corners that define the plane on the XY plane. */
			inline void getPlaneCorners(float &xMin, float &xMax, float &yMin, float &yMax) const
			{
				xMin = m_xMin; xMax = m_xMax;
				yMin = m_yMin; yMax = m_yMax;
			}

			/** Class factory  */
			static CTexturedPlanePtr Create(
				float				x_min,
				float				x_max,
				float				y_min,
				float				y_max)
			{
				return CTexturedPlanePtr( new CTexturedPlane(x_min, x_max, y_min, y_max) );
			}

			/** Ray trace
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

		private:
			/** Constructor
			  */
			CTexturedPlane(
				float				x_min = -1,
				float				x_max = 1,
				float				y_min = -1,
				float				y_max = 1
				);

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CTexturedPlane();
		};

	} // end namespace

} // End of namespace


#endif
