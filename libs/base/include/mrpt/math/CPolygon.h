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
#ifndef CPOLYGON_H
#define CPOLYGON_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
namespace math
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPolygon, mrpt::utils::CSerializable )

	/** A wrapper of a TPolygon2D class, implementing CSerializable.
	 * \ingroup geometry_grp
	 */
	class BASE_IMPEXP CPolygon : public mrpt::utils::CSerializable, public mrpt::math::TPolygon2D
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPolygon )

	public:
		/** Constructor
		  *  cx and cy are the "central" point coordinates (laser sensor location if applicable)
		  *  This parameters are NOT used in PointIntoPolygon, so they can be ignored.
		  * \sa PointIntoPolygon
		 */
		CPolygon() : TPolygon2D()
		{
		}

		/** Add a new vertex to polygon: */
		void  AddVertex(double x,double y) {
			TPolygon2D::push_back(TPoint2D(x,y));
		}

		/** Methods for accessing the vertexs:
		  * \sa verticesCount
		  */
		double  GetVertex_x(size_t i) const { ASSERT_(i<TPolygon2D::size()); return TPolygon2D::operator [](i).x; }
		double  GetVertex_y(size_t i) const { ASSERT_(i<TPolygon2D::size()); return TPolygon2D::operator [](i).y; }

		/** Returns the vertices count in the polygon: */
		size_t   verticesCount() const { return TPolygon2D::size(); }

		/** Set all vertices at once. */
		void  setAllVertices( const std::vector<double> &x, const std::vector<double> &y );
		/** Set all vertices at once. Please use the std::vector version whenever possible unless efficiency is really an issue */
		void  setAllVertices( size_t nVertices, const double *xs, const double *ys );
		/** Set all vertices at once. Please use the std::vector version whenever possible unless efficiency is really an issue */
		void  setAllVertices( size_t nVertices, const float *xs, const float *ys );

		/** Get all vertices at once. */
		void  getAllVertices( std::vector<double> &x, std::vector<double> &y ) const;

		/** Clear the polygon, erasing all vertexs. */
		void   Clear() { TPolygon2D::clear(); }

		/**	Check if a point is inside the polygon:
		 */
		bool  PointIntoPolygon(double x,double y) const {
			return TPolygon2D::contains(TPoint2D(x,y));
		}

	};

	} // End of namespace
} // End of namespace
#endif
