/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOLYGON_H
#define CPOLYGON_H

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
		/** Default constructor (empty polygon, 0 vertices) */
		CPolygon() : TPolygon2D()
		{ }

		/** Add a new vertex to polygon */
		void  AddVertex(double x,double y) {
			TPolygon2D::push_back(TPoint2D(x,y));
		}

		/** Methods for accessing the vertices \sa verticesCount */
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

		/** Get all vertices at once */
		void  getAllVertices( std::vector<double> &x, std::vector<double> &y ) const;

		/** Clear the polygon, erasing all vertices */
		void   Clear() { TPolygon2D::clear(); }

		/**	Check if a point is inside the polygon */
		bool  PointIntoPolygon(double x,double y) const {
			return TPolygon2D::contains(TPoint2D(x,y));
		}
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPolygon, mrpt::utils::CSerializable )

	} // End of namespace
} // End of namespace
#endif
