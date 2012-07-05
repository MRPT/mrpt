/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
