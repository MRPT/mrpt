/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers



#include <mrpt/math/CPolygon.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CPolygon, CSerializable, mrpt::math)


/*---------------------------------------------------------------
	Implements the writing to a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPolygon::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 2;
	else
	{
		// The number of vertexs:
		const uint32_t n = (uint32_t)TPolygon2D::size();

		// Size:
		out << n;

		// Vertices:
		if (n)
			out.WriteBufferFixEndianness<double>( (double*)&TPolygon2D::operator[](0), 2*n);
	}

}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPolygon::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:  // floats
		{
			// The number of vertexs:
			uint32_t	i,n;
			float		f;

			// All elemental typed variables:
			in >> n;
			in >> f; //max_x=f;
			in >> f; //max_y=f;
			in >> f; //min_x=f;
			in >> f; //min_y=f;
			in >> f; //cx=f;
			in >> f; //cy=f;

			TPolygon2D::resize(n);

			// The vertexs arrays:
			for (i=0;i<n;i++) { in >> f; TPolygon2D::operator[](i).x = f;	}
			for (i=0;i<n;i++) { in >> f; TPolygon2D::operator[](i).y = f;	}
		} break;

	case 1:
		{
			// The number of vertexs:
			uint32_t	n;
			double dumm;

			// All elemental typed variables:
			in >> n >> dumm >> dumm >> dumm >> dumm >> dumm >> dumm;
			        //max_x >> max_y >> min_x >> min_y >> cx >> cy;

			TPolygon2D::resize(n);

			// The vertexs arrays:
			for (size_t i=0;i<n;i++) { in >> dumm; TPolygon2D::operator[](i).x = dumm;	}
			for (size_t i=0;i<n;i++) { in >> dumm; TPolygon2D::operator[](i).y = dumm;	}
		} break;
	case 2:
		{
			// The number of vertexs:
			uint32_t	n;

			// All elemental typed variables:
			in >> n;

			TPolygon2D::resize(n);

			// The vertexs arrays:
			if (n>0)
				in.ReadBufferFixEndianness<double>( (double*)&TPolygon2D::operator[](0), 2*n);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
	Set all vertices at once, not to be used normally.
  ---------------------------------------------------------------*/
void  CPolygon::setAllVertices( const std::vector<double> &x, const std::vector<double> &y )
{
	ASSERT_(x.size()==y.size() && !x.empty());
	setAllVertices( x.size() ,&x[0],&y[0] );
}

/*---------------------------------------------------------------
	Set all vertices at once, not to be used normally.
  ---------------------------------------------------------------*/
void  CPolygon::setAllVertices( size_t nVertices, const double *xs, const double *ys )
{
	// Resize:
	TPolygon2D::resize(nVertices);
	for (size_t i=0;i<nVertices;i++)
	{
		TPolygon2D::operator[](i).x = xs[i];
		TPolygon2D::operator[](i).y = ys[i];
	}
}


/*---------------------------------------------------------------
	Set all vertices at once, not to be used normally.
  ---------------------------------------------------------------*/
void  CPolygon::setAllVertices( size_t nVertices, const float *xs, const float *ys )
{
	// Resize:
	TPolygon2D::resize(nVertices);
	for (size_t i=0;i<nVertices;i++)
	{
		TPolygon2D::operator[](i).x = xs[i];
		TPolygon2D::operator[](i).y = ys[i];
	}
}

void  CPolygon::getAllVertices( std::vector<double> &x, std::vector<double> &y ) const
{
	// Resize:
	const size_t n = TPolygon2D::size();
	x.resize(n);
	y.resize(n);
	for (size_t i=0;i<n;i++)
	{
		x[i] = TPolygon2D::operator[](i).x;
		y[i] = TPolygon2D::operator[](i).y;
	}
}
