/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/math/CSplineInterpolator1D.h>
#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CSplineInterpolator1D, CSerializable, mrpt::math)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CSplineInterpolator1D::CSplineInterpolator1D( bool  wrap2pi ) : m_wrap2pi(wrap2pi)
{
}

/*---------------------------------------------------------------
						appendXY
  ---------------------------------------------------------------*/
void CSplineInterpolator1D::appendXY( double x, double y )
{
	m_x2y[x] = y;
}

/*---------------------------------------------------------------
						query
  ---------------------------------------------------------------*/
double & CSplineInterpolator1D::query( double x, double &y, bool &out_valid ) const
{
	out_valid = false;
	y=0;

	std::pair<double,double>	p1,p2,p3,p4;

	std::map<double,double>::const_iterator it_ge1 = m_x2y.lower_bound( x );

	// Exact match?
	if( it_ge1 != m_x2y.end() && it_ge1->first == x )
	{
		y = it_ge1->second;
		out_valid = true;
		return y;
	}

	// Are we in the beginning or the end of the path?
	if( it_ge1 == m_x2y.end()  || it_ge1 == m_x2y.begin() )
	{
		return y;
	}

	p3 = *it_ge1;		// Third pair
	++it_ge1;
	if( it_ge1 == m_x2y.end() )
	{
		return y;
	}
	p4 = *it_ge1;		// Fourth pair

	it_ge1--;
	it_ge1--;
	p2 = *it_ge1;	// Second pair

	if( it_ge1 == m_x2y.begin() )
	{
		return y;
	}

	p1 = *(--it_ge1);	// First pair

	// ---------------------------------------
	//    SPLINE INTERPOLATION
	// ---------------------------------------
	vector_double	xs(4);
	xs[0] = p1.first;
	xs[1] = p2.first;
	xs[2] = p3.first;
	xs[3] = p4.first;

	vector_double	ys(4);
	ys[0] = p1.second;
	ys[1] = p2.second;
	ys[2] = p3.second;
	ys[3] = p4.second;

	out_valid = true;
	return y = math::spline(x,xs, ys, m_wrap2pi);
}


/*---------------------------------------------------------------
	Implements the writing to a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSplineInterpolator1D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_x2y << m_wrap2pi;
	}

}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSplineInterpolator1D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:  // floats
		{
			in >> m_x2y >> m_wrap2pi;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

