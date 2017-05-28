/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose2DInterpolator.h>
#include "CPoseInterpolatorBase.hpp" // templ impl
#include <mrpt/utils/stl_serialization.h>

using namespace mrpt::utils;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CPose2DInterpolator, CSerializable, mrpt::poses)

void CPose2DInterpolator::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_path;
	}
}

void CPose2DInterpolator::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> m_path;
		}
	break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

namespace mrpt {
namespace poses {

// Specialization for DIM=2
template <>
void CPoseInterpolatorBase<2>::impl_interpolation(
	const mrpt::math::CArrayDouble<4> &ts,
	const TTimePosePair p1,const TTimePosePair p2,const TTimePosePair p3,const TTimePosePair p4,
	const TInterpolatorMethod method,double td,pose_t &out_interp) const
{
	using mrpt::math::TPose2D;
	mrpt::math::CArrayDouble<4> X,Y,yaw;
	X[0] = p1.second.x; Y[0] = p1.second.y;  yaw[0] = p1.second.phi;
	X[1] = p2.second.x;	Y[1] = p2.second.y;  yaw[1] = p2.second.phi;
	X[2] = p3.second.x;	Y[2] = p3.second.y;  yaw[2] = p3.second.phi;
	X[3] = p4.second.x;	Y[3] = p4.second.y;  yaw[3] = p4.second.phi;

	unwrap2PiSequence(yaw);

	// Target interpolated values:
	switch (method)
	{
	case imSpline:
		{
		// ---------------------------------------
		//    SPLINE INTERPOLATION
		// ---------------------------------------
		out_interp.x   = math::spline(td, ts, X);
		out_interp.y   = math::spline(td, ts, Y);
		out_interp.phi = math::spline(td, ts, yaw,		true );	// Wrap 2pi
		}
		break;

	case imLinear2Neig:
		{
		out_interp.x   = math::interpolate2points(td, ts[1],X[1],ts[2],X[2]);
		out_interp.y   = math::interpolate2points(td, ts[1],Y[1],ts[2],Y[2]);
		out_interp.phi = math::interpolate2points(td, ts[1],yaw[1],ts[2],yaw[2],	true );	// Wrap 2pi
		}
		break;

	case imLinear4Neig:
		{
		out_interp.x   = math::leastSquareLinearFit<double,decltype(ts), 4>(td, ts, X);
		out_interp.y   = math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Y);
		out_interp.phi = math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, yaw,	true );	// Wrap 2pi
		}
		break;

	case imSSLLLL:
		{
		out_interp.x		= math::spline(td, ts, X);
		out_interp.y		= math::spline(td, ts, Y);
		out_interp.phi		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, yaw,	true );	// Wrap 2pi
		}
		break;

	case imSSLSLL:
		{
		out_interp.x   = math::spline(td, ts, X);
		out_interp.y   = math::spline(td, ts, Y);
		out_interp.phi = math::spline(td, ts, yaw,	true );					// Wrap 2pi
		}
		break;

	case imLinearSlerp:
		{
		const double ratio = (td-ts[1])/(ts[2]-ts[1]);
		const double Aang  = mrpt::math::angDistance(yaw[1],yaw[2]);
		out_interp.phi = yaw[1] + ratio*Aang;

		out_interp.x = math::interpolate2points(td, ts[1], X[1], ts[2], X[2]);
		out_interp.y = math::interpolate2points(td, ts[1], Y[1], ts[2], Y[2]);
	}
		break;

	case imSplineSlerp:
		{
		const double ratio = (td-ts[1])/(ts[2]-ts[1]);
		const double Aang  = mrpt::math::angDistance(yaw[1],yaw[2]);
		out_interp.phi = yaw[1] + ratio*Aang;

		out_interp.x = math::spline(td, ts, X);
		out_interp.y = math::spline(td, ts, Y);
		}
		break;

	default: THROW_EXCEPTION("Unknown value for interpolation method!");
	}; // end switch
}

// Explicit instantations:
template class BASE_IMPEXP CPoseInterpolatorBase<2>;

}
}
