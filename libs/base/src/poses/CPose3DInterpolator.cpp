/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DInterpolator.h>
#include "CPoseInterpolatorBase.hpp"  // templ impl
#include <mrpt/utils/stl_serialization.h>

using namespace mrpt::utils;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CPose3DInterpolator, CSerializable, mrpt::poses)

void  CPose3DInterpolator::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << m_path; // v1: change container element CPose3D->TPose3D
	}
}

void  CPose3DInterpolator::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			std::map<mrpt::system::TTimeStamp, mrpt::poses::CPose3D> old_path;
			in >> old_path;
			m_path.clear();
			for (const auto &p: old_path) {
				m_path[p.first] = mrpt::math::TPose3D(p.second);
			}
		}
	break;
	case 1:
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

// Specialization for DIM=3
template <>
void CPoseInterpolatorBase<3>::impl_interpolation(
	const mrpt::math::CArrayDouble<4> &ts,
	const TTimePosePair p1,const TTimePosePair p2,const TTimePosePair p3,const TTimePosePair p4,
	const TInterpolatorMethod method,double td,pose_t &out_interp) const
{
	using mrpt::math::TPose3D;
	mrpt::math::CArrayDouble<4> X,Y,Z,yaw,pitch,roll;
	X[0]	= p1.second.x;				Y[0]	= p1.second.y;					Z[0]	= p1.second.z;
	X[1]	= p2.second.x;				Y[1]	= p2.second.y;					Z[1]	= p2.second.z;
	X[2]	= p3.second.x;				Y[2]	= p3.second.y;					Z[2]	= p3.second.z;
	X[3]	= p4.second.x;				Y[3]	= p4.second.y;					Z[3]	= p4.second.z;

	yaw[0]  = p1.second.yaw;			pitch[0]  = p1.second.pitch;			roll[0]  = p1.second.roll;
	yaw[1]  = p2.second.yaw;			pitch[1]  = p2.second.pitch;			roll[1]  = p2.second.roll;
	yaw[2]  = p3.second.yaw;			pitch[2]  = p3.second.pitch;			roll[2]  = p3.second.roll;
	yaw[3]  = p4.second.yaw;			pitch[3]  = p4.second.pitch;			roll[3]  = p4.second.roll;

	unwrap2PiSequence(yaw);
	unwrap2PiSequence(pitch);
	unwrap2PiSequence(roll);

	// Target interpolated values:
	switch (method)
	{
	case imSpline:
		{
		// ---------------------------------------
		//    SPLINE INTERPOLATION
		// ---------------------------------------
		out_interp.x		= math::spline(td, ts, X);
		out_interp.y		= math::spline(td, ts, Y);
		out_interp.z		= math::spline(td, ts, Z);
		out_interp.yaw		= math::spline(td, ts, yaw,		true );	// Wrap 2pi
		out_interp.pitch	= math::spline(td, ts, pitch,	true );
		out_interp.roll	= math::spline(td, ts, roll,	true );
		}
		break;

	case imLinear2Neig:
		{
		out_interp.x		= math::interpolate2points(td, ts[1],X[1],ts[2],X[2]);
		out_interp.y		= math::interpolate2points(td, ts[1],Y[1],ts[2],Y[2]);
		out_interp.z		= math::interpolate2points(td, ts[1],Z[1],ts[2],Z[2]);
		out_interp.yaw		= math::interpolate2points(td, ts[1],yaw[1],ts[2],yaw[2],	true );	// Wrap 2pi
		out_interp.pitch	= math::interpolate2points(td, ts[1],pitch[1],ts[2],pitch[2],	true );
		out_interp.roll	= math::interpolate2points(td, ts[1],roll[1],ts[2],roll[2],	true );
		}
		break;

	case imLinear4Neig:
		{
		out_interp.x		= math::leastSquareLinearFit<double,decltype(ts), 4>(td, ts, X);
		out_interp.y		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Y);
		out_interp.z		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Z);
		out_interp.yaw		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, yaw,	true );	// Wrap 2pi
		out_interp.pitch	= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, pitch,	true );
		out_interp.roll		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, roll,	true );
		}
		break;

	case imSSLLLL:
		{
		out_interp.x		= math::spline(td, ts, X);
		out_interp.y		= math::spline(td, ts, Y);
		out_interp.z		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Z);
		out_interp.yaw		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, yaw,	true );	// Wrap 2pi
		out_interp.pitch	= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, pitch,	true );
		out_interp.roll	= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, roll,	true );
		}
		break;

	case imSSLSLL:
		{
		out_interp.x		= math::spline(td, ts, X);
		out_interp.y		= math::spline(td, ts, Y);
		out_interp.z		= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Z);
		out_interp.yaw		= math::spline(td, ts, yaw,	true );					// Wrap 2pi
		out_interp.pitch	= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, pitch,	true );
		out_interp.roll	= math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, roll,	true );
		}
		break;

	case imLinearSlerp:
		{
		const double ratio = (td-ts[1])/(ts[2]-ts[1]);
		mrpt::math::slerp_ypr(TPose3D(0,0,0,yaw[1], pitch[1], roll[1]), TPose3D(0, 0, 0, yaw[2], pitch[2], roll[2]), ratio, out_interp);

		out_interp.x = math::interpolate2points(td, ts[1], X[1], ts[2], X[2]);
		out_interp.y = math::interpolate2points(td, ts[1], Y[1], ts[2], Y[2]);
		out_interp.z = math::interpolate2points(td, ts[1], Z[1], ts[2], Z[2]);
	}
		break;

	case imSplineSlerp:
		{
		const double ratio = (td - ts[1]) / (ts[2] - ts[1]);
		mrpt::math::slerp_ypr(TPose3D(0, 0, 0, yaw[1], pitch[1], roll[1]), TPose3D(0, 0, 0, yaw[2], pitch[2], roll[2]), ratio, out_interp);

		out_interp.x = math::spline(td, ts, X);
		out_interp.y = math::spline(td, ts, Y);
		out_interp.z = math::spline(td, ts, Z);
		}
		break;

	default: THROW_EXCEPTION("Unknown value for interpolation method!");
	}; // end switch
}


// Explicit instantations:
template class BASE_IMPEXP CPoseInterpolatorBase<3>;
}
}


