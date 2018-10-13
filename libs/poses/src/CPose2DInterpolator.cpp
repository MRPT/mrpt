/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose2DInterpolator.h>
#include "CPoseInterpolatorBase.hpp"  // templ impl
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CPose2DInterpolator, CSerializable, mrpt::poses)

uint8_t CPose2DInterpolator::serializeGetVersion() const { return 0; }
void CPose2DInterpolator::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_path;
}
void CPose2DInterpolator::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
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

namespace mrpt::poses
{
// Specialization for DIM=2
template <>
void CPoseInterpolatorBase<2>::impl_interpolation(
	const TTimePosePair& p1, const TTimePosePair& p2, const TTimePosePair& p3,
	const TTimePosePair& p4, const TInterpolatorMethod method,
	const mrpt::Clock::time_point& t, pose_t& out_interp) const
{
	using mrpt::math::TPose2D;
	using doubleDuration = std::chrono::duration<double>;
	doubleDuration durationT(t.time_since_epoch());
	double td = durationT.count();
	mrpt::math::CArrayDouble<4> ts;
	ts[0] =
		std::chrono::duration_cast<doubleDuration>(p1.first.time_since_epoch())
			.count();
	ts[1] =
		std::chrono::duration_cast<doubleDuration>(p2.first.time_since_epoch())
			.count();
	ts[2] =
		std::chrono::duration_cast<doubleDuration>(p3.first.time_since_epoch())
			.count();
	ts[3] =
		std::chrono::duration_cast<doubleDuration>(p4.first.time_since_epoch())
			.count();

	mrpt::math::CArrayDouble<4> X, Y, yaw;
	X[0] = p1.second.x;
	Y[0] = p1.second.y;
	yaw[0] = p1.second.phi;
	X[1] = p2.second.x;
	Y[1] = p2.second.y;
	yaw[1] = p2.second.phi;
	X[2] = p3.second.x;
	Y[2] = p3.second.y;
	yaw[2] = p3.second.phi;
	X[3] = p4.second.x;
	Y[3] = p4.second.y;
	yaw[3] = p4.second.phi;

	unwrap2PiSequence(yaw);

	// Target interpolated values:
	switch (method)
	{
		case imSpline:
		{
			// ---------------------------------------
			//    SPLINE INTERPOLATION
			// ---------------------------------------
			out_interp.x = math::spline(td, ts, X);
			out_interp.y = math::spline(td, ts, Y);
			out_interp.phi = math::spline(td, ts, yaw, true);  // Wrap 2pi
		}
		break;

		case imLinear2Neig:
		{
			out_interp.x =
				math::interpolate2points(td, ts[1], X[1], ts[2], X[2]);
			out_interp.y =
				math::interpolate2points(td, ts[1], Y[1], ts[2], Y[2]);
			out_interp.phi = math::interpolate2points(
				td, ts[1], yaw[1], ts[2], yaw[2], true);  // Wrap 2pi
		}
		break;

		case imLinear4Neig:
		{
			out_interp.x =
				math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, X);
			out_interp.y =
				math::leastSquareLinearFit<double, decltype(ts), 4>(td, ts, Y);
			out_interp.phi =
				math::leastSquareLinearFit<double, decltype(ts), 4>(
					td, ts, yaw, true);  // Wrap 2pi
		}
		break;

		case imSSLLLL:
		{
			out_interp.x = math::spline(td, ts, X);
			out_interp.y = math::spline(td, ts, Y);
			out_interp.phi =
				math::leastSquareLinearFit<double, decltype(ts), 4>(
					td, ts, yaw, true);  // Wrap 2pi
		}
		break;

		case imSSLSLL:
		{
			out_interp.x = math::spline(td, ts, X);
			out_interp.y = math::spline(td, ts, Y);
			out_interp.phi = math::spline(td, ts, yaw, true);  // Wrap 2pi
		}
		break;

		case imLinearSlerp:
		{
			const double ratio = (td - ts[1]) / (ts[2] - ts[1]);
			const double Aang = mrpt::math::angDistance(yaw[1], yaw[2]);
			out_interp.phi = yaw[1] + ratio * Aang;

			out_interp.x =
				math::interpolate2points(td, ts[1], X[1], ts[2], X[2]);
			out_interp.y =
				math::interpolate2points(td, ts[1], Y[1], ts[2], Y[2]);
		}
		break;

		case imSplineSlerp:
		{
			const double ratio = (td - ts[1]) / (ts[2] - ts[1]);
			const double Aang = mrpt::math::angDistance(yaw[1], yaw[2]);
			out_interp.phi = yaw[1] + ratio * Aang;

			out_interp.x = math::spline(td, ts, X);
			out_interp.y = math::spline(td, ts, Y);
		}
		break;

		default:
			THROW_EXCEPTION("Unknown value for interpolation method!");
	};  // end switch
}

// Explicit instantations:
template class CPoseInterpolatorBase<2>;
}  // namespace mrpt::poses
