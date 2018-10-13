/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPoseInterpolatorBase.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
/** This class stores a time-stamped trajectory in SE(3) (CPose3D poses).
 *  It can also interpolate SE(3) poses over time using linear, splines or
 * SLERP interpolation, as set in CPose3DInterpolator::setInterpolationMethod()
 *  Usage:
 *   - Insert new poses into the sequence with CPose3DInterpolator::insert()
 *   - Query an exact/interpolated pose with
 * CPose3DInterpolator::interpolate().
 * Example:
 * \code
 * CPose3DInterpolator		path;
 *
 * path.setInterpolationMethod( mrpt::poses::imSplineSlerp );
 *
 * path.insert( t0, mrpt::poses::CPose3D(...) );
 * path.insert( t1, mrpt::math::TPose3D(...) ); // prefered (faster)
 *
 * mrpt::math::TPose3D p;
 * bool valid;
 *
 * cout << "Pose at t: " << path.interpolate(t,p,valid).asString() << endl;
 * \endcode
 *
 *  Time is represented with mrpt::Clock::time_point. See mrpt::system for
 * methods and utilities to manage these time references.
 *
 *  See TInterpolatorMethod for the list of interpolation methods. The default
 * method at constructor is "imLinearSlerp".
 *
 * \sa CPoseOrPoint
 * \ingroup interpolation_grp poses_grp
 */
class CPose3DInterpolator : public mrpt::serialization::CSerializable,
							public mrpt::poses::CPoseInterpolatorBase<3>
{
	DEFINE_SERIALIZABLE(CPose3DInterpolator)
};  // End of class def.
}  // namespace mrpt::poses
