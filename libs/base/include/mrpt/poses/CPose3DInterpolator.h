/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPoseInterpolatorBase.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace poses
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DInterpolator, mrpt::utils::CSerializable )

		/** This class stores a time-stamped trajectory in SE(3) (CPose3D poses). 
		  *  It can also interpolate SE(3) poses over time using linear, splines or SLERP interpolation, as set in CPose3DInterpolator::setInterpolationMethod()
		  *  Usage: 
		  *   - Insert new poses into the sequence with CPose3DInterpolator::insert()
		  *   - Query an exact/interpolated pose with CPose3DInterpolator::interpolate().
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
		  *  Time is represented with mrpt::system::TTimeStamp. See mrpt::system for methods and utilities to manage these time references.
		  *
		  *  See TInterpolatorMethod for the list of interpolation methods. The default method at constructor is "imLinearSlerp".
		  *
		  * \sa CPoseOrPoint
		 * \ingroup interpolation_grp poses_grp
		 */
		class BASE_IMPEXP CPose3DInterpolator :
			public mrpt::utils::CSerializable,
			public mrpt::poses::CPoseInterpolatorBase<3>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPose3DInterpolator )
		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPose3DInterpolator, mrpt::utils::CSerializable )

	} // End of namespace
} // End of namespace
