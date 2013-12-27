/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/SE_traits.h>

namespace mrpt { namespace srba {
namespace kf2kf_poses
{
	/** \defgroup mrpt_srba_kf2kf KF-to-KF relative pose parameterizations
		* \ingroup mrpt_srba_grp */

	/** \addtogroup mrpt_srba_kf2kf
		* @{ */


	struct SE3
	{
		static const size_t REL_POSE_DIMS = 6;  //!< Each relative pose is parameterized as a CPose3D()
		typedef mrpt::poses::CPose3D       pose_t;  //!< The pose class
		typedef mrpt::poses::SE_traits<3>  se_traits_t;  //!< The SE(3) traits struct (for Lie algebra log/exp maps, etc.)
	};

	struct SE2
	{
		static const size_t REL_POSE_DIMS = 3;  //!< Each relative pose is parameterized as a CPose3D()
		typedef mrpt::poses::CPose2D   pose_t;  //!< The pose class
		typedef mrpt::poses::SE_traits<2>  se_traits_t;  //!< The SE(2) traits struct (for Lie algebra log/exp maps, etc.)
	};

	/** @} */

}
} } // end NS
