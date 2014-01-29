/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once


namespace mrpt { namespace srba {

	/** Specify the kind of Jacobian to be used for compute_jacobian_dAepsDx_deps<> */
	enum landmark_jacob_family_t
	{
		jacob_point_landmark,   //!< Landmarks are points
		jacob_relpose_landmark  //!< "fake landmarks" used for relative pose observations
	};

} } // end NS

