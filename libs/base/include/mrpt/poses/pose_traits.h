/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef POSE_TRAITS_H
#define POSE_TRAITS_H

#include <mrpt/poses/poses_frwds.h>

namespace mrpt { namespace traits {

/**\brief Type Traits for manipulating covariance and information form
 * Probability Density Function (PDF) classes
 *
 * \ingroup poses_grp
 */
template<class T>
struct is_inf_type {
	// false by default
	static const bool value = false;
};

template<>
struct is_inf_type< ::mrpt::poses::CPosePDFGaussianInf > {
	static const bool value = true;
};
template<>
struct is_inf_type< ::mrpt::poses::CPose3DQuatPDFGaussianInf> {
	static const bool value = false;
};
template<>
struct is_inf_type< ::mrpt::poses::CPose3DPDFGaussianInf> {
	static const bool value = true;
};

} } // end of namespaces


#endif /* end of include guard: POSE_TRAITS_H */
