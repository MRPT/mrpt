/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/kmeans.h>

// This file is just a stub for the k-means++ library so MRPT users don't need
//  to include those headers too.

// Include the kmeans++ library, by David Arthur (darthur@gmail.com), 2009:
#include "kmeans++/KMeans.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;

namespace mrpt
{
	namespace math
	{
		namespace detail
		{
			/* -------------------------------------------
							 internal_kmeans
			   ------------------------------------------- */
			template <> BASE_IMPEXP
			double internal_kmeans<double>(
				const bool use_kmeansplusplus_method,
				const size_t nPoints,
				const size_t k,
				const size_t dims,
				const double *points,
				const size_t attempts,
				double* out_center,
				int *out_assignments)
			{
            MRPT_UNUSED_PARAM(use_kmeansplusplus_method);
				return RunKMeans(nPoints,k,dims,const_cast<double*>(points),attempts,out_center,out_assignments);
			}

			template <> BASE_IMPEXP
			double internal_kmeans<float>(
				const bool use_kmeansplusplus_method,
				const size_t nPoints,
				const size_t k,
				const size_t dims,
				const float *points,
				const size_t attempts,
				float* out_center,
				int *out_assignments)
			{
            MRPT_UNUSED_PARAM(use_kmeansplusplus_method);
				std::vector<double>  points_d(nPoints*dims);
				std::vector<double>  centers_d(k*dims);
				// Convert: float -> double
				for (size_t i=0;i<nPoints*dims;i++)
					points_d[i] = double(points[i]);

				const double ret = RunKMeans(nPoints,k,dims,&points_d[0],attempts,&centers_d[0],out_assignments);

				// Convert: double -> float
				if (out_center)
					for (size_t i=0;i<k*dims;i++)
						out_center[i] = float(centers_d[i]);

				return ret;
			}

		}
	}
}

