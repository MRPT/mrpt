/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

