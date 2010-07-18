/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

/* -------------------------------------------
                 internal_kmeans
   ------------------------------------------- */
template <> BASE_IMPEXP  
double mrpt::math::detail::internal_kmeans<double>(
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

