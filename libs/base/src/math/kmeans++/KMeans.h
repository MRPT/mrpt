/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// BEWARE: BETA VERSION
// --------------------
//
// The main set of utilities for runnning k-means and k-means++ on arbitrary data sets.
//
// Author: David Arthur (darthur@gmail.com), 2009

#ifndef KMEANS_H__
#define KMEANS_H__

// Includes
#include "KmUtils.h"
#include <iostream>

// Sets preferences for how much logging is done and where it is outputted, when k-means is run.
void ClearKMeansLogging();
void AddKMeansLogging(std::ostream *out, bool verbose);

// Runs k-means on the given set of points.
//   - n: The number of points in the data set
//   - k: The number of clusters to look for
//   - d: The number of dimensions that the data set lives in
//   - points: An array of size n*d where points[d*i + j] gives coordinate j of point i
//   - attempts: The number of times to independently run k-means with different starting centers.
//               The best result is always returned (as measured by the cost function).
//   - centers: This can either be null or an array of size k*d. In the latter case, it will be
//              filled with the locations of all final cluster centers. Specifically
//              centers[d*i + j] will give coordinate j of center i. If the cluster is unused, it
//              will contain NaN instead.
//   - assignments: This can either be null or an array of size n. In the latter case, it will be
//                  filled with the cluster that each point is assigned to (an integer between 0
//                  and k-1 inclusive).
// The final cost of the clustering is also returned.
Scalar RunKMeans(int n, int k, int d, Scalar *points, int attempts,
                 Scalar *centers, int *assignments);

// Runs k-means++ on the given set of points. Set RunKMeans for info on the parameters.
Scalar RunKMeansPlusPlus(int n, int k, int d, Scalar *points, int attempts,
                         Scalar *centers, int *assignments);

#endif
