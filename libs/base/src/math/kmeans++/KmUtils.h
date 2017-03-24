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
// Utilities for arbitrary dimensional points in space. All points are treated as simple value
// arrays. This is done for two reasons:
//   - Using value arrays instead of a point class makes all point operations very explicit, which
//     makes their usage easier to optimize.
//   - A value array is about as universal a format as possible, which makes it easier for
//     people to use the k-means code in any project.
// Also contains assertion code that can be disabled if desired.
//
// Author: David Arthur (darthur@gmail.com), 2009

#ifndef KM_UTILS_H__
#define KM_UTILS_H__

#include <mrpt/config.h> // For HAVE_MALLOC_H

// Includes
/* Jerome Monceaux : bilock@gmail.com
 * Add a specific case for apple
 */
#ifdef HAVE_MALLOC_H
# include <malloc.h>
#elif defined(HAVE_MALLOC_MALLOC_H)
# include <malloc/malloc.h>
#endif

#include <memory.h>
#include <cstdlib>

// The data-type used for a single coordinate for points
typedef double Scalar;

// Point utilities
// ===============

// Point creation and deletion
inline Scalar *PointAllocate(int d) {
  return (Scalar*)malloc(d * sizeof(Scalar));
}

inline void PointFree(Scalar *p) {
  free(p);
}

inline void PointCopy(Scalar *p1, const Scalar *p2, int d) {
  memcpy(p1, p2, d * sizeof(Scalar));
}

// Point vector tools
inline void PointAdd(Scalar *p1, const Scalar *p2, int d) {
  for (int i = 0; i < d; i++)
    p1[i] += p2[i];
}

inline void PointScale(Scalar *p, Scalar scale, int d) {
  for (int i = 0; i < d; i++)
    p[i] *= scale;
}

inline Scalar PointDistSq(const Scalar *p1, const Scalar *p2, int d) {
  Scalar result = 0;
  for (int i = 0; i < d; i++)
    result += (p1[i] - p2[i]) * (p1[i] - p2[i]);
  return result;
}

// Assertions
// ==========

// Comment out ENABLE_KMEANS_ASSERTS to turn off ASSERTS for added speed.
#define ENABLE_KMEANS_ASSERTS
#ifdef ENABLE_KMEANS_ASSERTS
int __KMeansAssertionFailure(const char *file, int line, const char *expression);
#define KM_ASSERT(expression) \
  (void)((expression) != 0? 0 : __KMeansAssertionFailure(__FILE__, __LINE__, #expression))
#else
#define KM_ASSERT(expression)
#endif

// Miscellaneous utilities
// =======================

// Returns a random integer chosen uniformly from the range [0, n-1]. Note that RAND_MAX could be
// less than n. On Visual Studio, it is only 32767. For larger values of RAND_MAX, we need to be
// careful of overflow.
inline int GetRandom(int n) {
  int u = rand() * RAND_MAX + rand();
  return ((u % n) + n) % n;
}

#endif
