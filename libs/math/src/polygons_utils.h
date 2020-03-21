/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/geometry.h>  // distance()

namespace mrpt::math
{
/**  Auxiliary functor class to compute polygon's center */
template <class T, int N>
class FAddPoint
{
   public:
	T& object;
	FAddPoint(T& o) : object(o)
	{
		for (size_t i = 0; i < N; i++) object[i] = 0.0;
	}
	void operator()(const T& o)
	{
		for (size_t i = 0; i < N; i++) object[i] += o[i];
	}
};

template <class T>
inline void removeUnusedVertices(T& poly)
{
	size_t N = poly.size();
	if (N < 3) return;
	std::vector<size_t> unused;
	if (std::abs(
			mrpt::math::distance(poly[N - 1], poly[0]) +
			mrpt::math::distance(poly[0], poly[1]) -
			mrpt::math::distance(poly[N - 1], poly[1])) <
		mrpt::math::getEpsilon())
		unused.push_back(0);
	for (size_t i = 1; i < N - 1; i++)
		if (std::abs(
				mrpt::math::distance(poly[i - 1], poly[i]) +
				mrpt::math::distance(poly[i], poly[i + 1]) -
				mrpt::math::distance(poly[i - 1], poly[i + 1])) <
			mrpt::math::getEpsilon())
			unused.push_back(i);
	if (std::abs(
			mrpt::math::distance(poly[N - 2], poly[N - 1]) +
			mrpt::math::distance(poly[N - 1], poly[0]) -
			mrpt::math::distance(poly[N - 2], poly[0])) <
		mrpt::math::getEpsilon())
		unused.push_back(N - 1);
	unused.push_back(N);
	size_t diff = 1;
	for (size_t i = 0; i < unused.size() - 1; i++)
	{
		size_t last = unused[i + 1];
		for (size_t j = unused[i] + 1 - diff; j < last - diff; j++)
			poly[j] = poly[j + diff];
	}
	poly.resize(N + 1 - unused.size());
}
template <class T>
inline void removeRepVertices(T& poly)
{
	size_t N = poly.size();
	if (N < 3) return;
	std::vector<size_t> rep;
	for (size_t i = 0; i < N - 1; i++)
		if (mrpt::math::distance(poly[i], poly[i + 1]) < getEpsilon())
			rep.push_back(i);
	if (mrpt::math::distance(poly[N - 1], poly[0]) < getEpsilon())
		rep.push_back(N - 1);
	rep.push_back(N);
	size_t diff = 1;
	for (size_t i = 0; i < rep.size() - 1; i++)
	{
		size_t last = rep[i + 1];
		for (size_t j = rep[i] + 1 - diff; j < last - diff; j++)
			poly[j] = poly[j + diff];
	}
	poly.resize(N + 1 - rep.size());
}

}  // namespace mrpt::math
