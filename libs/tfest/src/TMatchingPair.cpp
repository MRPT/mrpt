/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "tfest-precomp.h"	// Precompiled headers
//
#include <mrpt/core/format.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/os.h>
#include <mrpt/tfest/TMatchingPair.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>	// accumulate()

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::tfest;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

template <typename T>
void TMatchingPairListTempl<T>::dumpToFile(const std::string& fileName) const
{
	std::ofstream f(fileName);
	ASSERT_(f.is_open());
	for (const auto& it : *this)
	{
		f << mrpt::format(
			"%u %u %f %f %f %f %f %f %f\n", it.globalIdx, it.localIdx,
			it.global.x, it.global.y, it.global.z, it.local.x, it.local.y,
			it.local.z, it.errorSquareAfterTransformation);
	}
}

template <typename T>
void TMatchingPairListTempl<T>::saveAsMATLABScript(
	const std::string& filName) const
{
	FILE* f = os::fopen(filName.c_str(), "wt");
	if (!f) return;

	fprintf(f, "%% ----------------------------------------------------\n");
	fprintf(f, "%%  File generated automatically by the MRPT method:   \n");
	fprintf(f, "%%   saveAsMATLABScript                                \n");
	fprintf(f, "%%  Before calling this script, define line color:     \n");
	fprintf(f, "%%     colorLines=[0.5 0.5 0.5]                        \n");
	fprintf(f, "%% ----------------------------------------------------\n\n");

	fprintf(f, "axis equal; hold on;\n");
	for (const auto& it : *this)
	{
		fprintf(
			f, "line([%f %f %f],[%f %f %f],'Color',colorLines);\n", it.global.x,
			it.local.x, it.local.z, it.global.y, it.local.y, it.local.z);
		fprintf(
			f,
			"set(plot([%f %f %f],[%f %f "
			"%f],'.'),'Color',colorLines,'MarkerSize',15);\n",
			it.global.x, it.local.x, it.local.z, it.global.y, it.local.y,
			it.local.z);
	}
	fprintf(f, "view(3); grid on; xlabel('x'); ylabel('y'); zlabel('z');");
	os::fclose(f);
}

template <typename T>
bool TMatchingPairListTempl<T>::indexOtherMapHasCorrespondence(size_t idx) const
{
	for (const auto& it : *this)
		if (it.localIdx == idx) return true;
	return false;
}

template <typename T>
T TMatchingPairListTempl<T>::overallSquareError(const CPose2D& q) const
{
	vector<T> errs(base_t::size());
	squareErrorVector(q, errs);
	return std::accumulate(errs.begin(), errs.end(), T(0));
}

template <typename T>
T TMatchingPairListTempl<T>::overallSquareErrorAndPoints(
	const CPose2D& q, vector<T>& xs, vector<T>& ys) const
{
	vector<T> errs(base_t::size());
	squareErrorVector(q, errs, xs, ys);
	return std::accumulate(errs.begin(), errs.end(), T(0));
}

template <typename T>
bool TMatchingPairListTempl<T>::contains(const TMatchingPairTempl<T>& p) const
{
	for (const auto& corresp : *this)
		if (corresp == p) return true;
	return false;
}

template <typename T>
void TMatchingPairListTempl<T>::squareErrorVector(
	const CPose2D& q, vector<T>& out_sqErrs) const
{
	out_sqErrs.resize(base_t::size());
	// *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]

	const T ccos = static_cast<T>(std::cos(q.phi()));
	const T csin = static_cast<T>(std::sin(q.phi()));
	const T qx = static_cast<T>(q.x());
	const T qy = static_cast<T>(q.y());

	typename base_t::const_iterator corresp;
	typename vector<T>::iterator e_i;
	for (corresp = base_t::begin(), e_i = out_sqErrs.begin();
		 corresp != base_t::end(); ++corresp, ++e_i)
	{
		T xx = qx + ccos * corresp->local.x - csin * corresp->local.y;
		T yy = qy + csin * corresp->local.x + ccos * corresp->local.y;
		*e_i = square(corresp->global.x - xx) + square(corresp->global.y - yy);
	}
}

template <typename T>
void TMatchingPairListTempl<T>::squareErrorVector(
	const CPose2D& q, vector<T>& out_sqErrs, vector<T>& xs, vector<T>& ys) const
{
	out_sqErrs.resize(base_t::size());
	xs.resize(base_t::size());
	ys.resize(base_t::size());

	// *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]

	const T ccos = static_cast<T>(std::cos(q.phi()));
	const T csin = static_cast<T>(std::sin(q.phi()));
	const T qx = static_cast<T>(q.x());
	const T qy = static_cast<T>(q.y());

	typename base_t::const_iterator corresp;
	typename vector<T>::iterator e_i, xx, yy;
	for (corresp = base_t::begin(), e_i = out_sqErrs.begin(), xx = xs.begin(),
		yy = ys.begin();
		 corresp != base_t::end(); ++corresp, ++e_i, ++xx, ++yy)
	{
		*xx = qx + ccos * corresp->local.x - csin * corresp->local.y;
		*yy = qy + csin * corresp->local.x + ccos * corresp->local.y;
		*e_i =
			square(corresp->global.x - *xx) + square(corresp->global.y - *yy);
	}
}

template <typename T>
void TMatchingPairListTempl<T>::filterUniqueRobustPairs(
	const size_t num_elements_this_map,
	TMatchingPairListTempl<T>& out_filtered_list) const
{
	using TMatchingPairConstPtr = TMatchingPairTempl<T> const*;

	std::vector<TMatchingPairConstPtr> bestMatchForThisMap(
		num_elements_this_map, TMatchingPairConstPtr(nullptr));
	out_filtered_list.clear();

	// 1) Go through all the correspondences and keep the best corresp.
	//    for each "global map" (this) point.
	for (auto& c : *this)
	{
		if (bestMatchForThisMap[c.globalIdx] == nullptr ||	// first one
			c.errorSquareAfterTransformation <
				bestMatchForThisMap[c.globalIdx]
					->errorSquareAfterTransformation  // or better
		)
		{ bestMatchForThisMap[c.globalIdx] = &c; }
	}

	//   2) Go again through the list of correspondences and remove those
	//       who are not the best one for their corresponding global map.
	for (auto& c : *this)
	{
		if (bestMatchForThisMap[c.globalIdx] == &c)
			out_filtered_list.push_back(c);	 // Add to the output
	}
}

template <typename T>
void TMatchingPairTempl<T>::print(std::ostream& o) const
{
	o << "[" << globalIdx << "->" << localIdx << "]"
	  << ": "
	  << "(" << global.x << "," << global.y << "," << global.z << ")"
	  << " -> "
	  << "(" << local.x << "," << local.y << "," << local.z << ")";
}

// Explicit instantations:
namespace mrpt::tfest
{
template class TMatchingPairListTempl<float>;
template class TMatchingPairListTempl<double>;

template struct TMatchingPairTempl<float>;
template struct TMatchingPairTempl<double>;

}  // namespace mrpt::tfest
