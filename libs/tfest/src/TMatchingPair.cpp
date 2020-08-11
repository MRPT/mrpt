/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
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
			"%u %u %f %f %f %f %f %f %f\n", it.this_idx, it.other_idx,
			it.this_x, it.this_y, it.this_z, it.other_x, it.other_y, it.other_z,
			it.errorSquareAfterTransformation);
	}
}

template <typename T>
void TMatchingPairListTempl<T>::saveAsMATLABScript(
	const std::string& filName) const
{
	FILE* f = os::fopen(filName.c_str(), "wt");

	fprintf(f, "%% ----------------------------------------------------\n");
	fprintf(f, "%%  File generated automatically by the MRPT method:\n");
	fprintf(f, "%%   saveAsMATLABScript  \n");
	fprintf(
		f, "%%  Before calling this script, define the color of lines, eg:\n");
	fprintf(f, "%%     colorLines=[1 1 1]");
	fprintf(f, "%% ----------------------------------------------------\n\n");

	fprintf(f, "axis equal; hold on;\n");
	for (const auto& it : *this)
	{
		fprintf(
			f, "line([%f %f],[%f %f],'Color',colorLines);\n", it.this_x,
			it.other_x, it.this_y, it.other_y);
		fprintf(
			f,
			"set(plot([%f %f],[%f "
			"%f],'.'),'Color',colorLines,'MarkerSize',15);\n",
			it.this_x, it.other_x, it.this_y, it.other_y);
	}
	os::fclose(f);
}

template <typename T>
bool TMatchingPairListTempl<T>::indexOtherMapHasCorrespondence(size_t idx) const
{
	for (const auto& it : *this)
		if (it.other_idx == idx) return true;
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
		T xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		T yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square(corresp->this_x - xx) + square(corresp->this_y - yy);
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
		*xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		*yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square(corresp->this_x - *xx) + square(corresp->this_y - *yy);
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
		if (bestMatchForThisMap[c.this_idx] == nullptr ||  // first one
			c.errorSquareAfterTransformation <
				bestMatchForThisMap[c.this_idx]
					->errorSquareAfterTransformation  // or better
		)
		{
			bestMatchForThisMap[c.this_idx] = &c;
		}
	}

	//   2) Go again through the list of correspondences and remove those
	//       who are not the best one for their corresponding global map.
	for (auto& c : *this)
	{
		if (bestMatchForThisMap[c.this_idx] == &c)
			out_filtered_list.push_back(c);	 // Add to the output
	}
}

template <typename T>
void TMatchingPairTempl<T>::print(std::ostream& o) const
{
	o << "[" << this_idx << "->" << other_idx << "]"
	  << ": "
	  << "(" << this_x << "," << this_y << "," << this_z << ")"
	  << " -> "
	  << "(" << other_x << "," << other_y << "," << other_z << ")";
}

// Explicit instantations:
namespace mrpt::tfest
{
template class TMatchingPairListTempl<float>;
template class TMatchingPairListTempl<double>;

template class TMatchingPairTempl<float>;
template class TMatchingPairTempl<double>;

}  // namespace mrpt::tfest
