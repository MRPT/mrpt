/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <iostream>
#include <iterator>

namespace mrpt::slam::detail
{
using namespace mrpt;
using namespace mrpt::math;
using namespace std;

/** Auxiliary structure used in KLD-sampling in particle filters \sa
 * CPosePDFParticles, CMultiMetricMapPDF */
struct TPoseBin2D
{
	TPoseBin2D() = default;
	/** Bin indices */
	int x{0}, y{0}, phi{0};

	/** less-than ordering of bins for usage in STL containers */
	struct lt_operator
	{
		inline bool operator()(const TPoseBin2D& s1, const TPoseBin2D& s2) const
		{
			if (s1.x < s2.x) return true;
			if (s1.x > s2.x) return false;
			if (s1.y < s2.y) return true;
			if (s1.y > s2.y) return false;
			return s1.phi < s2.phi;
		}
	};
};

/** Auxiliary structure   */
struct TPathBin2D
{
	std::vector<TPoseBin2D> bins;

	/** less-than ordering of bins for usage in STL containers */
	struct lt_operator
	{
		bool operator()(const TPathBin2D& s1, const TPathBin2D& s2) const
		{
			ASSERT_(s1.bins.size() == s2.bins.size());
			for (size_t i = 0; i < s1.bins.size(); i++)
			{
				if (s1.bins[i].x < s2.bins[i].x) return true;
				if (s1.bins[i].x > s2.bins[i].x) return false;
				if (s1.bins[i].y < s2.bins[i].y) return true;
				if (s1.bins[i].y > s2.bins[i].y) return false;
				if (s1.bins[i].phi < s2.bins[i].phi) return true;
				if (s1.bins[i].phi > s2.bins[i].phi) return false;
				// else, keep comparing:
			}
			return false;  // If they're exactly equal, s1 is NOT < s2.
		}
	};
};

/** Auxiliary structure used in KLD-sampling in particle filters \sa
 * CPosePDFParticles, CMultiMetricMapPDF */
struct TPoseBin3D
{
	TPoseBin3D() = default;
	/** Bin indices */
	int x{0}, y{0}, z{0}, yaw{0}, pitch{0}, roll{0};

	/** less-than ordering of bins for usage in STL containers */
	struct lt_operator
	{
		bool operator()(const TPoseBin3D& s1, const TPoseBin3D& s2) const
		{
			if (s1.x < s2.x) return true;
			if (s1.x > s2.x) return false;
			if (s1.y < s2.y) return true;
			if (s1.y > s2.y) return false;
			if (s1.z < s2.z) return true;
			if (s1.z > s2.z) return false;
			if (s1.yaw < s2.yaw) return true;
			if (s1.yaw > s2.yaw) return false;
			if (s1.pitch < s2.pitch) return true;
			if (s1.pitch > s2.pitch) return false;
			return s1.roll < s2.roll;
		}
	};
};

}  // namespace mrpt::slam::detail
