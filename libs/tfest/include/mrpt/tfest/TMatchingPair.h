/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>
#include <iosfwd>
#include <cstdint>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/core/common.h>  // MRPT_IS_X86_AMD64

namespace mrpt::tfest
{
// Pragma defined to ensure no structure packing, so we can use SSE2
// vectorization on parts of this struture
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(push, 1)
#endif

/** A structure for holding correspondences between two sets of points or
 * points-like entities in 2D or 3D. Using `float` to save space since large
 * point clouds are likely stored in local coordinates using `float`.
 * \ingroup mrpt_base_grp
 */
struct TMatchingPair
{
	TMatchingPair() = default;

	TMatchingPair(
		uint32_t _this_idx, uint32_t _other_idx, float _this_x, float _this_y,
		float _this_z, float _other_x, float _other_y, float _other_z)
		: this_idx(_this_idx),
		  other_idx(_other_idx),
		  this_x(_this_x),
		  this_y(_this_y),
		  this_z(_this_z),
		  other_x(_other_x),
		  other_y(_other_y),
		  other_z(_other_z),
		  errorSquareAfterTransformation(0)
	{
	}

	uint32_t this_idx{0};
	uint32_t other_idx{0};
	float this_x{0}, this_y{0}, this_z{0};
	float other_x{0}, other_y{0}, other_z{0};
	float errorSquareAfterTransformation{0};
};

#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)  // End of pack = 1
#endif

using TMatchingPairPtr = TMatchingPair*;
using TMatchingPairConstPtr = TMatchingPair const*;

std::ostream& operator<<(
	std::ostream& o, const mrpt::tfest::TMatchingPair& pair);

/** A list of TMatchingPair
 * \ingroup mrpt_base_grp
 */
class TMatchingPairList : public std::vector<TMatchingPair>
{
   public:
	/** Checks if the given index from the "other" map appears in the list. */
	bool indexOtherMapHasCorrespondence(size_t idx) const;
	/** Saves the correspondences to a text file */
	void dumpToFile(const std::string& fileName) const;
	/** Saves the correspondences as a MATLAB script which draws them. */
	void saveAsMATLABScript(const std::string& filName) const;

	/** Computes the overall square error between the 2D points in the list of
	 * correspondences, given the 2D transformation "q"
	 *    \f[ \sum\limits_i e_i  \f]
	 *  Where \f$ e_i \f$ are the elements of the square error vector as
	 * computed by computeSquareErrorVector
	 * \sa squareErrorVector, overallSquareErrorAndPoints
	 */
	float overallSquareError(const mrpt::poses::CPose2D& q) const;

	/** Computes the overall square error between the 2D points in the list of
	 * correspondences, given the 2D transformation "q", and return the
	 * transformed points as well.
	 *    \f[ \sum\limits_i e_i  \f]
	 *  Where \f$ e_i \f$ are the elements of the square error vector as
	 * computed by computeSquareErrorVector
	 * \sa squareErrorVector
	 */
	float overallSquareErrorAndPoints(
		const mrpt::poses::CPose2D& q, std::vector<float>& xs,
		std::vector<float>& ys) const;

	/**  Returns a vector with the square error between each pair of
	 * correspondences in the list, given the 2D transformation "q"
	 *    Each element \f$ e_i \f$ is the square distance between the "this"
	 * (global) point and the "other" (local) point transformed through "q":
	 *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
	 * \sa overallSquareError
	 */
	void squareErrorVector(
		const mrpt::poses::CPose2D& q, std::vector<float>& out_sqErrs) const;

	/**  Returns a vector with the square error between each pair of
	 * correspondences in the list and the transformed "other" (local) points,
	 * given the 2D transformation "q"
	 *    Each element \f$ e_i \f$ is the square distance between the "this"
	 * (global) point and the "other" (local) point transformed through "q":
	 *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
	 * \sa overallSquareError
	 */
	void squareErrorVector(
		const mrpt::poses::CPose2D& q, std::vector<float>& out_sqErrs,
		std::vector<float>& xs, std::vector<float>& ys) const;

	/** Test whether the given pair "p" is within the pairings */
	bool contains(const TMatchingPair& p) const;

	/** Creates a filtered list of pairings with those ones which have a single
	 * correspondence which coincides
	 * in both directions, i.e. the best pairing of element `i` in map `this`
	 * is the best match for element `j` in map `other`,
	 * and viceversa*/
	void filterUniqueRobustPairs(
		const size_t num_elements_this_map,
		TMatchingPairList& out_filtered_list) const;
};

/** A comparison operator, for sorting lists of TMatchingPair's, first order by
 * this_idx, if equals, by other_idx   */
bool operator<(const TMatchingPair& a, const TMatchingPair& b);

/** A comparison operator  */
bool operator==(const TMatchingPair& a, const TMatchingPair& b);

/** A comparison operator */
bool operator==(const TMatchingPairList& a, const TMatchingPairList& b);

}  // namespace mrpt::tfest
