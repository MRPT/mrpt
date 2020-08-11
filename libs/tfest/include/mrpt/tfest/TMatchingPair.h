/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/common.h>  // MRPT_IS_X86_AMD64
#include <mrpt/poses/poses_frwds.h>

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace mrpt::tfest
{
/** \addtogroup mrpt_tfest_grp
 * @{ */

// Pragma defined to ensure no structure packing, so we can use SSE2
// vectorization on parts of this struture
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(push, 1)
#endif

/** A structure for holding correspondences between two sets of points or
 * points-like entities in 2D or 3D. Templatized version for double or float.
 */
template <typename T>
struct TMatchingPairTempl
{
	TMatchingPairTempl() = default;

	TMatchingPairTempl(
		uint32_t _this_idx, uint32_t _other_idx, T _this_x, T _this_y,
		T _this_z, T _other_x, T _other_y, T _other_z)
		: this_idx(_this_idx),
		  other_idx(_other_idx),
		  this_x(_this_x),
		  this_y(_this_y),
		  this_z(_this_z),
		  other_x(_other_x),
		  other_y(_other_y),
		  other_z(_other_z)
	{
	}

	uint32_t this_idx{0};
	uint32_t other_idx{0};
	T this_x{0}, this_y{0}, this_z{0};
	T other_x{0}, other_y{0}, other_z{0};
	T errorSquareAfterTransformation{0};

	void print(std::ostream& o) const;
};

/** A structure for holding correspondences between two sets of points or
 * points-like entities in 2D or 3D. Using `float` to save space since large
 * point clouds are likely stored in local coordinates using `float`.
 *
 * \sa TMatchingPair_d
 */
using TMatchingPair = TMatchingPairTempl<float>;

/** A structure for holding correspondences between two sets of points or
 * points-like entities in 2D or 3D. Using `double` for maximum accuracy when
 * required.
 *
 * \sa TMatchingPair
 */
using TMatchingPair_d = TMatchingPairTempl<double>;

#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)  // End of pack = 1
#endif

/** A list of TMatchingPair.
 */
template <typename T>
class TMatchingPairListTempl : public std::vector<TMatchingPairTempl<T>>
{
   public:
	using base_t = std::vector<TMatchingPairTempl<T>>;

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
	T overallSquareError(const mrpt::poses::CPose2D& q) const;

	/** Computes the overall square error between the 2D points in the list of
	 * correspondences, given the 2D transformation "q", and return the
	 * transformed points as well.
	 *    \f[ \sum\limits_i e_i  \f]
	 *  Where \f$ e_i \f$ are the elements of the square error vector as
	 * computed by computeSquareErrorVector
	 * \sa squareErrorVector
	 */
	T overallSquareErrorAndPoints(
		const mrpt::poses::CPose2D& q, std::vector<T>& xs,
		std::vector<T>& ys) const;

	/**  Returns a vector with the square error between each pair of
	 * correspondences in the list, given the 2D transformation "q"
	 *    Each element \f$ e_i \f$ is the square distance between the "this"
	 * (global) point and the "other" (local) point transformed through "q":
	 *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
	 * \sa overallSquareError
	 */
	void squareErrorVector(
		const mrpt::poses::CPose2D& q, std::vector<T>& out_sqErrs) const;

	/**  Returns a vector with the square error between each pair of
	 * correspondences in the list and the transformed "other" (local) points,
	 * given the 2D transformation "q"
	 *    Each element \f$ e_i \f$ is the square distance between the "this"
	 * (global) point and the "other" (local) point transformed through "q":
	 *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]
	 * \sa overallSquareError
	 */
	void squareErrorVector(
		const mrpt::poses::CPose2D& q, std::vector<T>& out_sqErrs,
		std::vector<T>& xs, std::vector<T>& ys) const;

	/** Test whether the given pair "p" is within the pairings */
	bool contains(const TMatchingPairTempl<T>& p) const;

	/** Creates a filtered list of pairings with those ones which have a single
	 * correspondence which coincides
	 * in both directions, i.e. the best pairing of element `i` in map `this`
	 * is the best match for element `j` in map `other`,
	 * and viceversa*/
	void filterUniqueRobustPairs(
		const size_t num_elements_this_map,
		TMatchingPairListTempl<T>& out_filtered_list) const;
};

template <typename T>
std::ostream& operator<<(
	std::ostream& o, const mrpt::tfest::TMatchingPairTempl<T>& pairs)
{
	pairs.print(o);
	return o;
}

/** Comparison operator, first sorts by this_idx, if equals, by other_idx
 */
template <typename T>
bool operator<(const TMatchingPairTempl<T>& a, const TMatchingPairTempl<T>& b)
{
	if (a.this_idx == b.this_idx)
		return (a.this_idx < b.this_idx);
	else
		return (a.other_idx < b.other_idx);
}

/** A comparison operator  */
template <typename T>
bool operator==(const TMatchingPairTempl<T>& a, const TMatchingPairTempl<T>& b)
{
	return (a.this_idx == b.this_idx) && (a.other_idx == b.other_idx);
}

/** A comparison operator */
template <typename T>
bool operator==(
	const TMatchingPairListTempl<T>& a, const TMatchingPairListTempl<T>& b)
{
	if (a.size() != b.size()) return false;
	for (auto it1 = a.begin(), it2 = b.begin(); it1 != a.end(); ++it1, ++it2)
		if (!((*it1) == (*it2))) return false;
	return true;
}

/** A list of TMatchingPair (T=float).  */
using TMatchingPairList = TMatchingPairListTempl<float>;

/** A list of TMatchingPair (T=double). */
using TMatchingPairList_d = TMatchingPairListTempl<double>;

/** @} */

}  // namespace mrpt::tfest
