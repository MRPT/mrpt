/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/common.h>  // MRPT_IS_X86_AMD64
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/static_string.h>

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
 *
 * \note Before MRPT 2.4.0, "local" and "global" points were named "other", and
 * "this", respectively, and were not packed into a TPoint structure.
 */
template <typename T>
struct TMatchingPairTempl
{
	TMatchingPairTempl() = default;

	TMatchingPairTempl(
		uint32_t _globalIdx, uint32_t _localIdx, T _global_x, T _global_y,
		T _global_z, T _local_x, T _local_y, T _local_z)
		: globalIdx(_globalIdx),
		  localIdx(_localIdx),
		  global(_global_x, _global_y, _global_z),
		  local(_local_x, _local_y, _local_z)
	{
	}

	TMatchingPairTempl(
		uint32_t _globalIdx, uint32_t _localIdx,
		const mrpt::math::TPoint3D_<T>& _global,
		const mrpt::math::TPoint3D_<T>& _local)
		: globalIdx(_globalIdx),
		  localIdx(_localIdx),
		  global(_global),
		  local(_local)
	{
	}

	uint32_t globalIdx{0};
	uint32_t localIdx{0};
	mrpt::math::TPoint3D_<T> global{0, 0, 0};
	mrpt::math::TPoint3D_<T> local{0, 0, 0};
	T errorSquareAfterTransformation{0};

	void print(std::ostream& o) const;

	constexpr static auto getClassName()
	{
		using namespace mrpt::typemeta;
		return literal("TMatchingPairTempl<") + TTypeName<T>::get() +
			literal(">");
	}
};

template <typename T>
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const TMatchingPairTempl<T>& obj)
{
	out << obj.globalIdx << obj.localIdx << obj.global.x << obj.global.y
		<< obj.global.z << obj.local.x << obj.local.y << obj.local.z
		<< obj.errorSquareAfterTransformation;
	return out;
}

template <typename T>
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, TMatchingPairTempl<T>& obj)
{
	in >> obj.globalIdx >> obj.localIdx >> obj.global.x >> obj.global.y >>
		obj.global.z >> obj.local.x >> obj.local.y >> obj.local.z >>
		obj.errorSquareAfterTransformation;
	return in;
}

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

/** Comparison operator, first sorts by globalIdx, if equals, by localIdx
 */
template <typename T>
bool operator<(const TMatchingPairTempl<T>& a, const TMatchingPairTempl<T>& b)
{
	if (a.globalIdx == b.globalIdx) return (a.localIdx < b.localIdx);
	else
		return (a.globalIdx < b.globalIdx);
}

/** A comparison operator  */
template <typename T>
bool operator==(const TMatchingPairTempl<T>& a, const TMatchingPairTempl<T>& b)
{
	return (a.globalIdx == b.globalIdx) && (a.localIdx == b.localIdx);
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
