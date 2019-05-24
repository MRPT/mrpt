/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/math_frwds.h>
#include <algorithm>

/* Utility functions to ease the use of Eigen matrices */

namespace mrpt::math
{
namespace detail
{
/** Internal resize which compiles to nothing on fixed-size matrices. */
template <typename MAT, int TypeSizeAtCompileTime>
struct TAuxResizer
{
	static inline void internal_resize(MAT&, size_t, size_t) {}
	static inline void internal_resize(MAT&, size_t) {}
};
template <typename MAT>
struct TAuxResizer<MAT, -1>
{
	static inline void internal_resize(MAT& obj, size_t row, size_t col)
	{
		obj.derived().conservativeResize(row, col);
	}
	static inline void internal_resize(MAT& obj, size_t nsize)
	{
		obj.derived().conservativeResize(nsize);
	}
};

// Generic version for all kind of matrices:
template <int R, int C>
struct MatOrVecResizer
{
	template <typename MAT>
	static inline void doit(MAT& mat, size_t new_rows, size_t new_cols)
	{
		::mrpt::math::detail::TAuxResizer<MAT, MAT::SizeAtCompileTime>::
			internal_resize(mat, new_rows, new_cols);
	}
};
// Specialization for column matrices:
template <int R>
struct MatOrVecResizer<R, 1>
{
	template <typename MAT>
	static inline void doit(MAT& mat, size_t new_rows, size_t)
	{
		::mrpt::math::detail::TAuxResizer<
			MAT, MAT::SizeAtCompileTime>::internal_resize(mat, new_rows);
	}
};
// Specialization for row matrices:
template <int C>
struct MatOrVecResizer<1, C>
{
	template <typename MAT>
	static inline void doit(MAT& mat, size_t, size_t new_cols)
	{
		::mrpt::math::detail::TAuxResizer<
			MAT, MAT::SizeAtCompileTime>::internal_resize(mat, new_cols);
	}
};
template <>
struct MatOrVecResizer<1, 1>
{
	template <typename MAT>
	static inline void doit(MAT& mat, size_t, size_t new_cols)
	{
		::mrpt::math::detail::TAuxResizer<
			MAT, MAT::SizeAtCompileTime>::internal_resize(mat, new_cols);
	}
};
}  // namespace detail

}  // namespace mrpt::math
