/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/types_math.h>  // Eigen
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/num_to_string.h>
#include <mrpt/math/point_poses2vectors.h>  // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()

namespace mrpt
{
namespace math
{
/** CArrayNumeric is an array for numeric types supporting several mathematical
 * operations (actually, just a wrapper on Eigen::Matrix<T,N,1>)
 * \sa CArrayFloat, CArrayDouble, CArray
 */
template <typename T, std::size_t N>
class CArrayNumeric : public Eigen::Matrix<T, N, 1>
{
   public:
	using value_type = T;
	using Base = Eigen::Matrix<T, N, 1>;

	/** Default constructor */
	CArrayNumeric() = default;
	/** Constructor from initial values ptr[0]-ptr[N-1] */
	CArrayNumeric(const T* ptr) : Eigen::Matrix<T, N, 1>(ptr) {}
	/** Initialization from a vector-like source, that is, anything implementing
	 * operator[]. */
	template <class Derived>
	explicit CArrayNumeric(const Eigen::MatrixBase<Derived>& obj)
		: Eigen::Matrix<T, N, 1>(obj)
	{
	}

	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CArrayNumeric)

	template <typename OtherDerived>
	inline CArrayNumeric<T, N>& operator=(
		const Eigen::MatrixBase<OtherDerived>& other)
	{
		Base::operator=(other);
		return *this;
	}
};

// --------------  Partial specializations of CArrayNumeric -----------

/** A partial specialization of CArrayNumeric for float numbers.
 * \sa CArrayNumeric, CArray */
template <std::size_t N>
using CArrayFloat = CArrayNumeric<float, N>;

/** A partial specialization of CArrayNumeric for double numbers.
 * \sa CArrayNumeric, CArray */
template <std::size_t N>
using CArrayDouble = CArrayNumeric<double, N>;

/** A partial specialization of CArrayNumeric for int numbers.
 * \sa CArrayNumeric, CArray */
template <std::size_t N>
using CArrayInt = CArrayNumeric<int, N>;

/** A partial specialization of CArrayNumeric for unsigned int numbers.
 * \sa CArrayNumeric, CArray */
template <std::size_t N>
using CArrayUInt = CArrayNumeric<unsigned int, N>;
}  // namespace math

namespace typemeta
{
// Extensions to mrpt::typemeta::TTypeName for matrices:
template <typename T, size_t N>
struct TTypeName<mrpt::math::CArrayNumeric<T, N>>
{
	constexpr static auto get()
	{
		return literal("CArrayNumeric<") + TTypeName<T>::get() + literal(",") +
			   literal(num_to_string<N>::value) + literal(">");
	}
};
template <size_t N>
struct TTypeName<mrpt::math::CArrayDouble<N>>
{
	constexpr static auto get()
	{
		return literal("CArrayDouble<") + literal(num_to_string<N>::value) +
			   literal(">");
	}
};
template <size_t N>
struct TTypeName<mrpt::math::CArrayFloat<N>>
{
	constexpr static auto get()
	{
		return literal("CArrayFloat<") + literal(num_to_string<N>::value) +
			   literal(">");
	}
};
}  // namespace typemeta
}  // namespace mrpt
