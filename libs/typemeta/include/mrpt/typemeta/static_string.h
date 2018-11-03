/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

/** \file static string for constexpr. Based on:
 * https://akrzemi1.wordpress.com/2017/06/28/compile-time-string-concatenation/
 * (Boost License)
 */

#include <mrpt/typemeta/xassert.h>

namespace mrpt
{
namespace typemeta
{
template <int N>
class string_literal
{
	const char (&_lit)[N + 1];

   public:
	/** Ctor from C string literal, with trailing zero. */
	constexpr string_literal(const char (&lit)[N + 1])
		: _lit((MRPT_X_ASSERT(lit[N] == '\0'), lit))
	{
	}
	constexpr std::size_t size() const { return N; }
	constexpr char operator[](int i) const
	{
		return MRPT_X_ASSERT(i >= 0 && i < N), _lit[i];
	}
	constexpr const char* c_str() const { return _lit; }
	constexpr operator const char*() const { return c_str(); }
	operator std::string() const { return _lit; }
};

template <int N_PLUS_1>
constexpr auto literal(const char (&lit)[N_PLUS_1])
	-> string_literal<N_PLUS_1 - 1>
{
	return string_literal<N_PLUS_1 - 1>(lit);
}

#define REQUIRES(...) typename std::enable_if<(__VA_ARGS__), bool>::type = true

namespace internal
{
// the type used to receive the pack
template <int... I>
struct sequence
{
};

// auxiliary meta-function for making (N+1)-sized sequence
// from an N-sized sequence
template <typename T>
struct append;

template <int... I>
struct append<sequence<I...>>
{
	using type = sequence<I..., sizeof...(I)>;
};

// recursive implementation of make_sequence

template <int I>
struct make_sequence_;

template <int I>
using make_sequence = typename make_sequence_<I>::type;

template <>
struct make_sequence_<0>  // recursion end
{
	using type = sequence<>;
};

template <int I>
struct make_sequence_ : append<make_sequence<I - 1>>
{
	static_assert(I >= 0, "negative size");
};
}  // namespace internal

template <int N>
class array_string
{
	char _array[N + 1];

	template <typename S1, typename S2, int... PACK1, int... PACK2>
	constexpr array_string(
		const S1& s1, const S2& s2, internal::sequence<PACK1...>,
		internal::sequence<PACK2...>)
		: _array{s1[PACK1]..., s2[PACK2]..., '\0'}
	{
	}

   public:
	/** ctor: literal + literal */
	template <int N1, REQUIRES(N1 <= N)>
	constexpr array_string(
		const string_literal<N1>& s1, const string_literal<N - N1>& s2)
		: array_string{s1, s2, internal::make_sequence<N1>{},
					   internal::make_sequence<N - N1>{}}
	{
	}

	/** ctor: string + literal */
	template <int N1, REQUIRES(N1 <= N)>
	constexpr array_string(
		const array_string<N1>& s1, const string_literal<N - N1>& s2)
		: array_string{s1, s2, internal::make_sequence<N1>{},
					   internal::make_sequence<N - N1>{}}
	{
	}

	/** ctor: string + string */
	template <int N1, REQUIRES(N1 <= N)>
	constexpr array_string(
		const array_string<N1>& s1, const array_string<N - N1>& s2)
		: array_string{s1, s2, internal::make_sequence<N1>{},
					   internal::make_sequence<N - N1>{}}
	{
	}

	constexpr std::size_t size() const { return N; }
	constexpr char operator[](int i) const
	{
		return MRPT_X_ASSERT(i >= 0 && i < N), _array[i];
	}
	constexpr const char* c_str() const { return _array; }
	constexpr operator const char*() const { return c_str(); }
	operator std::string() const { return c_str(); }
};

template <int N1, int N2>
constexpr auto operator+(
	const string_literal<N1>& s1, const string_literal<N2>& s2)
	-> array_string<N1 + N2>
{
	return array_string<N1 + N2>(s1, s2);
}

template <int N1, int N2>
constexpr auto operator+(
	const array_string<N1>& s1, const string_literal<N2>& s2)
	-> array_string<N1 + N2>
{
	return array_string<N1 + N2>(s1, s2);
}

template <int N1, int N2>
constexpr auto operator+(const array_string<N1>& s1, const array_string<N2>& s2)
	-> array_string<N1 + N2>
{
	return array_string<N1 + N2>(s1, s2);
}

}  // namespace typemeta
}  // namespace mrpt
