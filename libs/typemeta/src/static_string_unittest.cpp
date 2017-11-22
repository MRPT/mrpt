/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/static_string.h>
#include <gtest/gtest.h>

using namespace mrpt::typemeta;

TEST(StaticString, ctor)
{
	constexpr string_literal<3> a = "foo";
	(void)(a);

	constexpr auto b = literal("foo");
	(void)(b);
}

TEST(StaticString, concat_literals)
{
	constexpr auto a = literal("foo");
	constexpr auto b = literal("bar");
	constexpr auto ab = a + b;

	static_assert(ab.size() == 6, "***");
	static_assert(ab[0] == 'f', "***");
	static_assert(ab[5] == 'r', "***");
}

TEST(StaticString, concat_multi)
{
	constexpr auto a = literal("foo");
	constexpr auto b = literal("bar");
	constexpr auto ab = a + b;
	constexpr auto ba = b + a;

	// test sstring + literal:
	constexpr auto abc = ab+literal("more");
	static_assert(abc.size() == (6+4), "***");

	// test sstring + sstring:
	constexpr auto abba = ab+ba;
	static_assert(abba.size() == 2*6, "***");

	const char* s = abba.c_str();
	(void)(s);
}
