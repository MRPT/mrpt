/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/typemeta/num_to_string.h>
#include <mrpt/typemeta/static_string.h>

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
	// In GCC7 these ones can be "constexpr", but that fails in MSVC 2017 (!)
	auto ab = a + b;

	static_assert(ab.size() == 6, "***");
	EXPECT_EQ(ab[0], 'f');
	EXPECT_EQ(ab[5], 'r');
}

TEST(StaticString, concat_multi)
{
	constexpr auto a = literal("foo");
	constexpr auto b = literal("bar");
	auto ab = a + b;
	auto ba = b + a;

	// test sstring + literal:
	auto abc = ab + literal("more");
	static_assert(abc.size() == (6 + 4), "***");

	// test sstring + sstring:
	auto abba = ab + ba;
	static_assert(abba.size() == 2 * 6, "***");

	// Test c_str():
	const char* s = abba.c_str();
	const char* s2 = static_cast<const char*>(abba);
	EXPECT_EQ(s, s2);

	// Test cast to "const char*" () operator:
	const char* s3 = static_cast<const char*>(a);
	EXPECT_EQ(a.c_str(), s3);
}

TEST(num_to_string, ctor)
{
	constexpr auto a = num_to_string<13>::value;
	(void)(a);
}
