/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/TTypeName_stl.h>
#include <gtest/gtest.h>
#include <iostream>

struct MyFooClass {};
DECLARE_CUSTOM_TTYPENAME(MyFooClass);

TEST(TTypeName, types2str)
{
	using namespace mrpt::typemeta;
	using namespace std;

#define TST_FOR_TYPE(__TSTTYPE) \
	EXPECT_STREQ(#__TSTTYPE, TTypeName<__TSTTYPE>::get().c_str())

	// Basic types:
	TST_FOR_TYPE(int32_t);
	TST_FOR_TYPE(uint8_t);
	TST_FOR_TYPE(float);
	TST_FOR_TYPE(double);

	// STL:
	TST_FOR_TYPE(std::string);
	TST_FOR_TYPE(std::vector<double>);
	TST_FOR_TYPE(std::vector<int32_t>);
	TST_FOR_TYPE(std::set<double>);
	TST_FOR_TYPE(std::set<std::vector<double>>);

	// templates with a "," in its name break all our and gtest macros:
#define TST_FOR_TYPE2(__TSTTYPE,__TSTTYPE2ndpart) \
	if (std::string(#__TSTTYPE","#__TSTTYPE2ndpart)!=TTypeName<__TSTTYPE,__TSTTYPE2ndpart>::get()) \
		GTEST_FAIL() << "Failed: " << #__TSTTYPE","#__TSTTYPE2ndpart;

	TST_FOR_TYPE2(std::pair<int32_t,int32_t>);
	TST_FOR_TYPE2(std::map<double, std::set<int32_t>>);
	TST_FOR_TYPE2(std::array<double, 3>);

	// User-defined types:
	TST_FOR_TYPE(MyFooClass);
	TST_FOR_TYPE(std::vector<MyFooClass>);
}
