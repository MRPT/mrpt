/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/TTypeName_stl.h>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>  // shared_ptr

struct MyFooClass
{
	using Ptr = std::shared_ptr<MyFooClass>;
};
namespace MyNS
{
struct MyBarClass
{
};
struct MyBarClass2
{
	DECLARE_TTYPENAME_CLASSNAME(MyNS::MyBarClass2)
};
}  // namespace MyNS

DECLARE_CUSTOM_TTYPENAME(MyFooClass);
DECLARE_CUSTOM_TTYPENAME(MyNS::MyBarClass);

#define TST_FOR_TYPE(__TSTTYPE) \
	EXPECT_STREQ(#__TSTTYPE, TTypeName<__TSTTYPE>::get().c_str())

// templates with a "," in its name break all our and gtest macros:
#define TST_FOR_TYPE2(__TSTTYPE, __TSTTYPE2ndpart)                            \
	if (std::string(#__TSTTYPE "," #__TSTTYPE2ndpart) !=                      \
		TTypeName<__TSTTYPE, __TSTTYPE2ndpart>::get().c_str())                \
		GTEST_FAIL() << "Failed: " << #__TSTTYPE "," #__TSTTYPE2ndpart        \
					 << "\n Computed type is: "                               \
					 << TTypeName<__TSTTYPE, __TSTTYPE2ndpart>::get().c_str() \
					 << endl;

TEST(TTypeName, types2str)
{
	using namespace mrpt::typemeta;
	using namespace std;

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

	TST_FOR_TYPE2(std::pair<int32_t, int32_t>);
	TST_FOR_TYPE2(std::map<double, std::set<int32_t>>);
	TST_FOR_TYPE2(std::array<double, 3>);

	// User-defined types:
	TST_FOR_TYPE(MyFooClass);
	TST_FOR_TYPE(MyNS::MyBarClass);
	TST_FOR_TYPE(MyNS::MyBarClass2);
	TST_FOR_TYPE(std::vector<MyFooClass>);
	TST_FOR_TYPE(std::set<MyNS::MyBarClass>);
	TST_FOR_TYPE(std::set<MyNS::MyBarClass2>);
	TST_FOR_TYPE2(std::vector<std::array<MyNS::MyBarClass, 10>>);
}

TEST(TTypeName, types2str_shared_ptr)
{
	using namespace mrpt::typemeta;

	TST_FOR_TYPE(std::shared_ptr<MyFooClass>);
	TST_FOR_TYPE(std::vector<std::shared_ptr<MyFooClass>>);
	EXPECT_STREQ(
		"std::shared_ptr<MyFooClass>",
		TTypeName<MyFooClass::Ptr>::get().c_str());
}

TEST(TTypeName, types2stdstring)
{
	using namespace mrpt::typemeta;
	using namespace std;

	auto st1 = TTypeName<uint8_t>::get();
	const std::string s1(st1.c_str());
	const std::string s2("uint8_t");
	EXPECT_STREQ(s1.c_str(), s2.c_str());
}
