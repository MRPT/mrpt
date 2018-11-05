/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/** \example typemeta_TTypeName/test.cpp */

//! [example typename]
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/TTypeName_stl.h>
#include <iostream>
#include <memory>  // shared_ptr

// Declare custom user types:
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

void Test_TypeName()
{
	using namespace std;
	using namespace mrpt::typemeta;

	// Evaluation of type names as constexpr strings:
	constexpr auto s1 = TTypeName<int32_t>::get();
	cout << s1 << endl;

	cout << TTypeName<set<vector<double>>>::get() << endl;

	// Evaluation of user-defined types:
	cout << TTypeName<MyFooClass>::get() << endl;
	cout << TTypeName<MyFooClass::Ptr>::get() << endl;
	cout << TTypeName<MyNS::MyBarClass>::get() << endl;
	cout << TTypeName<MyNS::MyBarClass2>::get() << endl;

	// STL typenames as strings:
	cout << TTypeName<double>::get() << endl;
	cout << TTypeName<vector<double>>::get() << endl;
	cout << TTypeName<array<int32_t, 5>>::get() << endl;
	cout << TTypeName<set<double>>::get() << endl;
	cout << TTypeName<pair<int32_t, pair<int32_t, int32_t>>>::get() << endl;
	cout << TTypeName<map<double, set<int32_t>>>::get() << endl;
	cout << TTypeName<set<
				multimap<double, pair<MyFooClass, MyNS::MyBarClass2>>>>::get()
		 << endl;
}
//! [example typename]

int main(int argc, char** argv)
{
	try
	{
		Test_TypeName();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}
