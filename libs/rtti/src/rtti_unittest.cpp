/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/rtti/CObject.h>
#include <gtest/gtest.h>

namespace MyNS
{
class MyDerived1 : public mrpt::rtti::CObject
{
   public:
	MyDerived1() = default;
	DEFINE_MRPT_OBJECT(MyDerived1);
};
}  // namespace MyNS

IMPLEMENTS_MRPT_OBJECT(MyDerived1, mrpt::rtti::CObject, MyNS)

void do_register()
{
	mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(MyDerived1, MyNS));
}

TEST(rtti, CObject_CLASSID)
{
	EXPECT_TRUE(
		CLASS_ID(mrpt::rtti::CObject) ==
		&mrpt::rtti::CObject::GetRuntimeClassIdStatic());
}

TEST(rtti, MyDerived1_CLASSID)
{
	using namespace std;
	const auto cid_myd1 = CLASS_ID(MyNS::MyDerived1);
	EXPECT_TRUE(std::string(cid_myd1->className) == std::string("MyDerived1"));

	const auto cid_cobj = CLASS_ID(mrpt::rtti::CObject);
	EXPECT_TRUE(cid_myd1->getBaseClass() == cid_cobj);

	// RTTI IS_DERIVED()
	{
		auto p = mrpt::rtti::CObject::Ptr(new MyNS::MyDerived1);
		EXPECT_TRUE(IS_DERIVED(p, MyNS::MyDerived1));
		EXPECT_TRUE(IS_DERIVED(p, mrpt::rtti::CObject));
	}
}

TEST(rtti, Factory)
{
	do_register();
	mrpt::rtti::CObject::Ptr p = mrpt::rtti::classFactoryPtr("MyDerived1");
	EXPECT_TRUE(p);
}
