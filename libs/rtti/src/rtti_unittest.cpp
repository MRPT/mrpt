/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/rtti/CObject.h>

namespace MyNS
{
class MyDerived1 : public mrpt::rtti::CObject
{
   public:
	int m_value{0};

	MyDerived1() = default;
	MyDerived1(int v) : m_value(v) {}
	DEFINE_MRPT_OBJECT(MyDerived1, MyNS)
};

class MyDerived2 : public mrpt::rtti::CObject
{
   public:
	MyDerived2() = default;
	DEFINE_MRPT_OBJECT(MyDerived2, MyNS)
};

}  // namespace MyNS

// Register "MyNS::MyDerived1"
IMPLEMENTS_MRPT_OBJECT(MyDerived1, mrpt::rtti::CObject, MyNS)

// Register "MyNS::MyDerived2"
IMPLEMENTS_MRPT_OBJECT(MyDerived2, mrpt::rtti::CObject, MyNS)

void do_register()
{
	mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(MyDerived1, MyNS));
	mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(MyDerived2, MyNS));
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
	EXPECT_TRUE(
		std::string(cid_myd1->className) == std::string("MyNS::MyDerived1"));

	const auto cid_cobj = CLASS_ID(mrpt::rtti::CObject);
	EXPECT_TRUE(cid_myd1->getBaseClass() == cid_cobj);

	// RTTI IS_DERIVED(*)
	{
		auto p = mrpt::rtti::CObject::Ptr(new MyNS::MyDerived1);
		EXPECT_TRUE(IS_DERIVED(*p, MyNS::MyDerived1));
		EXPECT_TRUE(IS_DERIVED(*p, mrpt::rtti::CObject));
	}
}

TEST(rtti, Factory)
{
	do_register();
	{
		mrpt::rtti::CObject::Ptr p =
			mrpt::rtti::classFactory("MyNS::MyDerived1");
		EXPECT_TRUE(p);
	}
	{
		auto p = mrpt::rtti::classFactory("MyNS::MyDerived2");
		EXPECT_TRUE(p);
	}
}

TEST(rtti, CreateSmartPointerTypes)
{
	using T = MyNS::MyDerived1;
	{
		auto p = T::Create();
		EXPECT_TRUE(p);
		EXPECT_EQ(p->m_value, 0);
	}
	{
		auto p = T::Create(123);
		EXPECT_TRUE(p);
		EXPECT_EQ(p->m_value, 123);
	}
	{
		auto p = T::CreateUnique();
		EXPECT_TRUE(p);
		EXPECT_EQ(p->m_value, 0);
	}
	{
		auto p = T::CreateUnique(123);
		EXPECT_TRUE(p);
		EXPECT_EQ(p->m_value, 123);
	}
	{
		mrpt::aligned_allocator_cpp11<T> alloc;
		auto p = T::CreateAlloc(alloc, 123);
		EXPECT_TRUE(p);
		EXPECT_EQ(p->m_value, 123);
	}
}
