/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/** \example rtti_example1/test.cpp */

//! [example-define-class]
#include <mrpt/rtti/CObject.h>
#include <iostream>
#include <memory>

namespace MyNS
{
class Foo : public mrpt::rtti::CObject
{
   public:
	Foo() {}
	DEFINE_MRPT_OBJECT(Foo)

	void printName() { std::cout << "printName: Foo" << std::endl; }
};

class BarBase : public mrpt::rtti::CObject
{
   public:
	BarBase() {}
	DEFINE_VIRTUAL_MRPT_OBJECT(BarBase)

	virtual void printName() { std::cout << "printName: BarBase" << std::endl; }
};

class Bar : public BarBase
{
   public:
	Bar() {}
	DEFINE_MRPT_OBJECT(Bar)

	void printName() override { std::cout << "class: Bar" << std::endl; }
	void specificBarMethod()
	{
		std::cout << "specificBarMethod: reached." << std::endl;
	}
};
}  // namespace MyNS

IMPLEMENTS_MRPT_OBJECT(Foo, mrpt::rtti::CObject, MyNS)
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(BarBase, mrpt::rtti::CObject, MyNS)
IMPLEMENTS_MRPT_OBJECT(Bar, MyNS::BarBase, MyNS)

//! [example-define-class]

//! [example-define-class-test]
void Test_UserTypes()
{
	using namespace MyNS;
	const auto id_foo = CLASS_ID(Foo);
	std::cout << "RTTI Foo (static): " << id_foo->className << std::endl;

	// Pointers:
	Bar::Ptr pBar = std::make_shared<Bar>();
	BarBase::Ptr pBase = mrpt::ptr_cast<BarBase>::from(pBar);
	mrpt::rtti::CObject::Ptr pObj =
		mrpt::ptr_cast<mrpt::rtti::CObject>::from(pBar);

	pBar->printName();
	pBase->printName();
	std::cout << "Is Foo?     => " << (IS_DERIVED(pObj, Foo) ? "Yes" : "No")
			  << std::endl;
	std::cout << "Is BarBase? => " << (IS_DERIVED(pObj, BarBase) ? "Yes" : "No")
			  << std::endl;
	std::cout << "Is Bar?     => " << (IS_DERIVED(pObj, Bar) ? "Yes" : "No")
			  << std::endl;
	if (IS_CLASS(pObj, Bar))
	{
		auto pBar = mrpt::ptr_cast<Bar>::from(pObj);
		pBar->specificBarMethod();
	}
}

//! [example-define-class-test]

//! [example-factory]
void do_register()
{
	// Register with explicit namespace:
	mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(Foo, MyNS));
	{
		// Register without explicit namespace:
		using namespace MyNS;
		mrpt::rtti::registerClass(CLASS_ID(BarBase));
		mrpt::rtti::registerClass(CLASS_ID(Bar));
		mrpt::rtti::registerClassCustomName("MyNS::Bar", CLASS_ID(Bar));
	}
}

void Test_UserTypesFactory()
{
	do_register();

	// Test register:
	{
		const auto& allClasses = mrpt::rtti::getAllRegisteredClasses();
		for (const auto& cl : allClasses)
		{
			std::cout << "Known class: " << cl->className << ", children of "
					  << (cl->getBaseClass ? cl->getBaseClass()->className
										   : "(none)")
					  << std::endl;
		}
	}

	// Test factory:
	{
		mrpt::rtti::CObject::Ptr pObj =
			mrpt::rtti::classFactoryPtr("MyNS::Bar");
		if (IS_CLASS(pObj, MyNS::Bar))
		{
			auto pBar = mrpt::ptr_cast<MyNS::Bar>::from(pObj);
			pBar->specificBarMethod();
		}
	}
}

//! [example-factory]

int main(int argc, char** argv)
{
	try
	{
		Test_UserTypes();
		Test_UserTypesFactory();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}
