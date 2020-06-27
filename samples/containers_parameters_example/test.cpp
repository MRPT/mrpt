/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>

/** \example containers_parameters_example/test.cpp */

//! [example-parameters]
#include <mrpt/containers/Parameters.h>

#include <iostream>

void ParameterTest_1()
{
	mrpt::containers::Parameters p;
	p["K"] = 2.0;
	p["N"].as<uint64_t>() = 10;
	p["name"] = "Foo";
	p["enabled"] = true;
	p["books"] = mrpt::containers::Parameters::Sequence();
	p["books"].push_back("The Hobbit");
	p["books"].push_back(10.0);

	std::cout << "K=" << p["K"] << " N=" << p["N"] << "\n";
	std::cout << "name=" << p["name"]
			  << " bar=" << p.getOrDefault<std::string>("bar", "default")
			  << "\n";

	// Iterate a dictionary:
	for (const auto& kv : p.asMap())
	{
		std::cout << "key: `" << kv.first
				  << "` type: " << kv.second.type().name() << "\n";
		// Access value: kv.second.as<double>(), etc.
	}

	// Iterate sequence:
	for (const auto& item : p["books"].asSequence())
	{
		std::cout << "sequence item type: " << item.type().name() << "\n";
		// Access value: kv.second.as<std::string>(), etc.
	}
}

void ParameterTest_2()
{
	// You can use {} to initialize mappings (dictionaries):
	const auto p = mrpt::containers::Parameters::Map(
		{{"K", 2.0}, {"book", std::string("silmarillion")}});

	ASSERT_(!p.isSequence());
	ASSERT_(p.isMap());
	ASSERT_(!p.empty());
	ASSERT_(p.has("K"));
	ASSERT_(p["K"].isScalar());

	// or a sequence (each element may have a different type)
	const auto p2 =
		mrpt::containers::Parameters::Sequence({1.0, 2.0, "foo", true});

	ASSERT_(p2.isSequence());
}
//! [example-parameters]

int main()
{
	try
	{
		ParameterTest_1();
		ParameterTest_2();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT exception caught: " << mrpt::exception_to_str(e)
				  << std::endl;
		return -1;
	}
}
