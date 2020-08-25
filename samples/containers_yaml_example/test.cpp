/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>

/** \example containers_yaml_example/test.cpp */

//! [example-yaml]
#include <mrpt/containers/yaml.h>
#include <mrpt/core/demangle.h>  // demangle() utility

#include <iostream>

void YamlTest_1()
{
	std::cout << "==== YamlTest_1 ====\n\n";

	mrpt::containers::yaml p;
	p["K"] = 2.0;
	p["N"] = 10;
	p["name"] = "Foo";
	p["enabled"] = true;
	p["books"] = mrpt::containers::yaml::Sequence();
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
				  << "` type: " << mrpt::demangle(kv.second.typeName()) << "\n";
	}

	// Iterate a dictionary bis:
	mrpt::containers::yaml p2;
	p2["a"] = 1.0;
	p2["b"] = 10;
	p2["c"] = -1;

	for (const auto& kv : p2.asMap())
	{
		// This will raise an exception if stored type cannot be converted to
		// double:
		std::cout << "key: `" << kv.first << "` val: " << kv.second.as<double>()
				  << "\n";
	}

	// Iterate sequence:
	for (const auto& item : p["books"].asSequence())
	{
		std::cout << "sequence item type: " << mrpt::demangle(item.typeName())
				  << "\n";
		// Access value: kv.second.as<std::string>(), etc.
	}

	// Print:
	std::cout << "\n\nPrint as YAML:\n";
	p.printAsYAML();
}

void YamlTest_2()
{
	std::cout << "\n\n==== YamlTest_2 ====\n\n";

	// You can use {} to initialize mappings (dictionaries):
	const mrpt::containers::yaml p = mrpt::containers::yaml::Map(
		{{"K", 2.0}, {"book", std::string("silmarillion")}});

	ASSERT_(!p.isSequence());
	ASSERT_(p.isMap());
	ASSERT_(!p.isNullNode());
	ASSERT_(p.has("K"));
	ASSERT_(p["K"].isScalar());

	// or a sequence (each element may have a different type)
	const auto p2 = mrpt::containers::yaml::Sequence({1.0, 2.0, "foo", true});

	ASSERT_(p2.isSequence());

	p.printAsYAML(std::cout);
}

const auto sData = std::string(R"xxx(
myMap:
  K: 10.0
  P: -5.0
  Q: ~
  nestedMap:
    a: 1  # comment for a
    b: 2
    c: 3
)xxx");

void YamlTest_3()
{
	std::cout << "\n\n==== YamlTest_3 ====\n\n";

	// Parse a YAML or JSON text:
	mrpt::containers::yaml p = mrpt::containers::yaml::FromText(sData);

	// Get comments:
	std::cout << "Comment: '" << p["myMap"]["nestedMap"]["a"].comment()
			  << "'\n";

	// Manipulate comments:
	p["myMap"]["nestedMap"]["b"].comment("This is a comment for b");

	// Add values and comments at once:
	p["myMap"]["foo"] = mrpt::containers::vcp(1.0, "Another constant");

	p.printAsYAML(std::cout);
}
//! [example-yaml]

int main()
{
	try
	{
		YamlTest_1();
		YamlTest_2();
		YamlTest_3();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT exception caught: " << mrpt::exception_to_str(e)
				  << std::endl;
		return -1;
	}
}
