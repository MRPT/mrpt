/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/containers/Parameters.h>

#include <algorithm>  // count()

TEST(Parameters, emptyCtor)
{
	{
		mrpt::containers::Parameters p;
		EXPECT_TRUE(p.empty());
	}
}

TEST(Parameters, assignments)
{
	mrpt::containers::Parameters p;
	p["K"] = 2.0;
	p["N"].asRef<uint64_t>() = 10;
	p["name"] = "Pepico";
	p["one"] = "1.0";
	p["enabled"] = true;
	p["visible"] = false;
	p["hidden"] = "true";

	EXPECT_FALSE(p.empty());
	EXPECT_TRUE(p.has("K"));

	EXPECT_FALSE(p.isScalar());
	EXPECT_TRUE(p["K"].isScalar());
	EXPECT_TRUE(p["N"].isScalar());

	EXPECT_TRUE(p["enabled"]);
	EXPECT_FALSE(p["visible"]);
	EXPECT_TRUE(p["hidden"]);

	EXPECT_EQ(p.typeOfChild("K"), typeid(double));
	EXPECT_EQ(p["K"].as<double>(), 2.0);

	EXPECT_EQ(p.typeOfChild("N"), typeid(uint64_t));
	EXPECT_EQ(p["N"].as<uint64_t>(), 10U);

	EXPECT_EQ(p.typeOfChild("name"), typeid(std::string));
	EXPECT_EQ(p["name"].as<std::string>(), "Pepico");

	EXPECT_EQ(p.typeOfChild("enabled"), typeid(bool));

	{
		mrpt::containers::Parameters p2;
		EXPECT_TRUE(p2.empty());

		p2 = p;
		EXPECT_FALSE(p2.empty());
		EXPECT_TRUE(p2.has("K"));
	}

	// type conversions:
	EXPECT_EQ(p["K"].as<int>(), 2);
	EXPECT_EQ(p["N"].as<std::string>(), "10");
	EXPECT_EQ(p["N"].as<double>(), 10.0);
	EXPECT_EQ(p["one"].as<std::string>(), "1.0");
	EXPECT_EQ(p["one"].as<unsigned int>(), 1U);
	EXPECT_EQ(p["one"].as<int>(), 1);
	EXPECT_EQ(p["enabled"].as<std::string>(), "true");
	EXPECT_EQ(p["visible"].as<std::string>(), "false");
}

TEST(Parameters, initializers)
{
	{
		auto p = mrpt::containers::Parameters({"K", 1.0});
		EXPECT_TRUE(p.isSequence());
	}
	{
		auto p = mrpt::containers::Parameters({{"K", 1.0}});
		EXPECT_TRUE(p.isMap());
	}

	{
		auto p = mrpt::containers::Parameters({"K", 1.0, 10.0});
		EXPECT_TRUE(p.isSequence());
	}
	{
		auto p = mrpt::containers::Parameters(
			{{"K", 1.0}, {"T", 10.0}, {"Name", "bar"}});
		EXPECT_TRUE(p.isMap());
	}
}

TEST(Parameters, initializerMap)
{
	const auto p = mrpt::containers::Parameters(
		{{"K", 2.0}, {"book", std::string("silmarillion")}});

	EXPECT_FALSE(p.isSequence());
	EXPECT_TRUE(p.isMap());
	EXPECT_FALSE(p.empty());
	EXPECT_TRUE(p.has("K"));

	EXPECT_EQ(p.typeOfChild("K"), typeid(double));
	EXPECT_EQ(p["K"].as<double>(), 2.0);

	EXPECT_EQ(p.getOrDefault("K", 1.0), 2.0);
	EXPECT_EQ(p.getOrDefault("Q", 1.0), 1.0);
	EXPECT_EQ(p.getOrDefault<uint32_t>("K", 1), 2U);

	EXPECT_EQ(p.typeOfChild("book"), typeid(std::string));
	EXPECT_EQ(p["book"].as<std::string>(), "silmarillion");

	// non existing in const object:
	EXPECT_THROW(p["foo"], std::exception);
	// Access wrong type in const object:
	EXPECT_THROW(p["K"].asRef<std::string>(), std::exception);
}

TEST(Parameters, initializerSequence)
{
	const auto seq1 = mrpt::containers::Parameters({1.0, 2.0, 3.0});

	EXPECT_FALSE(seq1.empty());
	EXPECT_TRUE(seq1.isSequence());
	EXPECT_FALSE(seq1.isMap());
	EXPECT_THROW(seq1.has("foo"), std::exception);
	EXPECT_THROW(seq1["K"], std::exception);

	EXPECT_TRUE(static_cast<double>(seq1(0)) == 1.0);
	EXPECT_EQ(seq1(0).as<double>(), 1.0);
	EXPECT_EQ(seq1(0).as<double>(), 1.0);
	EXPECT_EQ(seq1(1).as<double>(), 2.0);
	EXPECT_EQ(seq1(2).as<double>(), 3.0);

	EXPECT_THROW(seq1(3), std::out_of_range);
	EXPECT_THROW(seq1(-1), std::out_of_range);

	auto seq2 = mrpt::containers::Parameters({1.0, 2.0, 3.0});
	EXPECT_EQ(seq2(1).as<double>(), 2.0);

	seq2(1) = 42.0;
	EXPECT_EQ(seq2(1).as<double>(), 42.0);

	seq2(1) = "foo";
	EXPECT_EQ(seq2(1).as<std::string>(), std::string("foo"));

	seq2(1) = mrpt::containers::Parameters({{"K", 1.0}});
	EXPECT_EQ(seq2(1)["K"].as<double>(), 1.0);

	seq2.push_back(std::string("foo2"));
	seq2.push_back(9.0);

	EXPECT_EQ(seq2(3).as<std::string>(), std::string("foo2"));
	EXPECT_EQ(seq2(4).as<double>(), 9.0);
}

TEST(Parameters, nested)
{
	auto p = mrpt::containers::Parameters({{"K", 2.0}});
	p["PID"] = mrpt::containers::Parameters({{"Kp", 10.0}, {"Ti", 10.0}});

	EXPECT_FALSE(p.empty());
	EXPECT_FALSE(p["PID"].empty());

	EXPECT_EQ(p["PID"].typeOfChild("Ti"), typeid(double));
	EXPECT_EQ(p["PID"]["Ti"].as<double>(), 10.0);

	EXPECT_FALSE(p.isScalar());
	EXPECT_FALSE(p["PID"].isScalar());
	EXPECT_TRUE(p["PID"]["Kp"].isScalar());

	// empty() not valid for values:
	EXPECT_THROW(p["PID"]["Ti"].empty(), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].clear(), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].has("xxx"), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].typeOfChild("xxx"), std::exception);

	// clear and recheck;
	p["PID"].clear();
	EXPECT_TRUE(p["PID"].empty());
}

TEST(Parameters, nested2)
{
	mrpt::containers::Parameters p;
	p["N"].asRef<uint64_t>() = 10;
	auto& pid = p["PID"] = mrpt::containers::Parameters();
	pid["Kp"] = 0.5;
	p["PID"]["Ti"] = 2.0;
	p["PID"]["N"].asRef<uint64_t>() = 1000;
	p["PID"]["name"] = "foo";

	EXPECT_EQ(p["PID"]["Kp"].as<double>(), 0.5);
	EXPECT_EQ(p["PID"]["Ti"].as<double>(), 2.0);
	EXPECT_EQ(p["PID"]["N"].as<uint64_t>(), 1000U);
	EXPECT_EQ(p["PID"]["name"].as<std::string>(), std::string("foo"));
}

const auto testMap = mrpt::containers::Parameters(
	{{"K", 2.0},
	 {"book", "silmarillion"},
	 {"mySequence",
	  mrpt::containers::Parameters::Sequence(
		  {1.0, 2.0,
		   mrpt::containers::Parameters::Map({{"P", 1.0}, {"Q", 2.0}})})},
	 {"myEmptyVal", mrpt::containers::Parameters()},
	 {"myDict",
	  mrpt::containers::Parameters({{"A", 1.0}, {"B", 2.0}, {"C", 3.0}})}});

TEST(Parameters, printYAML)
{
	// testMap.printAsYAML();
	std::stringstream ss;
	testMap.printAsYAML(ss);
	const auto s = ss.str();
	EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 13U);
}

TEST(Parameters, iterate)
{
	std::set<std::string> foundKeys;
	for (const auto& kv : testMap.asMap())
	{
		// std::cout << kv.first << ":" << kv.second.index() << "\n";
		foundKeys.insert(kv.first);
	}
	EXPECT_EQ(foundKeys.size(), 5U);

	foundKeys.clear();
	for (const auto& kv : testMap["myDict"].asMap())
	{
		// std::cout << kv.first << ":" << kv.second.index() << "\n";
		foundKeys.insert(kv.first);
	}
	EXPECT_EQ(foundKeys.size(), 3U);

	EXPECT_EQ(testMap["mySequence"].asSequence().size(), 3U);
}

TEST(Parameters, macros)
{
	mrpt::containers::Parameters p;
	p["K"] = 2.0;
	p["Ang"] = 90.0;
	p["N"].asRef<uint64_t>() = 10;
	p["name"] = "Pepico";
	p["PID"] = mrpt::containers::Parameters({{"Kp", 1.0}, {"Td", 0.8}});

	double K, Td, Foo = 9.0, Bar, Ang, Ang2 = M_PI;
	std::string name;
	MCP_LOAD_REQ(p, K);
	EXPECT_EQ(K, 2.0);

	MCP_LOAD_REQ(p, name);
	EXPECT_EQ(name, "Pepico");

	MCP_LOAD_REQ(p["PID"], Td);
	EXPECT_EQ(Td, 0.8);

	MCP_LOAD_REQ_DEG(p, Ang);
	EXPECT_NEAR(Ang, 0.5 * M_PI, 1e-6);

	MCP_LOAD_OPT_DEG(p, Ang2);
	EXPECT_NEAR(Ang2, M_PI, 1e-6);

	MCP_LOAD_OPT(p, Foo);
	EXPECT_EQ(Foo, 9.0);

	EXPECT_THROW(MCP_LOAD_REQ(p, Bar), std::exception);
}

// clang-format off
const auto sampleYamlBlock = std::string(R"xxx(
# blah blah
mySeq:
  - "first"
  - "second"
  - "third"
myMap:
  K: 10.0
  P: -5.0
)xxx");
// clang-format on

#if MRPT_HAS_YAMLCPP
TEST(Parameters, fromYAML)
{
	auto p = mrpt::containers::Parameters::FromYAMLText(sampleYamlBlock);
	EXPECT_EQ(p["mySeq"](0).as<std::string>(), "first");
	EXPECT_EQ(p["myMap"]["P"].as<double>(), -5.0);
}
#endif
