/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/Parameters.h>

TEST(Parameters, emptyCtor)
{
	{
		mrpt::containers::Parameters p;
		EXPECT_TRUE(p.empty());
	}

	{
		const auto p = mrpt::containers::Parameters({});
		EXPECT_TRUE(p.empty());
	}
}

TEST(Parameters, assignments)
{
	mrpt::containers::Parameters p;
	p["K"] = 2.0;
	p["N"].as<uint64_t>() = 10;
	p["name"] = "Pepico";

	EXPECT_FALSE(p.empty());
	EXPECT_TRUE(p.has("K"));

	EXPECT_EQ(p.typeOf("K"), "double");
	EXPECT_EQ(p["K"], 2.0);

	EXPECT_EQ(p.typeOf("N"), "uint64_t");
	EXPECT_EQ(p["N"].as<uint64_t>(), 10U);

	EXPECT_EQ(p.typeOf("name"), "std::string");
	EXPECT_EQ(p["name"].as<std::string>(), "Pepico");

	{
		mrpt::containers::Parameters p2;
		EXPECT_TRUE(p2.empty());

		p2 = p;
		EXPECT_FALSE(p2.empty());
		EXPECT_TRUE(p2.has("K"));
	}
}

TEST(Parameters, initializer)
{
	const auto p =
		mrpt::containers::Parameters({{"K", 2.0}, {"book", "silmarillion"}});

	EXPECT_FALSE(p.empty());
	EXPECT_TRUE(p.has("K"));

	EXPECT_EQ(p.typeOf("K"), "double");
	EXPECT_EQ(p["K"], 2.0);

	EXPECT_EQ(p.typeOf("book"), "std::string");
	EXPECT_EQ(p["book"].as<std::string>(), "silmarillion");

	// non existing in const object:
	EXPECT_THROW(p["foo"], std::exception);
	// Access wrong type in const object:
	EXPECT_THROW(p["K"].as<std::string>(), std::exception);
}

TEST(Parameters, nested)
{
	auto p = mrpt::containers::Parameters({{"K", 2.0}});
	p["PID"] = mrpt::containers::Parameters({{"Kp", 10.0}, {"Ti", 10.0}});

	EXPECT_FALSE(p.empty());
	EXPECT_FALSE(p["PID"].empty());

	EXPECT_EQ(p["PID"].typeOf("Ti"), "double");
	EXPECT_EQ(p["PID"]["Ti"], 10.0);

	// empty() not valid for values:
	EXPECT_THROW(p["PID"]["Ti"].empty(), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].clear(), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].has("xxx"), std::exception);
	EXPECT_THROW(p["PID"]["Ti"].typeOf("xxx"), std::exception);

	// clear and recheck;
	p["PID"].clear();
	EXPECT_TRUE(p["PID"].empty());
}

TEST(Parameters, nested2)
{
	mrpt::containers::Parameters p;
	p["N"] = 10;
	auto& pid = p["PID"] = mrpt::containers::Parameters();
	pid["Kp"] = 0.5;
	p["PID"]["Ti"] = 2.0;
	p["PID"]["N"].as<uint64_t>() = 1000;
	p["PID"]["name"] = "foo";

	EXPECT_EQ(p["PID"]["Kp"], 0.5);
	EXPECT_EQ(p["PID"]["Ti"], 2.0);
	EXPECT_EQ(p["PID"]["N"].as<uint64_t>(), 1000U);
	EXPECT_EQ(p["PID"]["name"].as<std::string>(), std::string("foo"));

#if 0
	std::cout << p["PID"]["Kp"].as<double>() << "\n";
	std::cout << p["PID"]["Ti"].as<double>() << "\n";
	std::cout << p["PID"]["N"].as<uint64_t>() << "\n";
	std::cout << p["PID"]["name"].as<std::string>() << "\n";
#endif
}