/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "common.h"
//
#include <mrpt/containers/yaml.h>
#include <mrpt/system/filesystem.h>

#ifdef RUN_YAMLCPP_COMPARISON
#include <yaml-cpp/yaml.h>
#endif

const char* fil = "/tmp/distribution.yaml";

static const auto smallYamlBlock = std::string(R"xxx(
# blah blah
mySeq:
  - "first"
  - "second"
  - "third"
  - ~
myMap:
  K: 10.0
  P: -5.0
  Q: ~
  nestedMap:
    a: 1  # comment for a
    b: 2
    c: 3
)xxx");

static bool prepareYamlTestFile()
{
	bool err = false;
	if (!mrpt::system::fileExists(fil))
	{
		// Download big yaml file:
		if (0 !=
			::system(mrpt::format(
						 "wget -O %s "
						 "https://github.com/ros/rosdistro/raw/master/indigo/"
						 "distribution.yaml",
						 fil)
						 .c_str()))
			err = true;
	}

	if (err || !mrpt::system::fileExists(fil))
	{
		std::cerr << "Skipping test due to missing downloaded test file.\n";
		return false;
	}
	return true;
}

double yaml_loadFromFile(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10; i++)
	{
		mrpt::containers::yaml doc;
		tl.enter("t");
		doc.loadFromFile(fil);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}
double yaml_FromFile(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10; i++)
	{
		tl.enter("t");
		auto doc = mrpt::containers::yaml::FromFile(fil);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}

double yaml_loadFromText(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10000; i++)
	{
		mrpt::containers::yaml doc;
		tl.enter("t");
		doc.loadFromText(smallYamlBlock);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}
double yaml_FromText(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10000; i++)
	{
		tl.enter("t");
		auto doc = mrpt::containers::yaml::FromText(smallYamlBlock);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}

double yaml_query(int, int)
{
	if (!prepareYamlTestFile()) return 0;

	const auto doc = mrpt::containers::yaml::FromFile(fil);
	std::string sTotal;

	mrpt::system::CTimeLogger tl;
	const unsigned int reps = 100000;
	for (unsigned int i = 0; i < reps; i++)
	{
		tl.enter("t");
		const std::string s1 =
			doc["repositories"]["imu_compass"]["release"]["url"];
		const std::string s2 = doc["release_platforms"]["ubuntu"](0);
		tl.leave("t");
		sTotal += s1;
		sTotal += s2;
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	ASSERT_EQUAL_(sTotal.size(), 62U * reps);
	return r;
}
double yaml_iterate(int, int)
{
	if (!prepareYamlTestFile()) return 0;

	const auto doc = mrpt::containers::yaml::FromFile(fil);
	size_t visits = 0, allEntries = 0;
	std::string sAllKeys;

	mrpt::system::CTimeLogger tl;
	const unsigned int reps = 100;
	for (unsigned int i = 0; i < reps; i++)
	{
		tl.enter("t");
		for (const auto& kv : doc["repositories"].asMap())
		{
			visits++;
			sAllKeys += kv.first.as<std::string>();
			allEntries += kv.second.size();
		}
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	ASSERT_EQUAL_(visits, 1231U * reps);
	ASSERT_EQUAL_(sAllKeys.size(), 16849U * reps);
	ASSERT_EQUAL_(allEntries, 4099U * reps);
	return r;
}

#ifdef RUN_YAMLCPP_COMPARISON
double yaml_yamlcpp_FromFile(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10; i++)
	{
		tl.enter("t");
		auto doc = YAML::LoadFile(fil);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}
double yaml_yamlcpp_Load(int, int)
{
	if (!prepareYamlTestFile()) return 0;
	mrpt::system::CTimeLogger tl;
	for (unsigned int i = 0; i < 10000; i++)
	{
		tl.enter("t");
		auto doc = YAML::Load(smallYamlBlock);
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	return r;
}
double yaml_yamlcpp_query(int, int)
{
	if (!prepareYamlTestFile()) return 0;

	const auto doc = YAML::LoadFile(fil);
	std::string sTotal;

	mrpt::system::CTimeLogger tl;
	const unsigned int reps = 100000;
	for (unsigned int i = 0; i < reps; i++)
	{
		tl.enter("t");
		const std::string s1 =
			doc["repositories"]["imu_compass"]["release"]["url"]
				.as<std::string>();
		const std::string s2 =
			doc["release_platforms"]["ubuntu"][0].as<std::string>();
		tl.leave("t");
		sTotal += s1;
		sTotal += s2;
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	ASSERT_EQUAL_(sTotal.size(), 62U * reps);
	return r;
}
double yaml_yamlcpp_iterate(int, int)
{
	if (!prepareYamlTestFile()) return 0;

	const auto doc = YAML::LoadFile(fil);
	size_t visits = 0, allEntries = 0;
	std::string sAllKeys;

	mrpt::system::CTimeLogger tl;
	const unsigned int reps = 100;
	for (unsigned int i = 0; i < reps; i++)
	{
		tl.enter("t");
		for (const auto& kv : doc["repositories"])
		{
			visits++;
			sAllKeys += kv.first.as<std::string>();
			allEntries += kv.second.size();
		}
		tl.leave("t");
	}
	double r = tl.getMeanTime("t");
	tl.clear(true);	 // deep clear to silent dtor stats
	ASSERT_EQUAL_(visits, 1231U * reps);
	ASSERT_EQUAL_(sAllKeys.size(), 16849U * reps);
	ASSERT_EQUAL_(allEntries, 4099U * reps);
	return r;
}
#endif

// ------------------------------------------------------
// register_tests_yaml
// ------------------------------------------------------
void register_tests_yaml()
{
	lstTests.emplace_back("yaml: loadFromFile() big file", &yaml_loadFromFile);
	lstTests.emplace_back("yaml: FromFile() big file", &yaml_FromFile);
	lstTests.emplace_back("yaml: loadFromText() small", &yaml_loadFromText);
	lstTests.emplace_back("yaml: FromText() small", &yaml_FromText);
	lstTests.emplace_back("yaml: query in a big doc", &yaml_query);
	lstTests.emplace_back("yaml: iterate a big doc", &yaml_iterate);

#ifdef RUN_YAMLCPP_COMPARISON
	lstTests.emplace_back(
		"yamlcpp: LoadFile() big file", &yaml_yamlcpp_FromFile);
	lstTests.emplace_back("yamlcpp: Load() small", &yaml_yamlcpp_Load);
	lstTests.emplace_back("yamlcpp: query in a big doc", &yaml_yamlcpp_query);
	lstTests.emplace_back("yamlcpp: iterate a big doc", &yaml_yamlcpp_iterate);
#endif
}
