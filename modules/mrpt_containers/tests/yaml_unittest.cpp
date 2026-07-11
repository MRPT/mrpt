/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/containers/config.h>
#include <mrpt/containers/yaml.h>

#include <algorithm>  // count()
#include <fstream>

#include "mrpt_test.h"

MRPT_TEST(yaml, emptyCtor)
{
  {
    mrpt::containers::yaml p;
    EXPECT_TRUE(p.empty());
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, assignments)
{
  mrpt::containers::yaml p;
  p["K"] = 2.0;
  p["N"] = uint64_t(10);
  p["name"] = "Pepico";
  p["one"] = "1.0";
  p["enabled"] = true;
  p["visible"] = false;
  p["hidden"] = "true";
  p["val_of_size_t"] = size_t(10);
  auto nHidden1 = p["hidden"];

  EXPECT_FALSE(p.empty());
  EXPECT_TRUE(p.has("K"));

  EXPECT_FALSE(p.isScalar());
  EXPECT_TRUE(p["K"].isScalar());
  EXPECT_TRUE(p["N"].isScalar());

  const auto* n1 = &nHidden1.node();
  const auto* n2 = &p["hidden"].node();
  EXPECT_EQ(n1, n2);

  EXPECT_TRUE(p["enabled"]);
  EXPECT_FALSE(p["visible"]);
  EXPECT_TRUE(p["hidden"]);

  EXPECT_EQ(p["val_of_size_t"].as<size_t>(), size_t(10));

  EXPECT_EQ(p["K"].scalarType(), typeid(double));
  EXPECT_EQ(p["K"].as<double>(), 2.0);
  EXPECT_EQ(p["K"].as<float>(), 2.0f);
  {
    const double k = p["K"];
    EXPECT_EQ(k, 2.0);
  }

  EXPECT_EQ(p["N"].scalarType(), typeid(uint64_t));
  EXPECT_EQ(p["N"].as<uint64_t>(), 10U);

  EXPECT_EQ(p["name"].scalarType(), typeid(std::string));
  EXPECT_EQ(p["name"].as<std::string>(), "Pepico");
  {
    const std::string name = p["name"];
    EXPECT_EQ(name, "Pepico");
  }

  EXPECT_THROW(
      {
        const double name = p["k"];
        (void)name;
      },
      std::exception);

  EXPECT_EQ(p["enabled"].scalarType(), typeid(bool));

  {
    mrpt::containers::yaml p2;
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
MRPT_TEST_END()

MRPT_TEST(yaml, initializers)
{
  {
    const mrpt::containers::yaml p = mrpt::containers::yaml::Sequence({"K", 1.0});
    EXPECT_TRUE(p.isSequence());

    EXPECT_THROW(p(-1), std::exception);
    EXPECT_THROW(p(2), std::exception);

    EXPECT_TRUE(p(0).as<std::string>() == "K");
    EXPECT_TRUE(p(1).as<double>() == 1.0);

    const auto& seq = p.asSequence();
    EXPECT_TRUE(seq.size() == 2U);
  }
  {
    mrpt::containers::yaml p = mrpt::containers::yaml::Map({
        {"K", 1.0}
    });
    EXPECT_TRUE(p.isMap());
    for (const auto& kv : p.asMap())
    {
      EXPECT_TRUE(kv.first.as<std::string>() == "K");
      EXPECT_TRUE(kv.second.isScalar());
      EXPECT_TRUE(std::holds_alternative<double>(kv.second.asScalar()));
      EXPECT_TRUE(kv.second.as<double>() == 1.0);
    }
  }

  {
    auto p = mrpt::containers::yaml({"K", 1.0, 10.0});
    EXPECT_TRUE(p.isSequence());
  }
  {
    mrpt::containers::yaml p = mrpt::containers::yaml::Map({
        {   "K",   1.0},
        {   "T",  10.0},
        {"Name", "bar"}
    });
    EXPECT_TRUE(p.isMap());
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, initializerMap)
{
  const mrpt::containers::yaml p = mrpt::containers::yaml::Map({
      {   "K",                         2.0},
      {"book", std::string("silmarillion")}
  });

  EXPECT_FALSE(p.isSequence());
  EXPECT_TRUE(p.isMap());
  EXPECT_FALSE(p.empty());
  EXPECT_TRUE(p.has("K"));

  EXPECT_EQ(p["K"].scalarType(), typeid(double));
  EXPECT_EQ(p["K"].as<double>(), 2.0);

  EXPECT_EQ(p.getOrDefault("K", 1.0), 2.0);
  EXPECT_EQ(p.getOrDefault("Q", 1.0), 1.0);
  EXPECT_EQ(p.getOrDefault<uint32_t>("K", 1), 2U);

  EXPECT_EQ(p["book"].scalarType(), typeid(std::string));
  EXPECT_EQ(p["book"].as<std::string>(), "silmarillion");

  // non existing in const object:
  EXPECT_THROW((void)p["foo"], std::exception);
  // Access wrong type in const object:
  EXPECT_THROW((void)p["K"].asRef<std::string>(), std::exception);
}
MRPT_TEST_END()

MRPT_TEST(yaml, initializerSequence)
{
  const auto seq1 = mrpt::containers::yaml({1.0, 2.0, 3.0});

  EXPECT_FALSE(seq1.empty());
  EXPECT_TRUE(seq1.isSequence());
  EXPECT_FALSE(seq1.isMap());
  EXPECT_THROW(seq1.has("foo"), std::exception);
  EXPECT_THROW((void)seq1["K"], std::exception);

  EXPECT_TRUE(static_cast<double>(seq1(0)) == 1.0);
  EXPECT_EQ(seq1(0).as<double>(), 1.0);
  EXPECT_EQ(seq1(0).as<double>(), 1.0);
  EXPECT_EQ(seq1(1).as<double>(), 2.0);
  EXPECT_EQ(seq1(2).as<double>(), 3.0);

  EXPECT_THROW(seq1(3), std::out_of_range);
  EXPECT_THROW(seq1(-1), std::out_of_range);

  EXPECT_EQ(seq1.size(), 3U);
  EXPECT_EQ(seq1(0).size(), 1U);

  auto seq2 = mrpt::containers::yaml({1.0, 2.0, 3.0});
  EXPECT_EQ(seq2(1).as<double>(), 2.0);

  seq2(1) = 42.0;
  EXPECT_EQ(seq2(1).as<double>(), 42.0);

  seq2(1) = "foo";
  EXPECT_EQ(seq2(1).as<std::string>(), std::string("foo"));

  seq2(1) = mrpt::containers::yaml::Map({
      {"K", 1.0}
  });
  EXPECT_EQ(seq2(1)["K"].as<double>(), 1.0);

  seq2.push_back(std::string("foo2"));
  seq2.push_back(9.0);

  EXPECT_EQ(seq2(3).as<std::string>(), std::string("foo2"));
  EXPECT_EQ(seq2(4).as<double>(), 9.0);
}
MRPT_TEST_END()

MRPT_TEST(yaml, nested)
{
  mrpt::containers::yaml p = mrpt::containers::yaml::Map({
      {"K", 2.0}
  });
  p["PID"] = mrpt::containers::yaml::Map({
      {"Kp",   10.0},
      {"Ti",   10.0},
      {"Kd", "-3.5"}
  });

  EXPECT_FALSE(p.empty());
  EXPECT_FALSE(p["PID"].empty());

  EXPECT_EQ(p["PID"]["Ti"].scalarType(), typeid(double));
  EXPECT_EQ(p["PID"]["Ti"].as<double>(), 10.0);

  EXPECT_EQ(p["PID"]["Kd"].as<double>(), -3.5);
  EXPECT_EQ(p["PID"]["Kd"].as<float>(), -3.5f);

  EXPECT_EQ(p["PID"].getOrDefault("Kd", 0.0), -3.5);
  EXPECT_EQ(p["PID"].getOrDefault("Kd", 0.0f), -3.5f);

  EXPECT_FALSE(p.isScalar());
  EXPECT_FALSE(p["PID"].isScalar());
  EXPECT_TRUE(p["PID"]["Kp"].isScalar());

  // empty() not valid for values:
  EXPECT_THROW((void)p["PID"]["Ti"].empty(), std::exception);
  EXPECT_THROW((void)p["PID"]["Ti"].has("xxx"), std::exception);
  EXPECT_THROW((void)p["PID"]["Ti"]["xxx"].scalarType(), std::exception);

  p["PID"]["Ti"].clear();
  EXPECT_TRUE(p["PID"]["Ti"].isNullNode());

  // clear and recheck;
  p["PID"].clear();
  EXPECT_TRUE(p["PID"].empty());
}
MRPT_TEST_END()

MRPT_TEST(yaml, nested2)
{
  mrpt::containers::yaml p;
  p["N"] = 10;
  p["PID"] = mrpt::containers::yaml::Map();
  auto pid = p["PID"];
  pid["Kp"] = 0.5;
  p["PID"]["Ti"] = 2.0;
  p["PID"]["N"] = 1000;
  p["PID"]["name"] = "foo";

  EXPECT_EQ(p["PID"]["Kp"].as<double>(), 0.5);
  EXPECT_EQ(p["PID"]["Ti"].as<double>(), 2.0);
  EXPECT_EQ(p["PID"]["N"].as<uint64_t>(), 1000U);
  EXPECT_EQ(p["PID"]["name"].as<std::string>(), std::string("foo"));

  EXPECT_EQ(p.size(), 2U);
  EXPECT_EQ(p["PID"].size(), 4U);
}
MRPT_TEST_END()

const mrpt::containers::yaml testMap = mrpt::containers::yaml::Map({
    {         "K",                                                                                      2.0},
    {      "book",                                                                           "silmarillion"},
    {"mySequence", mrpt::containers::yaml::Sequence(
 {1.0, 2.0, mrpt::containers::yaml::Map({{"P", 1.0}, {"Q", 2.0}})})                      },
    {"myEmptyVal",                                                                                       {}},
    {    "myDict",                        mrpt::containers::yaml::Map({{"A", 1.0}, {"B", 2.0}, {"C", 3.0}})}
});

MRPT_TEST(yaml, printYAML)
{
  mrpt::containers::YamlEmitOptions eo;
  eo.emitHeader = false;

  {
    std::stringstream ss;
    testMap.printAsYAML(ss, eo);
    const auto s = ss.str();
    EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 13U);
  }
  {
    std::stringstream ss;
    ss << testMap;
    const auto s = ss.str();
    EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 13U);
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, printDebugStructure)
{
  std::stringstream ss;
  testMap.printDebugStructure(ss);
  const auto s = ss.str();
  EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 44U);
}
MRPT_TEST_END()

MRPT_TEST(yaml, ctorMap)
{
  mrpt::containers::yaml c1 = mrpt::containers::yaml::Map();
  c1["K"] = 2.0;
  c1["myDict"] = mrpt::containers::yaml::Map();
  auto m = c1["myDict"];
  m["A"] = 1.0;
  m["B"] = 2.0;

  const mrpt::containers::yaml c2 = mrpt::containers::yaml::Map({
      {     "K",                                                   2.0},
      {"myDict", mrpt::containers::yaml::Map({{"A", 1.0}, {"B", 2.0}})}
  });

  std::stringstream ss1, ss2;
  c1.printAsYAML(ss1);
  c2.printAsYAML(ss2);
  EXPECT_EQ(ss1.str(), ss2.str());
}
MRPT_TEST_END()

MRPT_TEST(yaml, comments)
{
  using mrpt::containers::CommentPosition;

  mrpt::containers::yaml c1 = mrpt::containers::yaml::Map();
  c1["v"] = 0;

  c1["K"] = 3.14;
  c1["K"].comment("Form factor");

  c1["T"] = 27;
  c1.keyComment("T", "Temperature [C]");
  c1["T"].comment("Measured today");

  EXPECT_FALSE(c1["v"].hasComment());

  EXPECT_TRUE(c1["K"].hasComment());
  EXPECT_EQ(c1["K"].comment(), "Form factor");

  EXPECT_TRUE(c1.keyHasComment("T"));
  EXPECT_TRUE(c1["T"].hasComment());
  EXPECT_THROW(c1["T"].keyHasComment("T"), std::exception);

  using mrpt::containers::vcp;
  using mrpt::containers::vkcp;

  // Test vkcp:
  c1 << vkcp("L", 5.5, "Arm length [meters]");

  EXPECT_TRUE(c1.keyHasComment("L"));
  EXPECT_EQ(c1.keyComment("L"), "Arm length [meters]");

  EXPECT_NE(&c1.keyNode("L"), &c1["L"].node());

  EXPECT_EQ(c1.keyNode("L").as<std::string>(), "L");
  EXPECT_TRUE(c1.keyNode("L").hasComment(mrpt::containers::CommentPosition::TOP));

  // Chained vkcp:
  c1 << vkcp("A", 1.0, "Comment for A") << vkcp("B", 4.5, "Comment for B");
  EXPECT_TRUE(c1.keyHasComment("A"));
  EXPECT_TRUE(c1.keyHasComment("B"));

  c1["D"] = vcp(3.0, "Distance [meters]", CommentPosition::RIGHT);
  EXPECT_TRUE(c1["D"].hasComment());
  EXPECT_EQ(c1["D"].comment(CommentPosition::RIGHT), "Distance [meters]");

  // c1.printAsYAML();
  // c1.printDebugStructure(std::cout);

  mrpt::containers::yaml c2 = mrpt::containers::yaml::Map();
  c2["constants"] = c1;
  c2.keyComment("constants", "Universal constant definitions:");
  c2["constants"].comment("Another comment", CommentPosition::RIGHT);

  EXPECT_TRUE(c2["constants"].comment(CommentPosition::RIGHT) == "Another comment");
  EXPECT_TRUE(c2.keyComment("constants") == "Universal constant definitions:");

  // c2.printAsYAML(std::cout);
}
MRPT_TEST_END()

MRPT_TEST(yaml, iterate)
{
  std::stringstream ss;

  std::set<std::string> foundKeys;
  for (const auto& kv : testMap.asMap())
  {
    ss << kv.first.as<std::string>() << ":" << kv.second.typeName() << "\n";

    foundKeys.insert(kv.first.as<std::string>());
  }
  EXPECT_EQ(foundKeys.size(), 5U);

  foundKeys.clear();
  for (const auto& kv : testMap["myDict"].asMapRange())
  {
    ss << kv.first << ":" << kv.second.typeName() << "\n";
    foundKeys.insert(kv.first.as<std::string>());
  }
  EXPECT_EQ(foundKeys.size(), 3U);

  EXPECT_EQ(testMap["mySequence"].asSequence().size(), 3U);
}
MRPT_TEST_END()

MRPT_TEST(yaml, macros)
{
  mrpt::containers::yaml p;
  p["K"] = 2.0;
  p["Ang"] = 90.0;
  p["name"] = "Pepico";
  p["PID"] = mrpt::containers::yaml::Map({
      {"Kp", 1.0},
      {"Td", 0.8}
  });

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

  {
    mrpt::containers::yaml p2;
    int i = 10;
    MCP_SAVE(p2, i);

    EXPECT_EQ(p2["i"].as<int>(), 10);

    // TODO(jlbc): Add another enum test
#if 0
    {
      mrpt::system::VerbosityLevel vl = mrpt::system::LVL_WARN;
      MCP_SAVE(p2, vl);
    }

    EXPECT_EQ(p2["vl"].as<std::string>(), "WARN");

    {
      mrpt::system::VerbosityLevel vl;
      MCP_LOAD_REQ(p2, vl);
      EXPECT_EQ(vl, mrpt::system::LVL_WARN);
    }
    {
      mrpt::system::VerbosityLevel vl = mrpt::system::LVL_ERROR;
      MCP_LOAD_OPT(p2, vl);
      EXPECT_EQ(vl, mrpt::system::LVL_WARN);
    }
    {
      auto p3 = p2;
      mrpt::system::VerbosityLevel vl = mrpt::system::LVL_ERROR;
      p3["vl"] = "FakeEnumValue";
      EXPECT_THROW(MCP_LOAD_OPT(p3, vl), std::exception);
    }
#endif
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, hexadecimal)
{
  mrpt::containers::yaml p;
  p["color"] = "0x112233";

  const int c1 = 0x112233;
  const int c2 = p["color"].as<int>();

  EXPECT_EQ(c1, c2);
}
MRPT_TEST_END()

void foo(mrpt::containers::yaml_ref p)
{
  p["K"] = 2.0;
  p["N"] = 2;
}

MRPT_TEST(yaml, assignmentsInCallee)
{
  mrpt::containers::yaml p;

  p["params"] = mrpt::containers::yaml::Map();
  auto pp = p["params"];
  foo(pp);

  EXPECT_FALSE(p.empty());
  EXPECT_TRUE(p["params"].isMap());

  EXPECT_TRUE(p["params"].has("K"));
  EXPECT_TRUE(p["params"].has("N"));

  EXPECT_TRUE(p["params"]["K"].isScalar());
  EXPECT_EQ(p["params"]["K"].scalarType(), typeid(double));
  EXPECT_EQ(p["params"]["K"].as<double>(), 2.0);
  EXPECT_EQ(p["params"]["N"].as<int>(), 2);
}
MRPT_TEST_END()

#if MRPT_HAS_FYAML

// clang-format off
const auto sampleYamlBlock_1 = R"xxx(
~
)xxx";

const auto sampleYamlBlock_2 = R"xxx(
---
foo  # comment
)xxx";

const auto sampleYamlBlock_3 = R"xxx(
# blah blah
mySeq:
  - "first"
  - "second"
  - "third"
  - ~
myMap:
  K: 10.0
  Q: ~
  P: -5.0
  # comment for nestedMap map
  nestedMap:
    a: 1  # comment for a
    # comment for b
    b: 2
    c: 3
  R: 123
  S: -123
  T: 123 # foo
  V: 456 # bar
)xxx";

const auto sampleYamlBlock_4 = R"xxx(
myMap:
  e1: 10.0  # Right comment for e1 value
myMap2:
  # Top comment for e2
  e2: 10.0
myMap3:
  # Top comment for e3
  e3: 10.0 # right comment for e3 value
# Top comment for myMap4
myMap4:
  ~
# Top comment for myMap5
myMap5:
  a4: 1 # right comment for a4
)xxx";

// clang-format on

MRPT_TEST(yaml, fromYAML)
{
  {
    auto p = mrpt::containers::yaml::FromText("");
    EXPECT_TRUE(p.isNullNode());
  }

  {
    auto p = mrpt::containers::yaml::FromText(sampleYamlBlock_1);
    EXPECT_TRUE(p.isScalar());
    EXPECT_TRUE(p.isNullNode());
  }
  {
    auto p = mrpt::containers::yaml::FromText(sampleYamlBlock_2);

    EXPECT_TRUE(p.isScalar());
    EXPECT_TRUE(p.as<std::string>() == "foo");
  }

  {
    auto p = mrpt::containers::yaml::FromText(sampleYamlBlock_3);

    EXPECT_EQ(p["mySeq"](0).as<std::string>(), "first");
    EXPECT_EQ(p["myMap"]["P"].as<double>(), -5.0);
    EXPECT_EQ(p["myMap"]["K"].as<double>(), 10.0);

    EXPECT_FALSE(p.isNullNode());
    EXPECT_FALSE(p["myMap"].isNullNode());

    EXPECT_TRUE(p["mySeq"](3).isNullNode());
    EXPECT_TRUE(p["myMap"]["Q"].isNullNode());
    EXPECT_FALSE(p["myMap"]["K"].isNullNode());

    const auto& e = p["myMap"]["nestedMap"]["a"];
    EXPECT_TRUE(e.hasComment());

    using mrpt::containers::CommentPosition;
    EXPECT_FALSE(e.hasComment(CommentPosition::TOP));
    EXPECT_TRUE(e.hasComment(CommentPosition::RIGHT));

    EXPECT_EQ(e.comment(), "comment for a");
    EXPECT_EQ(e.comment(CommentPosition::RIGHT), "comment for a");
    EXPECT_THROW(auto s = e.comment(CommentPosition::TOP), std::exception);

    EXPECT_EQ(e.node().marks.line, 13);
    EXPECT_EQ(e.node().marks.column, 7);
    EXPECT_EQ(e.node().marks.input_pos, 147);

#if 0
		p.printDebugStructure(std::cout);
		p.printAsYAML();
#endif

    EXPECT_TRUE(p["myMap"]["nestedMap"].keyHasComment("b"));
    EXPECT_EQ(p["myMap"]["nestedMap"].keyComment("b"), "comment for b");
  }

  {
    const auto p = mrpt::containers::yaml::FromText(sampleYamlBlock_4);

    EXPECT_EQ(p["myMap"]["e1"].comment(), "Right comment for e1 value");
    EXPECT_FALSE(p["myMap4"].hasComment());
    EXPECT_TRUE(p["myMap5"]["a4"].hasComment());
    EXPECT_TRUE(p.keyHasComment("myMap5"));
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, printInShortFormat)
{
  mrpt::containers::yaml n1 = mrpt::containers::yaml::Sequence({1, 2, 3});

  n1.node().printInShortFormat = true;

  mrpt::containers::YamlEmitOptions eo;
  eo.emitHeader = false;

  {
    std::stringstream ss;
    n1.printAsYAML(ss, eo);
    EXPECT_EQ(ss.str(), "[1, 2, 3]\n");
  }

  mrpt::containers::yaml n2 = mrpt::containers::yaml::Map();
  n2["foo"] = 1.0;
  n2["bar"] = n1;

  {
    std::stringstream ss;
    n2.printAsYAML(ss, eo);
    EXPECT_EQ(ss.str(), "foo: 1.0\nbar: [1, 2, 3]\n");  // insertion order
  }

  mrpt::containers::yaml n3 = mrpt::containers::yaml::Map();
  n3["alpha"] = 1.0;
  n3["beta"] = n2;

  {
    std::stringstream ss;
    n3.printAsYAML(ss, eo);
    EXPECT_EQ(ss.str(), "alpha: 1.0\nbeta:\n  foo: 1.0\n  bar: [1, 2, 3]\n");  // insertion order
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, outOfRangeIntegers)
{
  mrpt::containers::yaml p;
  p["N1"] = "1292889";
  p["N2"] = "1171717171717171771782288282822129288118189";
  p["N3"] = "65535";
  p["N4"] = "65536";

  EXPECT_EQ(p["N1"].as<int>(), 1292889);
  EXPECT_THROW((void)p["N2"].as<int>(), std::exception);
  EXPECT_EQ(p["N3"].as<uint16_t>(), 65535);
  EXPECT_THROW((void)p["N4"].as<uint16_t>(), std::exception);
}
MRPT_TEST_END()

// clang-format off
const auto testYamlParseEmit_1 =
R"xxx(# comment line 1, and
# comment line 2
1.0
)xxx";

const auto testYamlParseEmit_2 =
R"xxx(- a  # comment for A
- b  # comment for B
- c  # comment for C
-
  d1: xxx  # cool
  d2: xxx  # facts
)xxx";

const auto testYamlParseEmit_3 =
R"xxx(a: 1.0  # A comment
b: 2.0  # B comment
)xxx";

const auto testYamlParseEmit_4 =
R"xxx(plain scalars:
  - a string
  - a string with a \ backslash that doesn't need to be escaped
  - can also use " quotes ' and $ a % lot /&?+ of other {} [] stuff
)xxx";

const auto testYamlParseEmit_5 =
R"xxx(literal: |
  a
  b
literal block scalar: |
  a multiline text
  line 2
  line 3
literal2: |-
  a
  b
)xxx";

const auto testYamlParseEmit_6 =
R"xxx(release_platforms:
  ubuntu:
  - focal
repositories:
  ament_cmake:
    doc:
      type: git
      url: https://github.com/ament/ament_cmake.git
      version: foxy
    release:
      packages:
      - ament_cmake
      - ament_cmake_auto
      - ament_cmake_core
      - ament_cmake_export_definitions
      - ament_cmake_export_dependencies
      - ament_cmake_export_include_directories
      - ament_cmake_export_interfaces
      - ament_cmake_export_libraries
      - ament_cmake_export_link_flags
      - ament_cmake_export_targets
      - ament_cmake_gmock
      - ament_cmake_gtest
      - ament_cmake_include_directories
      - ament_cmake_libraries
      - ament_cmake_nose
      - ament_cmake_pytest
      - ament_cmake_python
      - ament_cmake_target_dependencies
      - ament_cmake_test
      - ament_cmake_version
      tags:
        release: release/foxy/{package}/{version}
      url: https://github.com/ros2-gbp/ament_cmake-release.git
      version: 0.9.6-1
    source:
      test_pull_requests: true
      type: git
      url: https://github.com/ament/ament_cmake.git
      version: foxy
    status: developed
  ament_cmake_ros:
    doc:
      type: git
      url: https://github.com/ros2/ament_cmake_ros.git
      version: foxy
    release:
      packages:
      - ament_cmake_ros
      - domain_coordinator
      tags:
        release: release/foxy/{package}/{version}
      url: https://github.com/ros2-gbp/ament_cmake_ros-release.git
      version: 0.9.0-1
    source:
      test_pull_requests: true
      type: git
      url: https://github.com/ros2/ament_cmake_ros.git
      version: foxy
    status: maintained
  ament_index:
    doc:
      type: git
      url: https://github.com/ament/ament_index.git
      version: foxy
    release:
      packages:
      - ament_index_cpp
      - ament_index_python
      tags:
        release: release/foxy/{package}/{version}
      url: https://github.com/ros2-gbp/ament_index-release.git
      version: 1.0.0-1
    source:
      test_abi: true
      test_pull_requests: true
      type: git
      url: https://github.com/ament/ament_index.git
      version: foxy
    status: maintained
)xxx";
// clang-format on

MRPT_TEST(yaml, parseAndEmit)
{
  using mrpt::containers::CommentPosition;

  const std::vector<std::string> tsts = {
      //
      testYamlParseEmit_1, testYamlParseEmit_2, testYamlParseEmit_3,
      testYamlParseEmit_4, testYamlParseEmit_5,
      testYamlParseEmit_6  // "indentSequences=false" for this one
                           //
  };

#ifdef __EMSCRIPTEN__
  const bool hasYamlLint = false;
#else
  const bool hasYamlLint = (0 == ::system("yamllint --version"));
#endif

  int idx = 0;
  for (const auto& testText : tsts)
  {
    mrpt::containers::yaml p = mrpt::containers::yaml::FromText(testText);
    std::stringstream ss;

    mrpt::containers::YamlEmitOptions eo;
    eo.emitHeader = false;
    eo.indentSequences = idx != 5;

    p.printAsYAML(ss, eo);

    EXPECT_EQ(testText, ss.str())  //
        << "=== Input:\n"
        << testText << "=== Output:\n"
        << ss.str() << "=== Debug dump of test yaml doc [" << idx
        << "]:\n",  // Yes, it is an intentional ","
        p.printDebugStructure(std::cout);

    // Test with yamllint
    if (hasYamlLint)
    {
      const auto tmpFil = mrpt::format("/tmp/aaa_%i.yaml", idx);

      std::ofstream f(tmpFil);
      if (f.is_open())
      {
        p.printAsYAML(f);

        const auto sCmd = mrpt::format("yamllint %s -f parsable", tmpFil.c_str());

        // std::cout << "Running '" << sCmd << "'...\n";
        // std::string lintOut;
        // const int ret = mrpt::system::executeCommand(sCmd, &lintOut);
        const int ret = ::system(sCmd.c_str());
        EXPECT_EQ(ret, 0);
        // if (!lintOut.empty()) GTEST_FAIL() << "Linter output:\n" << lintOut << "\n";
      }
    }

    idx++;
  }
}
MRPT_TEST_END()

// clang-format off

const auto sampleJSONBlock_1 = std::string(R"xxx(
{"store":{"book":[{"category":"reference","author":"Nigel Rees","title":"Sayings of the Century","price":8.95},{"category":"fiction","author":"Evelyn Waugh","title":"Sword of Honour","price":12.99},{"category":"fiction","author":"J. R. R. Tolkien","title":"The Lord of the Rings","isbn":"0-395-19395-8","price":22.99}],"bicycle":{"color":"red","price":19.95}}}
)xxx");

const auto sampleJSONBlock_2 = std::string(R"xxx(
{
  "data": [{
    "type": "articles",
    "id": "1",
    "attributes": {
      "title": "JSON:API paints my bikeshed!",
      "body": "The shortest article. Ever."
    }
  }],
  "included": [
    {
      "type": "people",
      "id": "42",
      "attributes": {
        "name": "John"
      }
    }
  ]
}
)xxx");

// clang-format on

MRPT_TEST(yaml, fromJSON)
{
  {
    const auto p = mrpt::containers::yaml::FromText(sampleJSONBlock_1);
    // p.printAsYAML(std::cout);

    EXPECT_TRUE(p.has("store"));
    EXPECT_EQ(p["store"]["bicycle"]["color"].as<std::string>(), "red");
  }
  {
    const auto p = mrpt::containers::yaml::FromText(sampleJSONBlock_2);
    // p.printAsYAML(std::cout);

    EXPECT_TRUE(p.has("data"));
    EXPECT_EQ(p["data"](0)["id"].as<std::string>(), "1");
    EXPECT_EQ(p["included"](0)["id"].as<int>(), 42);
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, booleanVersions)
{
  mrpt::containers::yaml p;
  p["b01"] = "0";
  p["b02"] = "False";
  p["b03"] = "false";
  p["b04"] = "FALSE";
  p["b05"] = "No";

  p["b11"] = "1";
  p["b12"] = "True";
  p["b13"] = "true";
  p["b14"] = "TRUE";
  p["b15"] = "Yes";

  EXPECT_EQ(p["b01"].as<bool>(), false);
  EXPECT_EQ(p["b02"].as<bool>(), false);
  EXPECT_EQ(p["b03"].as<bool>(), false);
  EXPECT_EQ(p["b04"].as<bool>(), false);
  EXPECT_EQ(p["b05"].as<bool>(), false);

  EXPECT_EQ(p["b11"].as<bool>(), true);
  EXPECT_EQ(p["b12"].as<bool>(), true);
  EXPECT_EQ(p["b13"].as<bool>(), true);
  EXPECT_EQ(p["b14"].as<bool>(), true);
  EXPECT_EQ(p["b15"].as<bool>(), true);
}
MRPT_TEST_END()

MRPT_TEST(yaml, integerConversions)
{
  {
    mrpt::containers::yaml p;
    p["a1"] = "0x1122";
    p["a2"] = 0x1122;

    EXPECT_EQ(p["a1"].as<int>(), 0x1122);
    EXPECT_EQ(p["a2"].as<int>(), 0x1122);

    p["b1"] = +12345;
    p["b2"] = "+12345";
    p["b3"] = "+12345";
    p["b4"] = "12345abc";
    p["b5_bad"] = "abc12345";

    EXPECT_EQ(p["b1"].as<int>(), 12345);
    EXPECT_EQ(p["b2"].as<int>(), 12345);
    EXPECT_EQ(p["b3"].as<int>(), 12345);
    EXPECT_EQ(p["b4"].as<int>(), 12345);
    EXPECT_ANY_THROW((void)p["b5_bad"].as<int>());

    p["c1"] = -12345;
    p["c2"] = "-12345";
    p["c3"] = " -12345";

    EXPECT_EQ(p["c1"].as<int>(), -12345);
    EXPECT_EQ(p["c2"].as<int>(), -12345);
    EXPECT_EQ(p["c3"].as<int>(), -12345);
  }

  // Make sure the comments do not affect the result:
  //  R: 123
  //  S: -123
  //  T: 123 # foo
  //  V: 456 # bar
  {
    auto p = mrpt::containers::yaml::FromText(sampleYamlBlock_3);

    EXPECT_EQ(p["myMap"]["R"].as<int>(), 123);
    EXPECT_EQ(p["myMap"]["S"].as<int>(), -123);
    EXPECT_EQ(p["myMap"]["T"].as<int>(), 123);
    EXPECT_EQ(p["myMap"]["V"].as<int>(), 456);
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, narrowIntegerAssignments)
{
  mrpt::containers::yaml p;
  p["i8"] = int8_t(-8);
  p["u8"] = uint8_t(8);
  p["i16"] = int16_t(-16);
  p["u16"] = uint16_t(16);
  p["i32"] = int32_t(-32);
  p["u32"] = uint32_t(32);
  p["f"] = 1.5f;

  EXPECT_EQ(p["i8"].as<int>(), -8);
  EXPECT_EQ(p["u8"].as<int>(), 8);
  EXPECT_EQ(p["i16"].as<int>(), -16);
  EXPECT_EQ(p["u16"].as<int>(), 16);
  EXPECT_EQ(p["i32"].as<int>(), -32);
  EXPECT_EQ(p["u32"].as<int>(), 32);
  EXPECT_EQ(p["f"].as<double>(), 1.5);
}
MRPT_TEST_END()

MRPT_TEST(yaml, typeNameVariants)
{
  using mrpt::containers::yaml;

  {
    yaml::node_t n;  // default: null node (holds std::monostate directly)
    EXPECT_EQ(n.typeName(), "null");
  }
  {
    yaml::node_t n(true);
    EXPECT_EQ(n.typeName(), "scalar(bool)");
  }
  {
    yaml::node_t n(uint64_t(42));
    EXPECT_EQ(n.typeName(), "scalar(uint64_t)");
  }
  {
    yaml::node_t n(int64_t(-42));
    EXPECT_EQ(n.typeName(), "scalar(int64_t)");
  }
  {
    yaml::node_t n(3.14);
    EXPECT_EQ(n.typeName(), "scalar(double)");
  }
  {
    yaml::node_t n(std::string("foo"));
    EXPECT_EQ(n.typeName(), "scalar(std::string)");
  }
  {
    // A scalar node that itself holds std::monostate (as opposed to a null d):
    yaml p;
    p["a"];  // auto-creates a null child, but as a plain node (not scalar_t)
    EXPECT_EQ(p["a"].node().typeName(), "null");
  }
}
MRPT_TEST_END()

MRPT_TEST(yaml, mutableAsAccessors)
{
  using mrpt::containers::yaml;

  yaml p = yaml::Sequence({1.0, 2.0});
  yaml::sequence_t& seq = p.asSequence();
  EXPECT_EQ(seq.size(), 2U);

  yaml m = yaml::Map({
      {"K", 1.0}
  });
  yaml::map_t& mm = m.asMap();
  EXPECT_EQ(mm.size(), 1U);

  yaml s;
  s = 5.0;
  yaml::scalar_t& sc = s.asScalar();
  EXPECT_TRUE(std::holds_alternative<double>(sc));

  // Same for the underlying node_t:
  yaml::node_t& mutSeqNode = p.node();
  EXPECT_EQ(mutSeqNode.asSequence().size(), 2U);

  yaml::node_t& mutMapNode = m.node();
  EXPECT_EQ(mutMapNode.asMap().size(), 1U);

  yaml::node_t& mutScalarNode = s.node();
  EXPECT_TRUE(std::holds_alternative<double>(mutScalarNode.asScalar()));
}
MRPT_TEST_END()

MRPT_TEST(yaml, nullNodeEdgeCases)
{
  using mrpt::containers::yaml;

  yaml p;
  EXPECT_TRUE(p.isNullNode());
  EXPECT_FALSE(p.has("anything"));          // has() on a null node returns false
  EXPECT_EQ(p.scalarType(), typeid(void));  // scalarType() on a null node

  // read operator[] on a null node throws (only the const overload throws;
  // the non-const overload auto-vivifies the node into a map instead):
  const yaml constNullNode;
  EXPECT_THROW((void)constNullNode["x"], std::exception);
  EXPECT_NO_THROW((void)p["x"]);
  EXPECT_TRUE(p.isMap());

  // write operator[] on a non-map (scalar) node throws:
  yaml scalarNode;
  scalarNode = 1.0;
  EXPECT_THROW(scalarNode["x"] = 2.0, std::exception);

  // read operator[] on a non-map (scalar) node throws:
  const yaml constScalarNode = scalarNode;
  EXPECT_THROW((void)constScalarNode["x"], std::exception);
}
MRPT_TEST_END()

MRPT_TEST(yaml, emptyThrowsOnScalar)
{
  mrpt::containers::yaml p;
  p = 3.0;
  EXPECT_THROW((void)p.empty(), std::exception);
}
MRPT_TEST_END()

MRPT_TEST(yaml, eraseMissingOrOutOfRange)
{
  mrpt::containers::yaml m = mrpt::containers::yaml::Map({
      {"K", 1.0}
  });
  EXPECT_EQ(m.erase("notThere"), 0U);
  EXPECT_EQ(m.erase("K"), 1U);
  EXPECT_TRUE(m.empty());

  mrpt::containers::yaml s = mrpt::containers::yaml::Sequence({1.0, 2.0});
  EXPECT_FALSE(s.erase(-1));
  EXPECT_FALSE(s.erase(2));
  EXPECT_TRUE(s.erase(0));
  EXPECT_EQ(s.asSequence().size(), 1U);
}
MRPT_TEST_END()

MRPT_TEST(yaml, fromStream)
{
  std::stringstream ss;
  ss << "a: 1\nb: two\n";

  auto p = mrpt::containers::yaml::FromStream(ss);
  EXPECT_EQ(p["a"].as<int>(), 1);
  EXPECT_EQ(p["b"].as<std::string>(), "two");

  mrpt::containers::yaml q;
  std::stringstream ss2;
  ss2 << "c: 3\n";
  q.loadFromStream(ss2);
  EXPECT_EQ(q["c"].as<int>(), 3);
}
MRPT_TEST_END()

MRPT_TEST(yaml, loadFromFileMissing)
{
  EXPECT_THROW(
      mrpt::containers::yaml::FromFile("/nonexistent/path/does_not_exist.yaml"), std::exception);
}
MRPT_TEST_END()

MRPT_TEST(yaml, printAsYAMLToStdout)
{
  mrpt::containers::yaml p = mrpt::containers::yaml::Map({
      {"K", 1.0}
  });
  // Just exercise the no-arg overload that writes to std::cout; nothing to assert.
  testing::internal::CaptureStdout();
  p.printAsYAML();
  const std::string out = testing::internal::GetCapturedStdout();
  EXPECT_NE(out.find("K:"), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, doubleAsString)
{
  mrpt::containers::yaml p;
  p["x"] = 3.5;
  EXPECT_EQ(p["x"].as<std::string>(), "3.5");
}
MRPT_TEST_END()

MRPT_TEST(yaml, nullScalarAsBoolAndThrow)
{
  mrpt::containers::yaml p;
  p["a"];  // auto-created, still a null node (not a scalar)
  // as<T>() is only valid on scalar nodes, so it throws on a null node:
  EXPECT_THROW((void)p["a"].as<bool>(), std::exception);
  EXPECT_THROW((void)p["a"].as<int>(), std::exception);

  // A scalar node holding a null value (monostate) converts to false for bool.
  // This is distinct from a "null node" (no scalar at all): parse it from YAML
  // so it is a genuine null *scalar*:
  mrpt::containers::yaml q = mrpt::containers::yaml::FromText("a: null\n");
  EXPECT_TRUE(q["a"].isScalar());
  EXPECT_FALSE(q["a"].as<bool>());
}
MRPT_TEST_END()

MRPT_TEST(yaml, longCommentShortenedInDebugStructure)
{
  using mrpt::containers::CommentPosition;

  mrpt::containers::yaml p = mrpt::containers::yaml::Map({
      {"K", 1.0}
  });
  p["K"].comment("This is a very long top comment that must be shortened", CommentPosition::TOP);
  p["K"].comment("Right comment also quite long here", CommentPosition::RIGHT);

  std::stringstream ss;
  p.printDebugStructure(ss);
  const auto s = ss.str();
  EXPECT_NE(s.find("..."), std::string::npos);

  // A node with no comments prints "[none]":
  mrpt::containers::yaml q = mrpt::containers::yaml::Map({
      {"V", 2.0}
  });
  std::stringstream ss2;
  q.printDebugStructure(ss2);
  EXPECT_NE(ss2.str().find("[none]"), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, multilineRightComment)
{
  using mrpt::containers::CommentPosition;

  mrpt::containers::yaml p;
  p["x"] = 1.0;
  p["x"].comment("line one\nline two", CommentPosition::RIGHT);

  std::stringstream ss;
  p.printAsYAML(ss);
  const auto s = ss.str();
  // Newlines in a right-side comment are stripped:
  EXPECT_EQ(s.find("line one\nline two"), std::string::npos);
  EXPECT_NE(s.find("line oneline two"), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, nullScalarWithRightComment)
{
  using mrpt::containers::CommentPosition;

  mrpt::containers::yaml p;
  p["a"];  // auto-created null (non-scalar) node
  p["a"].comment("a right comment", CommentPosition::RIGHT);

  std::stringstream ss;
  p.printAsYAML(ss);
  EXPECT_NE(ss.str().find("a right comment"), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, shortFormatSkippedForNonScalarEntries)
{
  // A sequence with a nested map entry cannot be printed in short format,
  // even if requested: the printer silently falls back to the long form.
  mrpt::containers::yaml n = mrpt::containers::yaml::Sequence({1.0});
  n.asSequence().push_back(mrpt::containers::yaml::Map({
      {"K", 1.0}
  }));
  n.node().printInShortFormat = true;

  mrpt::containers::YamlEmitOptions eo;
  eo.emitHeader = false;

  std::stringstream ss;
  n.printAsYAML(ss, eo);
  const auto s = ss.str();

  // Long format uses "- " list markers; short format would use "[...]":
  EXPECT_NE(s.find("- "), std::string::npos);
  EXPECT_EQ(s.find('['), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, integerIndexOperators)
{
  using mrpt::containers::yaml;

  yaml p = yaml::Sequence({10.0, 20.0, 30.0});

  // yaml::operator[](int) / operator()(int), read and write:
  EXPECT_NEAR(p[1].as<double>(), 20.0, 1e-9);
  EXPECT_NEAR(p(1).as<double>(), 20.0, 1e-9);
  p(1) = 99.0;
  EXPECT_NEAR(p[1].as<double>(), 99.0, 1e-9);

  const yaml& cp = p;
  EXPECT_NEAR(cp[1].as<double>(), 99.0, 1e-9);
  EXPECT_NEAR(cp(1).as<double>(), 99.0, 1e-9);

  EXPECT_THROW((void)p(10), std::out_of_range);
  EXPECT_THROW((void)cp(10), std::out_of_range);

  // erase(int):
  EXPECT_TRUE(p.erase(0));
  EXPECT_EQ(p.asSequence().size(), 2U);
  EXPECT_FALSE(p.erase(100));

  // Nested map inside a sequence, exercised through yaml_ref/yaml_cref
  // integer index operators:
  yaml outer = yaml::Map({
      {"seq", yaml::Sequence({1.0, 2.0})}
  });
  outer["seq"][1] = 42.0;
  EXPECT_NEAR(outer["seq"][1].as<double>(), 42.0, 1e-9);
  EXPECT_THROW((void)outer["seq"][100], std::out_of_range);

  const yaml& coutgroup = outer;
  EXPECT_NEAR(coutgroup["seq"][1].as<double>(), 42.0, 1e-9);
  EXPECT_THROW((void)coutgroup["seq"][100], std::out_of_range);
}
MRPT_TEST_END()

MRPT_TEST(yaml, streamInsertionOperator)
{
  using mrpt::containers::yaml;

  yaml p = yaml::Map({
      {"k", 1.0}
  });

  std::stringstream ss1;
  ss1 << p;
  EXPECT_NE(ss1.str().find("k:"), std::string::npos);

  std::stringstream ss2;
  ss2 << p["k"];  // yaml_ref
  EXPECT_NE(ss2.str().find('1'), std::string::npos);

  const yaml& cp = p;
  std::stringstream ss3;
  ss3 << cp["k"];  // yaml_cref
  EXPECT_NE(ss3.str().find('1'), std::string::npos);
}
MRPT_TEST_END()

MRPT_TEST(yaml, keyCommentsAPI)
{
  using mrpt::containers::CommentPosition;
  using mrpt::containers::yaml;

  yaml p = yaml::Map({
      {"k", 1.0}
  });

  EXPECT_FALSE(p.keyHasComment("k"));
  EXPECT_FALSE(p.keyHasComment("k", CommentPosition::TOP));

  p.keyComment("k", "a key comment", CommentPosition::TOP);
  EXPECT_TRUE(p.keyHasComment("k"));
  EXPECT_TRUE(p.keyHasComment("k", CommentPosition::TOP));
  EXPECT_EQ(p.keyComment("k"), "a key comment");
  EXPECT_EQ(p.keyComment("k", CommentPosition::TOP), "a key comment");

  yaml::node_t& knMut = p.keyNode("k");
  EXPECT_EQ(knMut.internalAsStr(), "k");

  const yaml& cp = p;
  const yaml::node_t& kn = cp.keyNode("k");
  EXPECT_EQ(kn.internalAsStr(), "k");
  EXPECT_TRUE(cp.keyHasComment("k"));
  EXPECT_TRUE(cp.keyHasComment("k", CommentPosition::TOP));
  EXPECT_EQ(cp.keyComment("k"), "a key comment");
  EXPECT_EQ(cp.keyComment("k", CommentPosition::TOP), "a key comment");

  EXPECT_THROW((void)p.keyComment("missing"), std::exception);
}
MRPT_TEST_END()

MRPT_TEST(yaml, valueCommentsAPI)
{
  using mrpt::containers::CommentPosition;
  using mrpt::containers::yaml;

  // These calls on the root document object itself (as opposed to through
  // a yaml_ref/yaml_cref proxy from operator[]) exercise yaml::hasComment()/
  // comment() in yaml.cpp:
  yaml p = yaml::Map({
      {"k", 1.0}
  });

  EXPECT_FALSE(p.hasComment());
  EXPECT_FALSE(p.hasComment(CommentPosition::TOP));

  p.comment("root comment", CommentPosition::TOP);
  EXPECT_TRUE(p.hasComment());
  EXPECT_TRUE(p.hasComment(CommentPosition::TOP));
  EXPECT_EQ(p.comment(), "root comment");
  EXPECT_EQ(p.comment(CommentPosition::TOP), "root comment");

  // Proxy (yaml_ref/yaml_cref) comment accessors, inline in the header:
  EXPECT_FALSE(p["k"].hasComment());
  EXPECT_FALSE(p["k"].hasComment(CommentPosition::TOP));

  p["k"].comment("value comment", CommentPosition::TOP);
  EXPECT_TRUE(p["k"].hasComment());
  EXPECT_TRUE(p["k"].hasComment(CommentPosition::TOP));
  EXPECT_EQ(p["k"].comment(), "value comment");
  EXPECT_EQ(p["k"].comment(CommentPosition::TOP), "value comment");
}
MRPT_TEST_END()

MRPT_TEST(yaml, nestedRefChainedSubscript)
{
  using mrpt::containers::yaml;

  // Chaining subscripts on an already-obtained yaml_ref/yaml_cref exercises
  // yaml_ref::operator[](string/const char*/int) and yaml_cref's counterparts,
  // as opposed to the top-level yaml::operator[] overloads:
  yaml p = yaml::Map({
      {"outer", yaml::Map({{"inner", 1.0}})}
  });

  p["outer"]["inner"] = 2.0;
  EXPECT_NEAR(p["outer"]["inner"].as<double>(), 2.0, 1e-9);

  const char* innerKey = "inner";
  EXPECT_NEAR(p["outer"][innerKey].as<double>(), 2.0, 1e-9);

  const yaml& cp = p;
  EXPECT_NEAR(cp["outer"]["inner"].as<double>(), 2.0, 1e-9);
  EXPECT_NEAR(cp["outer"][innerKey].as<double>(), 2.0, 1e-9);

  // std::string keys (as opposed to string literals, which prefer the
  // `const char*` overloads) exercise the std::string overloads of
  // yaml_ref::operator[] and yaml_cref::operator[]:
  const std::string innerKeyStr = "inner";
  mrpt::containers::yaml_ref outerRef = p["outer"];
  outerRef[innerKeyStr] = 3.0;
  EXPECT_NEAR(outerRef[innerKeyStr].as<double>(), 3.0, 1e-9);

  const mrpt::containers::yaml_ref constOuterRef = outerRef;
  EXPECT_NEAR(constOuterRef[innerKeyStr].as<double>(), 3.0, 1e-9);

  mrpt::containers::yaml_cref outerCref = cp["outer"];
  EXPECT_NEAR(outerCref[innerKeyStr].as<double>(), 3.0, 1e-9);
}
MRPT_TEST_END()

MRPT_TEST(yaml, loadAndFromFileRoundTrip)
{
  using mrpt::containers::yaml;

  const std::string fileName = "test_yaml_roundtrip.yaml";
  {
    std::ofstream f(fileName);
    f << "a: 1\nb: two\n";
  }

  yaml p1;
  p1.loadFromFile(fileName);
  EXPECT_EQ(p1["a"].as<int>(), 1);
  EXPECT_EQ(p1["b"].as<std::string>(), "two");

  yaml p2 = yaml::FromFile(fileName);
  EXPECT_EQ(p2["a"].as<int>(), 1);

  std::remove(fileName.c_str());
}
MRPT_TEST_END()

MRPT_TEST(yaml, nullScalarTypeName)
{
  using mrpt::containers::yaml;

  // A scalar node holding std::monostate (a "null scalar") is also
  // considered a "null node" by isNullNode(), and typeName() reports "null"
  // for it too (same as a plain, never-assigned null node):
  yaml p = yaml::FromText("a: null\n");
  EXPECT_TRUE(p["a"].isScalar());
  EXPECT_TRUE(p["a"].isNullNode());
  EXPECT_EQ(p["a"].node().typeName(), "null");
}
MRPT_TEST_END()

#endif  // MRPT_HAS_FYAML
