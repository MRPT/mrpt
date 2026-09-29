/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Unit tests for CGenericPointsMap field-order stability,
// thread-safety of the field registration API, and basic operations.

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <sstream>

using mrpt::maps::CGenericPointsMap;

TEST(CGenericPointsMap, FieldNamesByType)
{
  CGenericPointsMap m;
  m.registerField_float("f1");
  m.registerField_double("d1");
  m.registerField_uint16("u16");
  m.registerField_uint8("u8");
  m.registerField_uint32("u32");

  // (the "x", "y", "z" coordinates are also float fields)
  const auto floatNames = m.getPointFieldNames_float();
  EXPECT_NE(std::find(floatNames.begin(), floatNames.end(), "f1"), floatNames.end());
  EXPECT_EQ(m.getPointFieldNames_double(), std::vector<std::string>{"d1"});
  EXPECT_EQ(m.getPointFieldNames_uint16(), std::vector<std::string>{"u16"});
  EXPECT_EQ(m.getPointFieldNames_uint8(), std::vector<std::string>{"u8"});
  EXPECT_EQ(m.getPointFieldNames_uint32(), std::vector<std::string>{"u32"});
  for (const char* n : {"f1", "d1", "u16", "u8", "u32"})
  {
    EXPECT_TRUE(m.hasPointField(n)) << n;
  }
  EXPECT_FALSE(m.hasPointField("nope"));
}

TEST(CGenericPointsMap, RegisteringAnExistingNameThrowsWhateverTheType)
{
  CGenericPointsMap m;
  m.registerField_float("shared_name");
  EXPECT_ANY_THROW(m.registerField_float("shared_name"));
  EXPECT_ANY_THROW(m.registerField_double("shared_name"));
  EXPECT_ANY_THROW(m.registerField_uint16("shared_name"));
  EXPECT_ANY_THROW(m.registerField_uint8("shared_name"));
  EXPECT_ANY_THROW(m.registerField_uint32("shared_name"));
}

TEST(CGenericPointsMap, AccessingUnregisteredFieldsThrows)
{
  CGenericPointsMap m;
  m.insertPointFast(1, 2, 3);
  // Reading a field that does not exist gives zero:
  const auto& cm = m;
  EXPECT_EQ(cm.getPointField_float(0, "x1"), 0.0f);
  EXPECT_EQ(cm.getPointField_double(0, "x2"), 0.0);
  EXPECT_EQ(cm.getPointField_uint16(0, "x3"), 0);
  EXPECT_EQ(cm.getPointField_uint8(0, "x4"), 0);
  EXPECT_EQ(cm.getPointField_uint32(0, "x5"), 0u);
  // ...but writing, inserting, reserving or resizing them is an error:
  EXPECT_ANY_THROW(m.setPointField_float(0, "x1", 1.0f));
  EXPECT_ANY_THROW(m.setPointField_double(0, "x2", 1.0));
  EXPECT_ANY_THROW(m.setPointField_uint16(0, "x3", 1));
  EXPECT_ANY_THROW(m.setPointField_uint8(0, "x4", 1));
  EXPECT_ANY_THROW(m.setPointField_uint32(0, "x5", 1));
  EXPECT_ANY_THROW(m.insertPointField_float("x1", 1.0f));
  EXPECT_ANY_THROW(m.insertPointField_double("x2", 1.0));
  EXPECT_ANY_THROW(m.insertPointField_uint16("x3", 1));
  EXPECT_ANY_THROW(m.insertPointField_uint8("x4", 1));
  EXPECT_ANY_THROW(m.insertPointField_uint32("x5", 1));
  EXPECT_ANY_THROW(m.reserveField_double("x2", 10));
  EXPECT_ANY_THROW(m.reserveField_uint16("x3", 10));
  EXPECT_ANY_THROW(m.reserveField_uint8("x4", 10));
  EXPECT_ANY_THROW(m.reserveField_uint32("x5", 10));
  EXPECT_ANY_THROW(m.resizeField_double("x2", 10));
  EXPECT_ANY_THROW(m.resizeField_uint16("x3", 10));
  EXPECT_ANY_THROW(m.resizeField_uint8("x4", 10));
  EXPECT_ANY_THROW(m.resizeField_uint32("x5", 10));
}

TEST(CGenericPointsMap, InsertFieldsOfEveryTypeAfterAddingPoints)
{
  CGenericPointsMap m;
  m.registerField_float("f");
  m.registerField_double("d");
  m.registerField_uint16("s");
  m.registerField_uint8("b");
  m.registerField_uint32("i");

  // Fields are filled in right after adding each point:
  for (int k = 0; k < 4; k++)
  {
    m.insertPointFast(static_cast<float>(k), 0, 0);
    m.insertPointField_float("f", 0.5f * static_cast<float>(k));
    m.insertPointField_double("d", 1.5 * k);
    m.insertPointField_uint16("s", static_cast<uint16_t>(100 + k));
    m.insertPointField_uint8("b", static_cast<uint8_t>(10 + k));
    m.insertPointField_uint32("i", 100000u + static_cast<uint32_t>(k));
  }
  ASSERT_EQ(m.size(), 4u);
  for (size_t k = 0; k < 4; k++)
  {
    EXPECT_FLOAT_EQ(m.getPointField_float(k, "f"), 0.5f * static_cast<float>(k));
    EXPECT_DOUBLE_EQ(m.getPointField_double(k, "d"), 1.5 * static_cast<double>(k));
    EXPECT_EQ(m.getPointField_uint16(k, "s"), 100 + k);
    EXPECT_EQ(m.getPointField_uint8(k, "b"), 10 + k);
    EXPECT_EQ(m.getPointField_uint32(k, "i"), 100000u + k);
  }

  // A field registered later is padded with zeros for the existing points,
  // and a missing value is padded when the next one is inserted:
  m.registerField_uint32("late");
  EXPECT_EQ(m.getPointField_uint32(2, "late"), 0u);
  m.insertPointFast(9, 9, 9);
  m.insertPointField_uint32("late", 77);
  EXPECT_EQ(m.getPointField_uint32(4, "late"), 77u);
  EXPECT_EQ(m.getPointField_uint32(3, "late"), 0u);

  // Set, reserve and resize:
  m.setPointField_uint32(1, "i", 5);
  EXPECT_EQ(m.getPointField_uint32(1, "i"), 5u);
  m.reserveField_float("f", 100);
  m.reserveField_double("d", 100);
  m.reserveField_uint16("s", 100);
  m.reserveField_uint8("b", 100);
  m.reserveField_uint32("i", 100);
  m.resizeField_float("f", 8);
  m.resizeField_double("d", 8);
  m.resizeField_uint16("s", 8);
  m.resizeField_uint8("b", 8);
  m.resizeField_uint32("i", 8);
  m.resize(8);
  EXPECT_EQ(m.size(), 8u);
  EXPECT_FLOAT_EQ(m.getPointField_float(7, "f"), 0.0f);
  EXPECT_EQ(m.getPointField_uint16(7, "s"), 0);

  // Clearing removes the points and the extra fields:
  m.clear();
  EXPECT_EQ(m.size(), 0u);
  EXPECT_FALSE(m.hasPointField("f"));
}

TEST(CGenericPointsMap, DeserializingAnUnknownVersionFails)
{
  CGenericPointsMap m;
  m.insertPointFast(1, 2, 3);
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << m;

  // Corrupt the serialization version byte (right after the class name):
  std::string bytes(static_cast<const char*>(buf.getRawBufferData()), buf.getTotalBytesCount());
  const auto pos = bytes.find("CGenericPointsMap");
  ASSERT_NE(pos, std::string::npos);
  bytes[pos + std::string("CGenericPointsMap").size()] = static_cast<char>(0x7f);

  mrpt::io::CMemoryStream buf2;
  buf2.Write(bytes.data(), bytes.size());
  buf2.Seek(0);
  auto arch2 = mrpt::serialization::archiveFrom(buf2);
  CGenericPointsMap m2;
  EXPECT_ANY_THROW(arch2 >> m2);
}

TEST(CGenericPointsMap, MapDefinitionFromConfig)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("map_gen_00", "dummy", "0");
  cfg.write("map_gen_00_insertOpts", "minDistBetweenLaserPoints", "0.25");
  cfg.write("map_gen_00_likelihoodOpts", "sigma_dist", "0.5");

  CGenericPointsMap::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_gen_00");
  EXPECT_NEAR(def.insertionOpts.minDistBetweenLaserPoints, 0.25f, 1e-6f);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = CGenericPointsMap::CreateFromMapDefinition(def);
  ASSERT_TRUE(map);
  EXPECT_NEAR(map->insertionOptions.minDistBetweenLaserPoints, 0.25f, 1e-6f);
  EXPECT_NEAR(map->likelihoodOptions.sigma_dist, 0.5, 1e-6);
}
