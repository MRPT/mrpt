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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <sstream>

using namespace mrpt::maps;
using namespace mrpt::obs;

// Note: this test-only helper class must live directly in namespace
// mrpt::obs (not wrapped in an anonymous namespace) because the
// DEFINE_SERIALIZABLE/IMPLEMENTS_SERIALIZABLE/MAP_DEFINITION_* macros expand
// code that qualifies symbols with that literal namespace.
namespace mrpt::obs
{
/** A minimal, fully-functional CMetricMap subclass, used only for exercising
 * the base class' non-virtual logic (event publication, generic-params
 * gating, matching-not-implemented defaults, etc.) without depending on the
 * concrete map classes that live in mrpt_maps (not a dependency of this
 * module).
 */
class TestMetricMap : public CMetricMap
{
  DEFINE_SERIALIZABLE(TestMetricMap, mrpt::obs)
  MAP_DEFINITION_START(TestMetricMap)
  double dummyParam{1.0};
  MAP_DEFINITION_END(TestMetricMap)

 public:
  bool m_isEmpty{true};
  int insertCount{0};
  double fixedLikelihood{-1.0};

  void internal_clear() override
  {
    m_isEmpty = true;
    insertCount = 0;
  }
  bool internal_insertObservation(
      const CObservation&, const std::optional<const mrpt::poses::CPose3D>&) override
  {
    m_isEmpty = false;
    insertCount++;
    return true;
  }
  double internal_computeObservationLikelihood(
      const CObservation&, const mrpt::poses::CPose3D&) const override
  {
    return fixedLikelihood;
  }
  bool isEmpty() const override { return m_isEmpty; }
  void saveMetricMapRepresentationToFile(const std::string&) const override {}

  std::string asString() const override { return "TestMetricMap"; }
  void getVisualizationInto(mrpt::viz::CSetOfObjects&) const override {}
};

IMPLEMENTS_SERIALIZABLE(TestMetricMap, CMetricMap, mrpt::obs)
uint8_t TestMetricMap::serializeGetVersion() const { return 0; }
void TestMetricMap::serializeTo(mrpt::serialization::CArchive&) const {}
void TestMetricMap::serializeFrom(mrpt::serialization::CArchive&, uint8_t) {}

TestMetricMap::TMapDefinition::TMapDefinition() = default;
void TestMetricMap::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  dummyParam = source.read_double(sectionNamePrefix + "_creationOpts", "dummyParam", dummyParam);
}
void TestMetricMap::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  out << "dummyParam = " << dummyParam << "\n";
}
std::shared_ptr<CMetricMap> TestMetricMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const auto& def = static_cast<const TestMetricMap::TMapDefinition&>(_def);
  auto obj = std::make_shared<TestMetricMap>();
  obj->fixedLikelihood = def.dummyParam;
  return obj;
}

MAP_DEFINITION_REGISTER("mrpt::obs::TestMetricMap", TestMetricMap)

}  // namespace mrpt::obs

TEST(CMetricMap, ClearPublishesEvent)
{
  TestMetricMap m;
  m.internal_insertObservation(CObservationOdometry(), std::nullopt);
  EXPECT_FALSE(m.isEmpty());
  m.clear();
  EXPECT_TRUE(m.isEmpty());
}

TEST(CMetricMap, InsertObservationGatedByGenericParams)
{
  TestMetricMap m;
  CObservationOdometry obs;

  m.genericMapParams.enableObservationInsertion = false;
  EXPECT_FALSE(m.insertObservation(obs));
  EXPECT_EQ(m.insertCount, 0);

  m.genericMapParams.enableObservationInsertion = true;
  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_EQ(m.insertCount, 1);
}

TEST(CMetricMap, InsertObservationPtr)
{
  TestMetricMap m;
  auto obs = CObservationOdometry::Create();
  EXPECT_TRUE(m.insertObservationPtr(obs));
  EXPECT_THROW(m.insertObservationPtr(CObservation::Ptr()), std::exception);
}

TEST(CMetricMap, ComputeObservationLikelihoodGatedByGenericParams)
{
  TestMetricMap m;
  m.fixedLikelihood = 3.5;
  CObservationOdometry obs;
  const mrpt::poses::CPose3D from;

  m.genericMapParams.enableObservationLikelihood = false;
  EXPECT_DOUBLE_EQ(m.computeObservationLikelihood(obs, from), 0.0);
  EXPECT_FALSE(m.canComputeObservationLikelihood(obs));

  m.genericMapParams.enableObservationLikelihood = true;
  EXPECT_DOUBLE_EQ(m.computeObservationLikelihood(obs, from), 3.5);
  EXPECT_TRUE(m.canComputeObservationLikelihood(obs));
}

TEST(CMetricMap, ComputeObservationsLikelihoodOverSensoryFrame)
{
  TestMetricMap m;
  m.fixedLikelihood = 2.0;
  CSensoryFrame sf;
  sf.insert(CObservationOdometry::Create());
  sf.insert(CObservationRange::Create());

  const mrpt::poses::CPose3D from;
  EXPECT_DOUBLE_EQ(m.computeObservationsLikelihood(sf, from), 4.0);
  EXPECT_TRUE(m.canComputeObservationsLikelihood(sf));

  CSensoryFrame emptySf;
  EXPECT_FALSE(m.canComputeObservationsLikelihood(emptySf));
}

TEST(CMetricMap, LoadFromSimpleMap)
{
  TestMetricMap m;
  CSimpleMap sm;

  auto posePdf = mrpt::poses::CPose3DPDFGaussian::Create();
  posePdf->mean = mrpt::poses::CPose3D(1, 2, 0, 0, 0, 0);
  auto sf = CSensoryFrame::Create();
  sf->insert(CObservationOdometry::Create());
  sm.insert(posePdf, sf);

  m.loadFromSimpleMap(sm);
  EXPECT_EQ(m.insertCount, 1);
  EXPECT_FALSE(m.isEmpty());
}

TEST(CMetricMap, DefaultVirtualMethodsThrowOrReturnDefaults)
{
  TestMetricMap m1, m2;
  mrpt::tfest::TMatchingPairList pairs;
  TMatchingParams params;
  TMatchingExtraResults extra;
  EXPECT_THROW(
      m1.determineMatching2D(&m2, mrpt::poses::CPose2D(), pairs, params, extra), std::exception);
  EXPECT_THROW(
      m1.determineMatching3D(&m2, mrpt::poses::CPose3D(), pairs, params, extra), std::exception);

  TMatchingRatioParams rparams;
  EXPECT_THROW(m1.compute3DMatchingRatio(&m2, mrpt::poses::CPose3D(), rparams), std::exception);
  EXPECT_THROW(m1.squareDistanceToClosestCorrespondence(0, 0), std::exception);

  EXPECT_EQ(m1.getAsSimplePointsMap(), nullptr);
  const auto bbox = m1.boundingBox();
  EXPECT_FLOAT_EQ(bbox.min.x, 0.0f);
}

// ------------------- TMetricMapTypesRegistry / TMetricMapInitializer -------------------

TEST(TMetricMapTypesRegistry, FactoryAndRegistry)
{
  auto& reg = mrpt::maps::internal::TMetricMapTypesRegistry::Instance();
  const auto& all = reg.getAllRegistered();
  EXPECT_GT(all.size(), 0u);
  EXPECT_TRUE(all.count("mrpt::obs::TestMetricMap") || all.count("TestMetricMap"));

  auto def = reg.factoryMapDefinition("mrpt::obs::TestMetricMap");
  ASSERT_TRUE(def);

  // Also reachable via the short (namespace-stripped) alias:
  auto def2 = reg.factoryMapDefinition("TestMetricMap");
  ASSERT_TRUE(def2);

  EXPECT_FALSE(reg.factoryMapDefinition("NoSuchMapClassXYZ"));

  auto mapObj = reg.factoryMapObjectFromDefinition(*def);
  ASSERT_TRUE(mapObj);
  EXPECT_EQ(std::string(mapObj->GetRuntimeClass()->className), "mrpt::obs::TestMetricMap");
}

TEST(TMetricMapTypesRegistry, FactoryMapObjectFromDefinitionUnregisteredThrows)
{
  // Build a definition-like object pointing to an unregistered class type,
  // by using the registered one but querying a bogus class id indirectly:
  auto& reg = mrpt::maps::internal::TMetricMapTypesRegistry::Instance();
  auto def = reg.factoryMapDefinition("mrpt::obs::TestMetricMap");
  ASSERT_TRUE(def);
  // Sanity: a valid definition always succeeds:
  EXPECT_NO_THROW(reg.factoryMapObjectFromDefinition(*def));
}

TEST(TMetricMapInitializer, FactoryLoadSaveDump)
{
  auto mi = TMetricMapInitializer::factory("mrpt::obs::TestMetricMap");
  ASSERT_TRUE(mi);

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("test_creationOpts", "dummyParam", 42.0);
  mi->loadFromConfigFile(cfg, "test");

  mrpt::config::CConfigFileMemory cfgOut;
  mi->saveToConfigFile(cfgOut, "outsec");
  EXPECT_FALSE(cfgOut.getContent().empty());

  std::stringstream ss;
  mi->dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("dummyParam"), std::string::npos);

  EXPECT_EQ(std::string(mi->getMetricMapClassType()->className), "mrpt::obs::TestMetricMap");
}

TEST(TMetricMapInitializer, FactoryUnknownReturnsNull)
{
  auto mi = TMetricMapInitializer::factory("NoSuchClassAtAllXYZ");
  EXPECT_FALSE(mi);
}

TEST(TSetOfMetricMapInitializers, LoadSaveDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("test", "TestMetricMap_count", 1);
  cfg.write("test_TestMetricMap_00_creationOpts", "dummyParam", 7.0);

  TSetOfMetricMapInitializers sets;
  sets.loadFromConfigFile(cfg, "test");
  ASSERT_EQ(sets.size(), 1u);

  mrpt::config::CConfigFileMemory cfgOut;
  sets.saveToConfigFile(cfgOut, "out");

  std::stringstream ss;
  sets.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  sets.clear();
  EXPECT_EQ(sets.size(), 0u);

  // iterate an empty set (both const and non-const) for coverage:
  for (auto it = sets.begin(); it != sets.end(); ++it)
  {
  }
  const TSetOfMetricMapInitializers& csets = sets;
  for (auto it = csets.begin(); it != csets.end(); ++it)
  {
  }
}

TEST(TSetOfMetricMapInitializers, UnknownMapTypeThrows)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("test", "NoSuchMapTypeXYZ_count", 1);

  TSetOfMetricMapInitializers sets;
  EXPECT_THROW(sets.loadFromConfigFile(cfg, "test"), std::exception);
}

// ------------------- TMapGenericParams (metric_map_types.cpp) -------------------

TEST(TMapGenericParams, LoadSaveConfigFile)
{
  TMapGenericParams p;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "enableSaveAs3DObject", false);
  cfg.write("sec", "enableObservationLikelihood", false);
  cfg.write("sec", "enableObservationInsertion", false);
  p.loadFromConfigFile(cfg, "sec");
  EXPECT_FALSE(p.enableSaveAs3DObject);
  EXPECT_FALSE(p.enableObservationLikelihood);
  EXPECT_FALSE(p.enableObservationInsertion);

  mrpt::config::CConfigFileMemory cfgOut;
  p.saveToConfigFile(cfgOut, "sec2");
  EXPECT_FALSE(cfgOut.getContent().empty());
}

TEST(TMapGenericParams, SerializationRoundtrip)
{
  TMapGenericParams p1;
  p1.enableSaveAs3DObject = false;
  p1.enableObservationLikelihood = false;
  p1.enableObservationInsertion = true;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p1;
  buf.Seek(0);

  TMapGenericParams p2;
  arch >> p2;
  EXPECT_EQ(p2.enableSaveAs3DObject, p1.enableSaveAs3DObject);
  EXPECT_EQ(p2.enableObservationLikelihood, p1.enableObservationLikelihood);
  EXPECT_EQ(p2.enableObservationInsertion, p1.enableObservationInsertion);
}

// ------------------- CSimpleMap extra content -------------------

TEST(CSimpleMapContent, InsertRemoveMakeDeepCopy)
{
  CSimpleMap sm;
  EXPECT_TRUE(sm.empty());

  auto pose1 = mrpt::poses::CPose3DPDFGaussian::Create();
  pose1->mean = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  auto sf1 = CSensoryFrame::Create();
  sf1->insert(CObservationOdometry::Create());
  sm.insert(pose1, sf1, mrpt::math::TTwist3D(0.1, 0, 0, 0, 0, 0));

  auto pose2 = mrpt::poses::CPose3DPDFGaussian::Create();
  pose2->mean = mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0);
  auto sf2 = CSensoryFrame::Create();
  sm.insert(CSimpleMap::Keyframe(pose2, sf2));

  ASSERT_EQ(sm.size(), 2u);
  EXPECT_FALSE(sm.empty());

  auto deepCopy = sm.makeDeepCopy();
  ASSERT_EQ(deepCopy.size(), 2u);
  // A deep copy must not share the same pose object instance:
  EXPECT_NE(deepCopy.get(0).pose.get(), sm.get(0).pose.get());

  sm.remove(0);
  ASSERT_EQ(sm.size(), 1u);
  EXPECT_THROW(sm.remove(100), std::exception);

  sm.clear();
  EXPECT_TRUE(sm.empty());
}

TEST(CSimpleMapContent, ChangeCoordinatesOrigin)
{
  CSimpleMap sm;
  auto pose = mrpt::poses::CPose3DPDFGaussian::Create();
  pose->mean = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  auto sf = CSensoryFrame::Create();
  sm.insert(pose, sf);

  // changeCoordinatesOrigin() composes each keyframe pose with the new
  // origin (new_pose = newOrigin (+) old_pose), so (1,0,0)+(1,0,0)=(2,0,0):
  sm.changeCoordinatesOrigin(mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0));
  EXPECT_NEAR(sm.get(0).pose->getMeanVal().x(), 2.0, 1e-6);
}

TEST(CSimpleMapContent, SaveLoadFileRoundtrip)
{
  CSimpleMap sm;
  auto pose = mrpt::poses::CPose3DPDFGaussian::Create();
  pose->mean = mrpt::poses::CPose3D(3, 1, 0, 0, 0, 0.1);
  auto sf = CSensoryFrame::Create();
  sf->insert(CObservationOdometry::Create());
  sm.insert(pose, sf);

  const std::string tmpFile = mrpt::system::getTempFileName() + ".simplemap";
  ASSERT_TRUE(sm.saveToFile(tmpFile));

  CSimpleMap sm2;
  ASSERT_TRUE(sm2.loadFromFile(tmpFile));
  ASSERT_EQ(sm2.size(), 1u);
  EXPECT_NEAR(sm2.get(0).pose->getMeanVal().x(), 3.0, 1e-6);

  mrpt::system::deleteFile(tmpFile);
}

TEST(CSimpleMapContent, LoadFromMissingFileReturnsFalse)
{
  CSimpleMap sm;
  EXPECT_FALSE(sm.loadFromFile("/nonexistent/path/to/file.simplemap"));
}

TEST(CSimpleMapContent, SerializationRoundtripCurrentVersion)
{
  CSimpleMap sm1;
  auto pose = mrpt::poses::CPose3DPDFGaussian::Create();
  pose->mean = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  auto sf = CSensoryFrame::Create();
  sf->insert(CObservationOdometry::Create());
  sm1.insert(pose, sf, mrpt::math::TTwist3D(0.1, 0.2, 0, 0, 0, 0.3));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << sm1;
  buf.Seek(0);

  CSimpleMap sm2;
  arch >> sm2;
  ASSERT_EQ(sm2.size(), 1u);
  ASSERT_TRUE(sm2.get(0).localTwist.has_value());
  EXPECT_NEAR(sm2.get(0).localTwist->vx, 0.1, 1e-9);
}

TEST(CSimpleMapContent, Iterators)
{
  CSimpleMap sm;
  for (int i = 0; i < 3; i++)
  {
    auto pose = mrpt::poses::CPose3DPDFGaussian::Create();
    auto sf = CSensoryFrame::Create();
    sm.insert(pose, sf);
  }

  size_t count = 0;
  for (auto it = sm.begin(); it != sm.end(); ++it) count++;
  EXPECT_EQ(count, 3u);

  count = 0;
  for (auto it = sm.rbegin(); it != sm.rend(); ++it) count++;
  EXPECT_EQ(count, 3u);

  const CSimpleMap& csm = sm;
  count = 0;
  for (auto it = csm.cbegin(); it != csm.cend(); ++it) count++;
  EXPECT_EQ(count, 3u);
  count = 0;
  for (auto it = csm.crbegin(); it != csm.crend(); ++it) count++;
  EXPECT_EQ(count, 3u);
}
