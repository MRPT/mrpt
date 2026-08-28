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

/** Coverage for the whole family of PTG classes: the ones not exercised by
 *  `PTGs_unittest.cpp` (which only instantiates a subset from a config file),
 *  plus the shared services of the CParameterizedTrajectoryGenerator base:
 *  config file and stream (de)serialization, TP-space index mapping,
 *  visualization, clearance diagrams and the robot-shape mixins.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CC.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CS.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h>
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfLines.h>

#include <fstream>
#include <memory>

using namespace mrpt::nav;

namespace
{
/** Config with the keys shared by every CPTG_DiffDrive_CollisionGridBased
 *  derived class. A small refDistance/num_paths keeps the collision grid
 *  build time low. */
void fill_diffdrive_cfg(mrpt::config::CConfigFileMemory& cfg, const std::string& s)
{
  cfg.write(s, "num_paths", 21);
  cfg.write(s, "refDistance", 2.0);
  cfg.write(s, "resolution", 0.25);
  cfg.write(s, "v_max_mps", 1.0);
  cfg.write(s, "w_max_dps", 60.0);
  cfg.write(s, "K", 1.0);
  cfg.write(s, "cte_a0v_deg", 57.0);
  cfg.write(s, "cte_a0w_deg", 57.0);
  // A small square robot shape:
  cfg.write(s, "shape_x0", -0.2);
  cfg.write(s, "shape_y0", 0.2);
  cfg.write(s, "shape_x1", 0.2);
  cfg.write(s, "shape_y1", 0.2);
  cfg.write(s, "shape_x2", 0.2);
  cfg.write(s, "shape_y2", -0.2);
  cfg.write(s, "shape_x3", -0.2);
  cfg.write(s, "shape_y3", -0.2);
}

void fill_holo_cfg(mrpt::config::CConfigFileMemory& cfg, const std::string& s)
{
  cfg.write(s, "num_paths", 21);
  cfg.write(s, "refDistance", 3.0);
  cfg.write(s, "T_ramp_max", 0.8);
  cfg.write(s, "v_max_mps", 1.0);
  cfg.write(s, "w_max_dps", 60.0);
  cfg.write(s, "robot_radius", 0.35);
}

/** All the collision-grid-based PTG class names, i.e. those whose paths are
 *  precomputed into a look-up table. */
const char* const diffdrive_ptg_names[] = {
    "CPTG_DiffDrive_C", "CPTG_DiffDrive_alpha", "CPTG_DiffDrive_CC", "CPTG_DiffDrive_CCS",
    "CPTG_DiffDrive_CS"};

CParameterizedTrajectoryGenerator::Ptr make_diffdrive_ptg(const std::string& className)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_diffdrive_cfg(cfg, "PTG");
  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG(className, cfg, "PTG", "");
  ptg->initialize(std::string(), false /*verbose*/);
  return ptg;
}
}  // namespace

// ---------------------------------------------------------------------------
//  Every PTG flavor: factory + initialize + basic invariants
// ---------------------------------------------------------------------------
TEST(PTGVariants, all_diffdrive_flavors_initialize)
{
  for (const auto* name : diffdrive_ptg_names)
  {
    auto ptg = make_diffdrive_ptg(name);
    ASSERT_TRUE(ptg) << name;
    EXPECT_TRUE(ptg->isInitialized()) << name;
    EXPECT_EQ(ptg->getPathCount(), 21U) << name;
    EXPECT_EQ(ptg->getAlphaValuesCount(), ptg->getPathCount()) << name;
    EXPECT_NEAR(ptg->getRefDistance(), 2.0, 1e-9) << name;
    EXPECT_FALSE(ptg->getDescription().empty()) << name;
    EXPECT_GT(ptg->getMaxRobotRadius(), .0) << name;
    EXPECT_GT(ptg->getMaxLinVel(), .0) << name;
    EXPECT_GT(ptg->getMaxAngVel(), .0) << name;

    // Every path must have at least one step, and step<->distance must be
    // monotonically consistent:
    for (uint16_t k = 0; k < ptg->getPathCount(); k++)
    {
      const size_t nSteps = ptg->getPathStepCount(k);
      ASSERT_GT(nSteps, 0U) << name << " k=" << k;
      EXPECT_GE(ptg->getPathDist(k, 0), .0) << name;
      EXPECT_GE(ptg->getPathDist(k, static_cast<uint32_t>(nSteps - 1)), ptg->getPathDist(k, 0))
          << name;
    }

    ptg->deinitialize();
    EXPECT_FALSE(ptg->isInitialized()) << name;
  }
}

TEST(PTGVariants, diffdrive_config_file_roundtrip)
{
  for (const auto* name : diffdrive_ptg_names)
  {
    auto ptg = make_diffdrive_ptg(name);

    mrpt::config::CConfigFileMemory cfg2;
    ptg->saveToConfigFile(cfg2, "OUT");

    auto ptg2 = CParameterizedTrajectoryGenerator::CreatePTG(name, cfg2, "OUT", "");
    ASSERT_TRUE(ptg2) << name;
    EXPECT_EQ(ptg2->getPathCount(), ptg->getPathCount()) << name;
    EXPECT_NEAR(ptg2->getRefDistance(), ptg->getRefDistance(), 1e-9) << name;
    EXPECT_EQ(ptg2->getDescription(), ptg->getDescription()) << name;
  }
}

TEST(PTGVariants, diffdrive_serialization_roundtrip)
{
  for (const auto* name : diffdrive_ptg_names)
  {
    auto ptg = make_diffdrive_ptg(name);

    mrpt::io::CMemoryStream buf;
    auto arch = mrpt::serialization::archiveFrom(buf);
    arch << *ptg;
    buf.Seek(0);

    mrpt::serialization::CSerializable::Ptr obj;
    arch >> obj;
    auto ptg2 = std::dynamic_pointer_cast<CParameterizedTrajectoryGenerator>(obj);
    ASSERT_TRUE(ptg2) << name;
    EXPECT_EQ(ptg2->getPathCount(), ptg->getPathCount()) << name;
    EXPECT_NEAR(ptg2->getRefDistance(), ptg->getRefDistance(), 1e-9) << name;
    EXPECT_EQ(ptg2->getDescription(), ptg->getDescription()) << name;
    // Deserialization leaves the PTG un-initialized (no LUT yet):
    EXPECT_FALSE(ptg2->isInitialized()) << name;
  }
}

TEST(PTGVariants, diffdrive_default_params_are_usable)
{
  for (const auto* name : diffdrive_ptg_names)
  {
    mrpt::config::CConfigFileMemory cfg;
    fill_diffdrive_cfg(cfg, "PTG");
    auto ptg = CParameterizedTrajectoryGenerator::CreatePTG(name, cfg, "PTG", "");
    ASSERT_TRUE(ptg);

    ptg->loadDefaultParams();
    EXPECT_GT(ptg->getPathCount(), 0U) << name;
    EXPECT_GT(ptg->getRefDistance(), .0) << name;
  }
}

TEST(PTGVariants, diffdrive_render_path_and_shape)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  mrpt::viz::CSetOfLines gl;
  ptg->renderPathAsSimpleLine(0, gl, 0.10 /*decimate*/, -1.0 /*whole path*/);
  EXPECT_GT(gl.size(), 0U);

  // Truncating the path yields no more segments than the full path:
  mrpt::viz::CSetOfLines gl2;
  ptg->renderPathAsSimpleLine(0, gl2, 0.10, 0.5 /*max_path_distance*/);
  EXPECT_LE(gl2.size(), gl.size());

  mrpt::viz::CSetOfLines glShape;
  ptg->add_robotShape_to_setOfLines(glShape, mrpt::poses::CPose2D(1, 2, 0.5));
  EXPECT_GT(glShape.size(), 0U);
}

TEST(PTGVariants, diffdrive_tp_obstacles_and_inverse_map)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  std::vector<double> tp_obs;
  ptg->initTPObstacles(tp_obs);
  ASSERT_EQ(tp_obs.size(), ptg->getPathCount());
  for (const double d : tp_obs)
  {
    EXPECT_NEAR(d, ptg->getRefDistance(), 1e-9);
  }

  double tp_obs_k = .0;
  ptg->initTPObstacleSingle(0, tp_obs_k);
  EXPECT_NEAR(tp_obs_k, ptg->getRefDistance(), 1e-9);

  // An obstacle right ahead must shorten at least one TP-obstacle entry:
  const auto tp_obs_org = tp_obs;
  ptg->updateTPObstacle(1.0, .0, tp_obs);
  EXPECT_NE(tp_obs, tp_obs_org);

  // A point reachable by the PTG must be invertible into (k,d):
  const auto inv = ptg->inverseMap_WS2TP(1.0, 0.05);
  EXPECT_TRUE(inv.has_value());
  EXPECT_TRUE(ptg->PTG_IsIntoDomain(1.0, 0.05));
}

TEST(PTGVariants, diffdrive_vel_cmds)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  auto emptyCmd = ptg->getSupportedKinematicVelocityCommand();
  ASSERT_TRUE(emptyCmd);
  EXPECT_EQ(emptyCmd->getVelCmdLength(), 2U);

  auto cmd = ptg->directionToMotionCommand(ptg->getPathCount() / 2);
  ASSERT_TRUE(cmd);
  EXPECT_EQ(cmd->getVelCmdLength(), emptyCmd->getVelCmdLength());
}

// ---------------------------------------------------------------------------
//  Base class services
// ---------------------------------------------------------------------------
TEST(PTGBase, alpha_index_roundtrip)
{
  const unsigned int N = 64;
  for (uint16_t k = 0; k < N; k++)
  {
    const double a = CParameterizedTrajectoryGenerator::Index2alpha(k, N);
    EXPECT_GE(a, -M_PI);
    EXPECT_LE(a, M_PI);
    EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(a, N), k);
  }
  EXPECT_ANY_THROW(CParameterizedTrajectoryGenerator::Index2alpha(N, N));
}

TEST(PTGBase, alpha2index_covers_the_whole_circle)
{
  const unsigned int N = 32;
  EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(-M_PI, N), 0);
  EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(M_PI - 1e-9, N), N - 1);
  EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(.0, N), N / 2);
}

TEST(PTGBase, alpha2index_wraps_angles_outside_the_pi_range)
{
  const unsigned int N = 32;
  const double a = 0.25 * M_PI;
  const uint16_t k = CParameterizedTrajectoryGenerator::Alpha2index(a, N);
  // Adding a full turn must not change the resulting path index:
  EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(a + 2 * M_PI, N), k);
  EXPECT_EQ(CParameterizedTrajectoryGenerator::Alpha2index(a - 2 * M_PI, N), k);
}

TEST(PTGBase, instance_alpha_index_helpers_use_the_path_count)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");
  const uint16_t k = 7;
  const double a = ptg->index2alpha(k);
  EXPECT_EQ(ptg->alpha2index(a), k);
}

TEST(PTGBase, collision_behavior_is_settable)
{
  const auto saved = CParameterizedTrajectoryGenerator::getCollisionBehavior();

  CParameterizedTrajectoryGenerator::setCollisionBehavior(PTGCollisionBehavior::STOP);
  EXPECT_EQ(CParameterizedTrajectoryGenerator::getCollisionBehavior(), PTGCollisionBehavior::STOP);

  CParameterizedTrajectoryGenerator::setCollisionBehavior(PTGCollisionBehavior::BACK_AWAY);
  EXPECT_EQ(
      CParameterizedTrajectoryGenerator::getCollisionBehavior(), PTGCollisionBehavior::BACK_AWAY);

  CParameterizedTrajectoryGenerator::setCollisionBehavior(saved);
}

TEST(PTGBase, output_debug_path_prefix_is_settable)
{
  const auto saved = CParameterizedTrajectoryGenerator::getOutputDebugPathPrefix();
  CParameterizedTrajectoryGenerator::setOutputDebugPathPrefix("./some_dir");
  EXPECT_EQ(CParameterizedTrajectoryGenerator::getOutputDebugPathPrefix(), "./some_dir");
  CParameterizedTrajectoryGenerator::setOutputDebugPathPrefix(saved);
}

TEST(PTGBase, debugDumpInFiles_writes_the_trajectory_tables)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  const std::string sDir = mrpt::system::getTempFileName() + std::string("_ptg_dump");
  const auto saved = CParameterizedTrajectoryGenerator::getOutputDebugPathPrefix();
  CParameterizedTrajectoryGenerator::setOutputDebugPathPrefix(sDir);

  EXPECT_TRUE(ptg->debugDumpInFiles("UnitTest"));
  EXPECT_TRUE(mrpt::system::fileExists(sDir + "/PTGs/PTGUnitTest_x.txt"));
  EXPECT_TRUE(mrpt::system::fileExists(sDir + "/PTGs/PTGUnitTest_y.txt"));
  EXPECT_TRUE(mrpt::system::fileExists(sDir + "/PTGs/PTGUnitTest_phi.txt"));
  EXPECT_TRUE(mrpt::system::fileExists(sDir + "/PTGs/PTGUnitTest_d.txt"));

  CParameterizedTrajectoryGenerator::setOutputDebugPathPrefix(saved);
  mrpt::system::deleteFilesInDirectory(sDir + "/PTGs", true);
}

TEST(PTGBase, nav_dynamic_state_comparison_and_stream)
{
  CParameterizedTrajectoryGenerator::TNavDynamicState a, b;
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);

  b.curVelLocal = mrpt::math::TTwist2D(1.0, .0, .0);
  EXPECT_TRUE(a != b);

  b.relTarget = mrpt::math::TPose2D(1, 2, 0.3);
  b.targetRelSpeed = 0.5;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  b.writeToStream(arch);
  buf.Seek(0);

  CParameterizedTrajectoryGenerator::TNavDynamicState c;
  c.readFromStream(arch);
  EXPECT_TRUE(c == b);
}

TEST(PTGBase, updateNavDynamicState_is_forwarded_to_the_ptg)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  CParameterizedTrajectoryGenerator::TNavDynamicState st;
  st.curVelLocal = mrpt::math::TTwist2D(0.5, .0, .0);
  st.relTarget = mrpt::math::TPose2D(2, 0, 0);
  st.targetRelSpeed = 0.0;

  ptg->updateNavDynamicState(st);
  EXPECT_TRUE(ptg->getCurrentNavDynamicState() == st);

  // Re-applying the same state is a no-op, and `force_update` re-applies it:
  ptg->updateNavDynamicState(st);
  ptg->updateNavDynamicState(st, true /*force_update*/);
  EXPECT_TRUE(ptg->getCurrentNavDynamicState() == st);
}

TEST(PTGBase, score_priority_and_clearance_resolution_setters)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  ptg->setScorePriorty(0.25);
  EXPECT_NEAR(ptg->getScorePriority(), 0.25, 1e-12);

  ptg->setClearanceStepCount(7);
  EXPECT_EQ(ptg->getClearanceStepCount(), 7U);
  ptg->setClearanceDecimatedPaths(5);
  EXPECT_EQ(ptg->getClearanceDecimatedPaths(), 5U);
}

TEST(PTGBase, setRefDistance_is_rejected_by_grid_based_ptgs_once_initialized)
{
  // The collision grid is built for a fixed reference distance:
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");
  EXPECT_ANY_THROW(ptg->setRefDistance(5.0));

  mrpt::config::CConfigFileMemory cfg;
  fill_holo_cfg(cfg, "PTG");
  auto holo = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg, "PTG", "");
  holo->setRefDistance(5.0);
  EXPECT_NEAR(holo->getRefDistance(), 5.0, 1e-9);
}

TEST(PTGBase, clearance_diagram_is_filled_from_obstacles)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  ClearanceDiagram cd;
  ptg->initClearanceDiagram(cd);
  EXPECT_FALSE(cd.empty());
  EXPECT_EQ(cd.get_actual_num_paths(), ptg->getPathCount());

  ptg->updateClearance(2.0, 1.0, cd);
  ptg->updateClearance(-2.0, -1.0, cd);

  std::vector<double> tp_obs;
  ptg->initTPObstacles(tp_obs);
  ptg->updateClearancePost(cd, tp_obs);

  // Clearance values must be finite and non-negative:
  for (uint16_t k = 0; k < ptg->getPathCount(); k++)
  {
    const double c = cd.getClearance(k, 0.5, false /*integrate_over_path*/);
    EXPECT_TRUE(std::isfinite(c)) << "k=" << k;
    EXPECT_GE(c, .0) << "k=" << k;
  }
}

TEST(PTGBase, getPathTwist_is_consistent_with_the_path)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  const uint16_t k = ptg->getPathCount() / 2;
  ASSERT_GT(ptg->getPathStepCount(k), 2U);

  const auto tw = ptg->getPathTwist(k, 1);
  EXPECT_TRUE(std::isfinite(tw.vx));
  EXPECT_TRUE(std::isfinite(tw.vy));
  EXPECT_TRUE(std::isfinite(tw.omega));
}

TEST(PTGBase, factory_accepts_legacy_numeric_names)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_diffdrive_cfg(cfg, "PTG");

  // MRPT <1.5.0 named the PTG classes by a single digit:
  const std::pair<const char*, const char*> legacy[] = {
      {"1",     "CPTG_DiffDrive_C"},
      {"2", "CPTG_DiffDrive_alpha"},
      {"3",   "CPTG_DiffDrive_CCS"},
      {"4",    "CPTG_DiffDrive_CC"},
      {"5",    "CPTG_DiffDrive_CS"}
  };

  for (const auto& [digit, className] : legacy)
  {
    auto ptg = CParameterizedTrajectoryGenerator::CreatePTG(digit, cfg, "PTG", "");
    ASSERT_TRUE(ptg) << digit;
    auto ref = CParameterizedTrajectoryGenerator::CreatePTG(className, cfg, "PTG", "");
    EXPECT_EQ(ptg->GetRuntimeClass(), ref->GetRuntimeClass()) << digit;
  }
}

TEST(PTGBase, factory_rejects_unknown_class_names)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_diffdrive_cfg(cfg, "PTG");
  EXPECT_ANY_THROW(
      CParameterizedTrajectoryGenerator::CreatePTG("NotAPTGClassName", cfg, "PTG", ""));
  // A registered class that is not a PTG:
  EXPECT_ANY_THROW(
      CParameterizedTrajectoryGenerator::CreatePTG("mrpt::poses::CPose3D", cfg, "PTG", ""));
}

TEST(PTGBase, factory_honors_the_key_prefix)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_diffdrive_cfg(cfg, "S");

  // With the same section but a prefix that matches no key, loading fails:
  EXPECT_ANY_THROW(
      CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", cfg, "S", "PTG0_"));
}

// ---------------------------------------------------------------------------
//  Robot shape mixins
// ---------------------------------------------------------------------------
TEST(PTGRobotShape, polygonal_shape_queries)
{
  auto ptg = std::make_shared<CPTG_DiffDrive_C>();

  mrpt::math::CPolygon poly;
  poly.add_vertex(-0.5, 0.5);
  poly.add_vertex(0.5, 0.5);
  poly.add_vertex(0.5, -0.5);
  poly.add_vertex(-0.5, -0.5);
  ptg->setRobotShape(poly);

  EXPECT_EQ(ptg->getRobotShape().size(), 4U);
  EXPECT_NEAR(ptg->getMaxRobotRadius(), std::sqrt(0.5), 1e-9);

  EXPECT_TRUE(ptg->isPointInsideRobotShape(.0, .0));
  EXPECT_FALSE(ptg->isPointInsideRobotShape(10.0, .0));

  // Inside the shape there is no clearance at all:
  EXPECT_NEAR(ptg->evalClearanceToRobotShape(.0, .0), .0, 1e-12);
  // Far away, clearance grows with the distance:
  EXPECT_GT(ptg->evalClearanceToRobotShape(10.0, .0), ptg->evalClearanceToRobotShape(5.0, .0));
  // Just outside the shape a minimum "fake" clearance is enforced:
  EXPECT_GT(ptg->evalClearanceToRobotShape(0.55, .0), .0);

  // Degenerate shapes are rejected:
  mrpt::math::CPolygon tooFew;
  tooFew.add_vertex(0, 0);
  tooFew.add_vertex(1, 0);
  EXPECT_ANY_THROW(ptg->setRobotShape(tooFew));
}

TEST(PTGRobotShape, circular_shape_queries)
{
  auto ptg = std::make_shared<CPTG_Holo_Blend>();
  ptg->setRobotShapeRadius(0.5);
  EXPECT_NEAR(ptg->getRobotShapeRadius(), 0.5, 1e-12);
  EXPECT_NEAR(ptg->getMaxRobotRadius(), 0.5, 1e-12);

  EXPECT_TRUE(ptg->isPointInsideRobotShape(0.1, .0));
  EXPECT_FALSE(ptg->isPointInsideRobotShape(1.0, .0));

  EXPECT_NEAR(ptg->evalClearanceToRobotShape(1.5, .0), 1.0, 1e-9);

  mrpt::viz::CSetOfLines gl;
  ptg->add_robotShape_to_setOfLines(gl, mrpt::poses::CPose2D(0, 0, 0));
  EXPECT_GT(gl.size(), 0U);
}

// ---------------------------------------------------------------------------
//  CPTG_Holo_Blend specifics
// ---------------------------------------------------------------------------
TEST(PTGHoloBlend, initialize_and_basic_queries)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_holo_cfg(cfg, "PTG");

  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg, "PTG", "");
  ASSERT_TRUE(ptg);
  ptg->initialize(std::string(), false);

  EXPECT_TRUE(ptg->isInitialized());
  EXPECT_EQ(ptg->getPathCount(), 21U);
  EXPECT_TRUE(ptg->supportVelCmdNOP());
  EXPECT_GT(ptg->getPathStepDuration(), .0);
  EXPECT_GT(ptg->maxTimeInVelCmdNOP(0), .0);

  auto cmd = ptg->getSupportedKinematicVelocityCommand();
  ASSERT_TRUE(cmd);
  EXPECT_EQ(cmd->getVelCmdLength(), 4U);
}

TEST(PTGHoloBlend, config_file_and_serialization_roundtrip)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_holo_cfg(cfg, "PTG");
  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg, "PTG", "");
  ptg->initialize(std::string(), false);

  mrpt::config::CConfigFileMemory cfg2;
  ptg->saveToConfigFile(cfg2, "OUT");
  auto ptgB = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg2, "OUT", "");
  ASSERT_TRUE(ptgB);
  EXPECT_EQ(ptgB->getDescription(), ptg->getDescription());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *ptg;
  buf.Seek(0);
  mrpt::serialization::CSerializable::Ptr obj;
  arch >> obj;
  auto ptgC = std::dynamic_pointer_cast<CParameterizedTrajectoryGenerator>(obj);
  ASSERT_TRUE(ptgC);
  EXPECT_EQ(ptgC->getPathCount(), ptg->getPathCount());
  EXPECT_EQ(ptgC->getDescription(), ptg->getDescription());
}

TEST(PTGHoloBlend, relative_priority_depends_on_target_speed)
{
  mrpt::config::CConfigFileMemory cfg;
  fill_holo_cfg(cfg, "PTG");
  auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg, "PTG", "");
  ptg->initialize(std::string(), false);

  const auto prio = ptg->evalPathRelativePriority(0, 1.0);
  EXPECT_TRUE(std::isfinite(prio));
  EXPECT_GT(prio, .0);
}

// ---------------------------------------------------------------------------
//  Collision-grid cache files and per-path queries
// ---------------------------------------------------------------------------
TEST(PTGVariants, collision_grid_is_cached_to_and_reloaded_from_a_file)
{
  const std::string cacheFil = mrpt::system::getTempFileName() + std::string("_ptg_cache.dat.gz");

  mrpt::config::CConfigFileMemory cfg;
  fill_diffdrive_cfg(cfg, "PTG");

  // 1st run: the grid is built from scratch and dumped to the cache file.
  auto ptg1 = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", cfg, "PTG", "");
  ptg1->initialize(cacheFil, false /*verbose*/);
  ASSERT_TRUE(mrpt::system::fileExists(cacheFil));

  // 2nd run: the very same grid is now read back from the cache.
  auto ptg2 = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", cfg, "PTG", "");
  ptg2->initialize(cacheFil, false);
  EXPECT_TRUE(ptg2->isInitialized());

  std::vector<double> obs1, obs2;
  ptg1->initTPObstacles(obs1);
  ptg2->initTPObstacles(obs2);
  ptg1->updateTPObstacle(1.0, 0.3, obs1);
  ptg2->updateTPObstacle(1.0, 0.3, obs2);
  EXPECT_EQ(obs1, obs2);

  // A corrupt/foreign cache file is detected and the grid rebuilt instead:
  {
    std::ofstream f(cacheFil, std::ios::binary | std::ios::trunc);
    f << "not a collision grid";
  }
  auto ptg3 = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_DiffDrive_C", cfg, "PTG", "");
  EXPECT_NO_THROW(ptg3->initialize(cacheFil, false));
  EXPECT_TRUE(ptg3->isInitialized());

  mrpt::system::deleteFile(cacheFil);
}

TEST(PTGVariants, per_path_tp_obstacle_updates_match_the_full_update)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");

  std::vector<double> all;
  ptg->initTPObstacles(all);
  ptg->updateTPObstacle(1.0, 0.2, all);

  for (uint16_t k = 0; k < ptg->getPathCount(); k++)
  {
    double single = .0;
    ptg->initTPObstacleSingle(k, single);
    ptg->updateTPObstacleSingle(1.0, 0.2, k, single);
    EXPECT_NEAR(single, all[k], 1e-9) << "k=" << k;
  }
}

TEST(PTGVariants, points_beyond_the_reference_distance_map_outside_the_unit_range)
{
  for (const auto* name : diffdrive_ptg_names)
  {
    auto ptg = make_diffdrive_ptg(name);
    // These PTGs answer with the nearest precomputed path, so the way to tell
    // an unreachable point apart is its normalized distance being >1:
    const double farAway = 10 * ptg->getRefDistance();
    if (const auto inv = ptg->inverseMap_WS2TP(farAway, farAway))
    {
      EXPECT_GT(inv->second, 1.0) << name;
    }
  }
}

TEST(PTGVariants, getPathStepForDist_beyond_the_path_end_fails)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");
  uint32_t step = 0;
  EXPECT_FALSE(ptg->getPathStepForDist(0, 1e6 /*way beyond refDistance*/, step));
}

TEST(PTGVariants, deinitialized_ptgs_reject_path_queries)
{
  auto ptg = make_diffdrive_ptg("CPTG_DiffDrive_C");
  ptg->deinitialize();
  EXPECT_ANY_THROW(ptg->getPathStepCount(0));
}
