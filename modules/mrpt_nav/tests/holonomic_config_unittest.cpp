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

/** Configuration, factory, serialization and log-record coverage for the
 *  holonomic navigation methods, complementing the behavioral tests in
 *  holonomic_unittest.cpp.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/nav/holonomic/CHolonomicFullEval.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::nav;
using CHRM = CAbstractHolonomicReactiveMethod;

namespace
{
CHRM::NavInput make_nav_input(size_t nDirs, double targetX, double targetY)
{
  CHRM::NavInput ni;
  ni.obstacles.assign(nDirs, 1.0);
  ni.maxObstacleDist = 1.0;
  ni.maxRobotSpeed = 1.0;
  ni.targets.emplace_back(targetX, targetY, .0);
  return ni;
}

/** CHolonomicFullEval requires a clearance diagram; the other methods ignore
 *  it. Kept alive by the caller for the duration of navigate(). */
ClearanceDiagram make_clearance(size_t nDirs)
{
  ClearanceDiagram cd;
  cd.resize(nDirs, std::min<size_t>(nDirs, 10));
  for (size_t k = 0; k < cd.get_decimated_num_paths(); k++)
  {
    cd.get_path_clearance_decimated(k)[1.0] = 1.0;
  }
  return cd;
}
}  // namespace

// ---------------------------------------------------------------------------
//  Factory
// ---------------------------------------------------------------------------
TEST(HolonomicMethods, factory_creates_the_three_known_methods)
{
  for (const char* name : {"CHolonomicVFF", "CHolonomicND", "CHolonomicFullEval"})
  {
    auto m = CHRM::Factory(name);
    ASSERT_TRUE(m) << name;
    EXPECT_EQ(m->getConfigFileSectionName(), name);
  }
}

TEST(HolonomicMethods, factory_rejects_unknown_and_unrelated_classes)
{
  EXPECT_FALSE(CHRM::Factory("NoSuchHolonomicMethod"));
  EXPECT_FALSE(CHRM::Factory("mrpt::poses::CPose3D"));
}

TEST(HolonomicMethods, config_section_name_is_settable)
{
  CHolonomicVFF m;
  EXPECT_EQ(m.getConfigFileSectionName(), "CHolonomicVFF");
  m.setConfigFileSectionName("MySection");
  EXPECT_EQ(m.getConfigFileSectionName(), "MySection");
}

TEST(HolonomicMethods, associated_ptg_is_stored)
{
  CHolonomicVFF m;
  EXPECT_EQ(m.getAssociatedPTG(), nullptr);
  // A non-owning observer pointer; nullptr round-trips fine:
  m.setAssociatedPTG(nullptr);
  EXPECT_EQ(m.getAssociatedPTG(), nullptr);
}

// ---------------------------------------------------------------------------
//  CHolonomicVFF
// ---------------------------------------------------------------------------
TEST(CHolonomicVFF, config_file_roundtrip)
{
  CHolonomicVFF a;
  a.options.TARGET_SLOW_APPROACHING_DISTANCE = 0.25;
  a.options.TARGET_ATTRACTIVE_FORCE = 42.0;

  mrpt::config::CConfigFileMemory cfg;
  a.saveConfigFile(cfg);

  CHolonomicVFF b;
  b.initialize(cfg);
  EXPECT_NEAR(b.options.TARGET_SLOW_APPROACHING_DISTANCE, 0.25, 1e-9);
  EXPECT_NEAR(b.options.TARGET_ATTRACTIVE_FORCE, 42.0, 1e-9);

  EXPECT_NEAR(b.getTargetApproachSlowDownDistance(), 0.25, 1e-9);
  b.setTargetApproachSlowDownDistance(0.9);
  EXPECT_NEAR(b.options.TARGET_SLOW_APPROACHING_DISTANCE, 0.9, 1e-9);
}

TEST(CHolonomicVFF, serialization_roundtrip)
{
  CHolonomicVFF a;
  a.options.TARGET_ATTRACTIVE_FORCE = 33.0;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << a;
  buf.Seek(0);

  CHolonomicVFF b;
  arch >> b;
  EXPECT_NEAR(b.options.TARGET_ATTRACTIVE_FORCE, 33.0, 1e-9);
}

TEST(CHolonomicVFF, produces_a_log_record)
{
  CHolonomicVFF m;
  auto ni = make_nav_input(50, 0.5, .0);
  const auto no = m.navigate(ni);
  EXPECT_TRUE(no.logRecord);
  EXPECT_EQ(no.logRecord->getDirectionScores(), nullptr);
}

// ---------------------------------------------------------------------------
//  CHolonomicND
// ---------------------------------------------------------------------------
TEST(CHolonomicND, config_file_roundtrip)
{
  CHolonomicND a;
  a.options.TOO_CLOSE_OBSTACLE = 0.2;
  a.options.WIDE_GAP_SIZE_PERCENT = 0.3;
  a.options.RISK_EVALUATION_SECTORS_PERCENT = 0.2;
  a.options.RISK_EVALUATION_DISTANCE = 0.5;
  a.options.MAX_SECTOR_DIST_FOR_D2_PERCENT = 0.35;
  a.options.TARGET_SLOW_APPROACHING_DISTANCE = 0.7;
  a.options.factorWeights = {1.0, 2.0, 3.0, 4.0};

  mrpt::config::CConfigFileMemory cfg;
  a.saveConfigFile(cfg);

  CHolonomicND b;
  b.initialize(cfg);
  EXPECT_NEAR(b.options.TOO_CLOSE_OBSTACLE, 0.2, 1e-9);
  EXPECT_NEAR(b.options.WIDE_GAP_SIZE_PERCENT, 0.3, 1e-9);
  EXPECT_NEAR(b.options.RISK_EVALUATION_DISTANCE, 0.5, 1e-9);
  EXPECT_NEAR(b.options.TARGET_SLOW_APPROACHING_DISTANCE, 0.7, 1e-9);
  ASSERT_EQ(b.options.factorWeights.size(), 4U);
  EXPECT_NEAR(b.options.factorWeights[2], 3.0, 1e-9);
}

TEST(CHolonomicND, serialization_roundtrip)
{
  CHolonomicND a;
  a.options.RISK_EVALUATION_DISTANCE = 0.77;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << a;
  buf.Seek(0);

  CHolonomicND b;
  arch >> b;
  EXPECT_NEAR(b.options.RISK_EVALUATION_DISTANCE, 0.77, 1e-9);
}

TEST(CHolonomicND, log_record_reports_the_selected_gap)
{
  CHolonomicND m;
  auto ni = make_nav_input(60, 0.5, .0);
  // A blocked sector forces the method to pick a gap:
  for (size_t i = 25; i < 35; i++) ni.obstacles[i] = 0.05;

  const auto no = m.navigate(ni);
  ASSERT_TRUE(no.logRecord);
  auto* nd = dynamic_cast<CLogFileRecord_ND*>(no.logRecord.get());
  ASSERT_NE(nd, nullptr);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *nd;
  buf.Seek(0);
  CLogFileRecord_ND nd2;
  arch >> nd2;
  EXPECT_EQ(nd2.gaps_ini.size(), nd->gaps_ini.size());
}

// ---------------------------------------------------------------------------
//  CHolonomicFullEval
// ---------------------------------------------------------------------------
TEST(CHolonomicFullEval, config_file_roundtrip)
{
  CHolonomicFullEval a;
  a.options.TOO_CLOSE_OBSTACLE = 0.2;
  a.options.TARGET_SLOW_APPROACHING_DISTANCE = 0.5;
  a.options.OBSTACLE_SLOW_DOWN_DISTANCE = 0.25;
  a.options.HYSTERESIS_SECTOR_COUNT = 7;
  a.options.LOG_SCORE_MATRIX = true;
  a.options.clearance_threshold_ratio = 0.11;
  a.options.gap_width_ratio_threshold = 0.22;

  mrpt::config::CConfigFileMemory cfg;
  a.saveConfigFile(cfg);

  CHolonomicFullEval b;
  b.initialize(cfg);
  EXPECT_NEAR(b.options.TOO_CLOSE_OBSTACLE, 0.2, 1e-9);
  EXPECT_NEAR(b.options.OBSTACLE_SLOW_DOWN_DISTANCE, 0.25, 1e-9);
  EXPECT_NEAR(b.options.HYSTERESIS_SECTOR_COUNT, 7, 1e-9);
  EXPECT_TRUE(b.options.LOG_SCORE_MATRIX);
  EXPECT_NEAR(b.options.clearance_threshold_ratio, 0.11, 1e-9);
  EXPECT_NEAR(b.options.gap_width_ratio_threshold, 0.22, 1e-9);
  EXPECT_EQ(b.options.factorWeights.size(), a.options.factorWeights.size());
  EXPECT_EQ(b.options.PHASE_FACTORS.size(), a.options.PHASE_FACTORS.size());
}

TEST(CHolonomicFullEval, config_load_validates_the_phase_definitions)
{
  CHolonomicFullEval a;
  mrpt::config::CConfigFileMemory cfg;
  a.saveConfigFile(cfg);

  // Declare more phases than the ones actually described:
  cfg.write("CHolonomicFullEval", "PHASE_COUNT", 99);
  CHolonomicFullEval b;
  EXPECT_ANY_THROW(b.initialize(cfg));
}

TEST(CHolonomicFullEval, serialization_roundtrip)
{
  CHolonomicFullEval a;
  a.options.TOO_CLOSE_OBSTACLE = 0.33;
  a.options.clearance_threshold_ratio = 0.44;
  a.options.gap_width_ratio_threshold = 0.55;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << a;
  buf.Seek(0);

  CHolonomicFullEval b;
  arch >> b;
  EXPECT_NEAR(b.options.TOO_CLOSE_OBSTACLE, 0.33, 1e-9);
  EXPECT_NEAR(b.options.clearance_threshold_ratio, 0.44, 1e-9);
  EXPECT_NEAR(b.options.gap_width_ratio_threshold, 0.55, 1e-9);
}

TEST(CHolonomicFullEval, score_matrix_is_logged_when_enabled)
{
  CHolonomicFullEval m;
  m.options.LOG_SCORE_MATRIX = true;

  auto ni = make_nav_input(40, 0.5, .0);
  const auto cd = make_clearance(40);
  ni.clearance = &cd;
  const auto no = m.navigate(ni);
  ASSERT_TRUE(no.logRecord);
  auto* fe = dynamic_cast<CLogFileRecord_FullEval*>(no.logRecord.get());
  ASSERT_NE(fe, nullptr);
  const auto* scores = fe->getDirectionScores();
  ASSERT_NE(scores, nullptr);
  EXPECT_GT(scores->rows(), 0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *fe;
  buf.Seek(0);
  CLogFileRecord_FullEval fe2;
  arch >> fe2;
  EXPECT_EQ(fe2.selectedSector, fe->selectedSector);
}

TEST(CHolonomicFullEval, no_score_matrix_when_disabled)
{
  auto ni = make_nav_input(40, 0.5, .0);
  const auto cd = make_clearance(40);
  ni.clearance = &cd;

  const auto scoreMatrixSize = [&](bool enableLogging)
  {
    CHolonomicFullEval m;
    m.options.LOG_SCORE_MATRIX = enableLogging;
    const auto no = m.navigate(ni);
    auto* fe = dynamic_cast<CLogFileRecord_FullEval*>(no.logRecord.get());
    return fe ? fe->dirs_scores.size() : 0U;
  };

  // The score matrix is only copied into the log record on demand:
  EXPECT_LT(scoreMatrixSize(false), scoreMatrixSize(true));
}

TEST(CHolonomicFullEval, target_approach_slowdown_can_be_disabled)
{
  CHolonomicFullEval m;
  m.options.TARGET_SLOW_APPROACHING_DISTANCE = 5.0;  // always "close"

  auto ni = make_nav_input(40, 0.2, .0);  // target very near
  const auto cd = make_clearance(40);
  ni.clearance = &cd;
  const auto slowNo = m.navigate(ni);

  m.enableApproachTargetSlowDown(false);
  const auto fastNo = m.navigate(ni);

  EXPECT_GE(fastNo.desiredSpeed, slowNo.desiredSpeed);
}

TEST(CHolonomicFullEval, all_directions_blocked_yields_a_stop)
{
  CHolonomicFullEval m;
  auto ni = make_nav_input(40, 1.0, .0);
  const auto cd = make_clearance(40);
  ni.clearance = &cd;
  for (auto& o : ni.obstacles) o = 0.01;  // everything too close

  const auto no = m.navigate(ni);
  EXPECT_NEAR(no.desiredSpeed, .0, 1e-9);
}

TEST(CHolonomicND, all_directions_blocked_yields_a_stop)
{
  CHolonomicND m;
  auto ni = make_nav_input(40, 1.0, .0);
  for (auto& o : ni.obstacles) o = 0.01;

  const auto no = m.navigate(ni);
  EXPECT_NEAR(no.desiredSpeed, .0, 1e-9);
}

TEST(HolonomicMethods, multiple_targets_are_accepted)
{
  for (const char* name : {"CHolonomicVFF", "CHolonomicND", "CHolonomicFullEval"})
  {
    auto m = CHRM::Factory(name);
    ASSERT_TRUE(m) << name;

    auto ni = make_nav_input(40, 0.5, .0);
    const auto cd = make_clearance(40);
    ni.clearance = &cd;
    ni.targets.emplace_back(0.8, 0.2, .0);  // a second, higher-priority target

    const auto no = m->navigate(ni);
    EXPECT_GE(no.desiredSpeed, .0) << name;
    EXPECT_LE(no.desiredSpeed, ni.maxRobotSpeed + 1e-6) << name;
  }
}
