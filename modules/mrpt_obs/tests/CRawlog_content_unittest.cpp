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
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::obs;

namespace
{
CActionCollection buildActionCollection()
{
  CActionCollection ac;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(mrpt::poses::CPose2D(0.1, 0.0, 0.0), opts);
  ac.insert(act);
  return ac;
}

CSensoryFrame buildSensoryFrame()
{
  CSensoryFrame sf;
  auto obs = CObservationOdometry::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->sensorLabel = "ODO";
  sf.insert(obs);
  return sf;
}
}  // namespace

TEST(CRawlogContent, EmptyAndClear)
{
  CRawlog rawlog;
  EXPECT_TRUE(rawlog.empty());
  EXPECT_EQ(rawlog.size(), 0u);

  auto sf = buildSensoryFrame();
  rawlog.insert(sf);
  EXPECT_FALSE(rawlog.empty());
  EXPECT_EQ(rawlog.size(), 1u);

  rawlog.clear();
  EXPECT_TRUE(rawlog.empty());
}

TEST(CRawlogContent, InsertActionObservationsAndTypes)
{
  CRawlog rawlog;

  auto ac = buildActionCollection();
  rawlog.insert(ac);

  auto sf = buildSensoryFrame();
  rawlog.insert(sf);

  auto obs = CObservationOdometry::Create();
  rawlog.insert(std::dynamic_pointer_cast<mrpt::serialization::CSerializable>(obs));

  ASSERT_EQ(rawlog.size(), 3u);
  EXPECT_EQ(rawlog.getType(0), CRawlog::TEntryType::etActionCollection);
  EXPECT_EQ(rawlog.getType(1), CRawlog::TEntryType::etSensoryFrame);
  EXPECT_EQ(rawlog.getType(2), CRawlog::TEntryType::etObservation);

  EXPECT_NO_THROW(rawlog.getAsAction(0));
  EXPECT_THROW(rawlog.getAsAction(1), std::exception);
  EXPECT_NO_THROW(rawlog.getAsObservations(1));
  EXPECT_THROW(rawlog.getAsObservations(0), std::exception);
  EXPECT_NO_THROW(rawlog.getAsObservation(2));
  EXPECT_THROW(rawlog.getAsObservation(0), std::exception);
  EXPECT_NO_THROW(rawlog.getAsGeneric(0));

  EXPECT_THROW(rawlog.getType(100), std::exception);
  EXPECT_THROW(rawlog.getAsAction(100), std::exception);
  EXPECT_THROW(rawlog.getAsObservations(100), std::exception);
  EXPECT_THROW(rawlog.getAsObservation(100), std::exception);
  EXPECT_THROW(rawlog.getAsGeneric(100), std::exception);
}

TEST(CRawlogContent, InsertSingleAction)
{
  CRawlog rawlog;
  CActionRobotMovement2D act;
  CActionRobotMovement2D::TMotionModelOptions opts;
  act.computeFromOdometry(mrpt::poses::CPose2D(0.2, 0.1, 0.05), opts);
  rawlog.insert(act);
  ASSERT_EQ(rawlog.size(), 1u);
  EXPECT_EQ(rawlog.getType(0), CRawlog::TEntryType::etActionCollection);
}

TEST(CRawlogContent, InsertCommentSetsCommentText)
{
  CRawlog rawlog;
  auto comment = CObservationComment::Create();
  comment->text = "hello rawlog";
  rawlog.insert(std::dynamic_pointer_cast<mrpt::serialization::CSerializable>(comment));

  // Comments are not stored as regular entries, but merged into the rawlog's
  // comment text:
  EXPECT_TRUE(rawlog.empty());
  EXPECT_EQ(rawlog.getCommentText(), "hello rawlog");

  std::string t;
  rawlog.getCommentText(t);
  EXPECT_EQ(t, "hello rawlog");

  rawlog.setCommentText("new comment");
  EXPECT_EQ(rawlog.getCommentText(), "new comment");

  mrpt::config::CConfigFileMemory cfg;
  rawlog.getCommentTextAsConfigFile(cfg);
}

TEST(CRawlogContent, RemoveSingleAndRange)
{
  CRawlog rawlog;
  for (int i = 0; i < 5; i++)
  {
    auto obs = CObservationOdometry::Create();
    rawlog.insert(std::dynamic_pointer_cast<mrpt::serialization::CSerializable>(obs));
  }
  ASSERT_EQ(rawlog.size(), 5u);

  rawlog.remove(0);
  EXPECT_EQ(rawlog.size(), 4u);

  rawlog.remove(0, 1);
  EXPECT_EQ(rawlog.size(), 2u);

  EXPECT_THROW(rawlog.remove(100), std::exception);
  EXPECT_THROW(rawlog.remove(0, 100), std::exception);
}

TEST(CRawlogContent, SwapAndSelfSwap)
{
  CRawlog a, b;
  auto obsA = CObservationOdometry::Create();
  a.insert(std::dynamic_pointer_cast<mrpt::serialization::CSerializable>(obsA));
  a.setCommentText("A");

  EXPECT_TRUE(b.empty());
  a.swap(b);
  EXPECT_TRUE(a.empty());
  EXPECT_FALSE(b.empty());
  EXPECT_EQ(b.getCommentText(), "A");

  b.swap(b);  // self-swap should be a no-op
  EXPECT_FALSE(b.empty());
}

TEST(CRawlogContent, IteratorsAndErase)
{
  CRawlog rawlog;
  auto ac = buildActionCollection();
  rawlog.insert(ac);
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);

  size_t count = 0;
  for (auto it = rawlog.begin(); it != rawlog.end(); ++it) count++;
  EXPECT_EQ(count, 2u);

  auto it = rawlog.begin();
  EXPECT_EQ(it.getType(), CRawlog::TEntryType::etActionCollection);
  auto it2 = it++;
  EXPECT_TRUE(it2 != it);
  it2 = ++it2;

  rawlog.erase(rawlog.begin());
  EXPECT_EQ(rawlog.size(), 1u);

  // const_iterator:
  const CRawlog& cr = rawlog;
  size_t constCount = 0;
  for (auto cit = cr.begin(); cit != cr.end(); ++cit) constCount++;
  EXPECT_EQ(constCount, 1u);
}

TEST(CRawlogContent, SaveAndLoadRawlogFile)
{
  CRawlog rawlog;
  auto ac = buildActionCollection();
  rawlog.insert(ac);
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);
  rawlog.setCommentText("test comment");

  const std::string tmpFile = mrpt::system::getTempFileName() + ".rawlog";
  ASSERT_TRUE(rawlog.saveToRawLogFile(tmpFile));

  CRawlog rawlog2;
  ASSERT_TRUE(rawlog2.loadFromRawLogFile(tmpFile));
  EXPECT_EQ(rawlog2.size(), rawlog.size());
  EXPECT_EQ(rawlog2.getCommentText(), "test comment");

  mrpt::system::deleteFile(tmpFile);
}

TEST(CRawlogContent, LoadFromMissingFileReturnsFalse)
{
  CRawlog rawlog;
  EXPECT_FALSE(rawlog.loadFromRawLogFile("/nonexistent/path/to/file.rawlog"));
}

TEST(CRawlogContent, SaveEmbeddedFullRawlogAndReload)
{
  // Saving a full CRawlog object (rather than the loose sequence saved by
  // saveToRawLogFile) and reloading it exercises the "whole CRawlog object"
  // branch in loadFromRawLogFile():
  CRawlog rawlog;
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);

  const std::string tmpFile = mrpt::system::getTempFileName() + "_full.rawlog";
  {
    mrpt::io::CFileGZOutputStream fo(tmpFile);
    auto arch = mrpt::serialization::archiveFrom(fo);
    arch << rawlog;
  }

  CRawlog rawlog2;
  ASSERT_TRUE(rawlog2.loadFromRawLogFile(tmpFile));
  EXPECT_EQ(rawlog2.size(), rawlog.size());

  mrpt::system::deleteFile(tmpFile);
}

TEST(CRawlogContent, SerializationRoundtrip)
{
  CRawlog rawlog;
  auto ac = buildActionCollection();
  rawlog.insert(ac);
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);
  rawlog.setCommentText("roundtrip");

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << rawlog;
  buf.Seek(0);

  CRawlog rawlog2;
  arch >> rawlog2;

  EXPECT_EQ(rawlog2.size(), rawlog.size());
  EXPECT_EQ(rawlog2.getCommentText(), "roundtrip");
}

TEST(CRawlogContent, ReadActionObservationPair)
{
  CRawlog rawlog;
  auto ac = buildActionCollection();
  rawlog.insert(ac);
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  for (const auto& e : rawlog) arch << *e;
  buf.Seek(0);

  CActionCollection::Ptr action;
  CSensoryFrame::Ptr observations;
  size_t rawlogEntry = 0;
  const bool ok = CRawlog::readActionObservationPair(arch, action, observations, rawlogEntry);
  EXPECT_TRUE(ok);
  ASSERT_TRUE(action);
  ASSERT_TRUE(observations);
  EXPECT_EQ(rawlogEntry, 2u);
}

TEST(CRawlogContent, ReadActionObservationPairEOF)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  CActionCollection::Ptr action;
  CSensoryFrame::Ptr observations;
  size_t rawlogEntry = 0;
  const bool ok = CRawlog::readActionObservationPair(arch, action, observations, rawlogEntry);
  EXPECT_FALSE(ok);
}

TEST(CRawlogContent, GetActionObservationPairOrObservationBothForms)
{
  // Case 1: action + sensory frame pair
  {
    CRawlog rawlog;
    auto ac = buildActionCollection();
    rawlog.insert(ac);
    auto sf = buildSensoryFrame();
    rawlog.insert(sf);

    mrpt::io::CMemoryStream buf;
    auto arch = mrpt::serialization::archiveFrom(buf);
    for (const auto& e : rawlog) arch << *e;
    buf.Seek(0);

    CActionCollection::Ptr action;
    CSensoryFrame::Ptr observations;
    CObservation::Ptr observation;
    size_t rawlogEntry = 0;
    const bool ok = CRawlog::getActionObservationPairOrObservation(
        arch, action, observations, observation, rawlogEntry);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(action);
    EXPECT_TRUE(observations);
    EXPECT_FALSE(observation);
  }
  // Case 2: standalone observation (format #2 rawlogs)
  {
    mrpt::io::CMemoryStream buf;
    auto arch = mrpt::serialization::archiveFrom(buf);
    auto obs = CObservationOdometry::Create();
    arch << *obs;
    buf.Seek(0);

    CActionCollection::Ptr action;
    CSensoryFrame::Ptr observations;
    CObservation::Ptr observation;
    size_t rawlogEntry = 0;
    const bool ok = CRawlog::getActionObservationPairOrObservation(
        arch, action, observations, observation, rawlogEntry);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(observation);
  }
}

TEST(CRawlogContent, GetActionObservationPairNonStatic)
{
  CRawlog rawlog;
  auto ac = buildActionCollection();
  rawlog.insert(ac);
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);

  CActionCollection::ConstPtr action;
  CSensoryFrame::ConstPtr observations;
  size_t rawlogEntry = 0;
  const bool ok = rawlog.getActionObservationPair(action, observations, rawlogEntry);
  EXPECT_TRUE(ok);
  ASSERT_TRUE(action);
  ASSERT_TRUE(observations);
}

TEST(CRawlogContent, GetActionObservationPairFailsWhenMissing)
{
  CRawlog rawlog;
  auto sf = buildSensoryFrame();
  rawlog.insert(sf);  // no action collection present

  CActionCollection::ConstPtr action;
  CSensoryFrame::ConstPtr observations;
  size_t rawlogEntry = 0;
  const bool ok = rawlog.getActionObservationPair(action, observations, rawlogEntry);
  EXPECT_FALSE(ok);
}

TEST(CRawlogContent, FindObservationsByClassInRange)
{
  CRawlog rawlog;
  const auto t0 = mrpt::Clock::now();
  for (int i = 0; i < 5; i++)
  {
    auto obs = CObservationOdometry::Create();
    obs->timestamp = t0 + std::chrono::seconds(i);
    rawlog.insert(std::dynamic_pointer_cast<mrpt::serialization::CSerializable>(obs));
  }

  mrpt::obs::TListTimeAndObservations found;
  rawlog.findObservationsByClassInRange(
      t0 + std::chrono::seconds(1), t0 + std::chrono::seconds(4), CLASS_ID(CObservationOdometry),
      found);
  EXPECT_EQ(found.size(), 3u);  // indices 1,2,3 (t0+4 excluded, half-open range)
}

TEST(CRawlogContent, FindObservationsByClassInRangeEmptyRawlog)
{
  CRawlog rawlog;
  mrpt::obs::TListTimeAndObservations found;
  rawlog.findObservationsByClassInRange(
      mrpt::Clock::now(), mrpt::Clock::now(), CLASS_ID(CObservationOdometry), found);
  EXPECT_TRUE(found.empty());
}

TEST(CRawlogContent, DetectImagesDirectoryFallback)
{
  const std::string result = CRawlog::detectImagesDirectory("/tmp/nonexistent_rawlog_xyz.rawlog");
  EXPECT_FALSE(result.empty());
}
