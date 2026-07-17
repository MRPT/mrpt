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
#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cmath>
#include <filesystem>
#include <sstream>

using mrpt::maps::CBeacon;
using mrpt::maps::CBeaconMap;

static CBeacon makeGaussianBeacon(mrpt::maps::CBeacon::TBeaconID id, double x, double y, double z)
{
  CBeacon b;
  b.m_ID = id;
  b.m_typePDF = CBeacon::pdfGauss;
  b.m_locationGauss.mean = mrpt::poses::CPoint3D(x, y, z);
  b.m_locationGauss.cov.setIdentity();
  b.m_locationGauss.cov *= 0.01;
  return b;
}

// =========================================================================
//  Basic construction and isEmpty
// =========================================================================

TEST(CBeaconMap, EmptyOnConstruction)
{
  CBeaconMap m;
  EXPECT_TRUE(m.isEmpty());
  EXPECT_EQ(m.size(), 0u);
}

// =========================================================================
//  push_back, size, operator[], get
// =========================================================================

TEST(CBeaconMap, PushBackAndAccess)
{
  CBeaconMap m;

  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));
  m.push_back(makeGaussianBeacon(2, 3.0, 4.0, 0.0));
  m.push_back(makeGaussianBeacon(3, 5.0, 6.0, 0.0));

  EXPECT_FALSE(m.isEmpty());
  EXPECT_EQ(m.size(), 3u);

  EXPECT_EQ(m[0].m_ID, 1);
  EXPECT_EQ(m[1].m_ID, 2);
  EXPECT_EQ(m.get(2).m_ID, 3);

  EXPECT_NEAR(m[0].m_locationGauss.mean.x(), 1.0, 1e-9);
  EXPECT_NEAR(m[1].m_locationGauss.mean.y(), 4.0, 1e-9);
}

// =========================================================================
//  Mutable access via operator[] and get
// =========================================================================

TEST(CBeaconMap, MutableAccess)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(10, 0, 0, 0));

  m[0].m_ID = 99;
  EXPECT_EQ(m.get(0).m_ID, 99);
}

// =========================================================================
//  Iterators
// =========================================================================

TEST(CBeaconMap, IteratorAccess)
{
  CBeaconMap m;
  for (int i = 0; i < 5; ++i) m.push_back(makeGaussianBeacon(i, double(i), 0, 0));

  int count = 0;
  for (const auto& b : m)
  {
    EXPECT_EQ(b.m_ID, count);
    ++count;
  }
  EXPECT_EQ(count, 5);
}

// =========================================================================
//  internal_clear via the CMetricMap interface
// =========================================================================

TEST(CBeaconMap, Clear)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1, 1, 1));
  m.push_back(makeGaussianBeacon(2, 2, 2, 2));
  EXPECT_EQ(m.size(), 2u);

  m.clear();
  EXPECT_TRUE(m.isEmpty());
  EXPECT_EQ(m.size(), 0u);
}

// =========================================================================
//  resize
// =========================================================================

TEST(CBeaconMap, Resize)
{
  CBeaconMap m;
  m.resize(4);
  EXPECT_EQ(m.size(), 4u);

  m.resize(2);
  EXPECT_EQ(m.size(), 2u);
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CBeaconMap, SerializeRoundTrip)
{
  CBeaconMap src;
  src.push_back(makeGaussianBeacon(7, 1.5, 2.5, 0.5));
  src.push_back(makeGaussianBeacon(8, -1.0, -2.0, 0.0));

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CBeaconMap dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    auto* ptr = dynamic_cast<CBeaconMap*>(obj.get());
    ASSERT_NE(ptr, nullptr);
    dst = *ptr;
  }

  EXPECT_EQ(dst.size(), src.size());
  EXPECT_EQ(dst[0].m_ID, 7);
  EXPECT_EQ(dst[1].m_ID, 8);
  EXPECT_NEAR(dst[0].m_locationGauss.mean.x(), 1.5, 1e-6);
  EXPECT_NEAR(dst[1].m_locationGauss.mean.y(), -2.0, 1e-6);
}

// =========================================================================
//  changeCoordinatesReference
// =========================================================================

TEST(CBeaconMap, ChangeCoordinatesReference)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 0.0, 0.0));

  // Translate by (10, 0, 0)
  mrpt::poses::CPose3D offset(10.0, 0.0, 0.0, 0, 0, 0);
  m.changeCoordinatesReference(offset);

  EXPECT_NEAR(m[0].m_locationGauss.mean.x(), 11.0, 1e-6);
}

// =========================================================================
//  saveToMATLABScript3D (just check it returns true without crashing)
// =========================================================================

TEST(CBeaconMap, SaveToMATLABScript)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));

  const auto fname = (std::filesystem::temp_directory_path() / "test_beaconmap.m").string();
  bool ok = m.saveToMATLABScript3D(fname);
  EXPECT_TRUE(ok);
}

// =========================================================================
//  internal_insertObservation(): wrong observation type
// =========================================================================

TEST(CBeaconMap, InsertObservationWrongTypeReturnsFalse)
{
  CBeaconMap m;
  mrpt::obs::CObservationOdometry obs;

  EXPECT_FALSE(m.insertObservation(obs));
  EXPECT_TRUE(m.isEmpty());
}

// =========================================================================
//  internal_insertObservation(): non-sensible ranges are ignored
// =========================================================================

TEST(CBeaconMap, InsertObservationInvalidRangeIgnored)
{
  CBeaconMap m;
  mrpt::obs::CObservationBeaconRanges obs;
  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 1;
  meas.sensedDistance = 0.0f;  // Non-sensible range: must be ignored
  obs.sensedData.push_back(meas);

  // The observation type is recognized, so it returns true, but nothing
  // gets inserted since the range is not sensible:
  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_TRUE(m.isEmpty());
}

// =========================================================================
//  internal_insertObservation(): create new beacons, MonteCarlo mode
// =========================================================================

TEST(CBeaconMap, InsertObservationMonteCarloCreatesNewBeacon)
{
  CBeaconMap m;
  m.insertionOptions.insertAsMonteCarlo = true;
  m.insertionOptions.MC_numSamplesPerMeter = 100;

  mrpt::obs::CObservationBeaconRanges obs;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = 5.0f;
    obs.sensedData.push_back(meas);
  }
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 2;
    meas.sensedDistance = 3.0f;
    obs.sensedData.push_back(meas);
  }

  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_EQ(m.size(), 2u);

  const CBeacon* b1 = m.getBeaconByID(1);
  ASSERT_NE(b1, nullptr);
  EXPECT_EQ(b1->m_typePDF, CBeacon::pdfMonteCarlo);
  EXPECT_GT(b1->m_locationMC.size(), 0u);

  const CBeacon* b2 = m.getBeaconByID(2);
  ASSERT_NE(b2, nullptr);
  EXPECT_EQ(b2->m_typePDF, CBeacon::pdfMonteCarlo);
}

// =========================================================================
//  internal_insertObservation(): create new beacons, SOG mode
// =========================================================================

TEST(CBeaconMap, InsertObservationSOGCreatesNewBeacon)
{
  CBeaconMap m;
  m.insertionOptions.insertAsMonteCarlo = false;

  mrpt::obs::CObservationBeaconRanges obs;
  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 1;
  meas.sensedDistance = 5.0f;
  obs.sensedData.push_back(meas);

  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_EQ(m.size(), 1u);

  const CBeacon* b = m.getBeaconByID(1);
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->m_typePDF, CBeacon::pdfSOG);
  EXPECT_GT(b->m_locationSOG.size(), 0u);
}

// =========================================================================
//  internal_insertObservation(): fuse into an existing Gaussian beacon
//  (EKF update, checked against the hand-derived expected values)
// =========================================================================

TEST(CBeaconMap, InsertObservationGaussianFuseUpdatesMeanAndCov)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 10.0, 0.0, 0.0));

  mrpt::obs::CObservationBeaconRanges obs;
  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 1;
  meas.sensedDistance = 12.0f;  // Biased range: pulls the mean farther away
  obs.sensedData.push_back(meas);

  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_EQ(m.size(), 1u);

  const CBeacon* b = m.getBeaconByID(1);
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->m_typePDF, CBeacon::pdfGauss);
  EXPECT_NEAR(b->m_locationGauss.mean.x(), 11.21951, 1e-3);
  EXPECT_NEAR(b->m_locationGauss.mean.y(), 0.0, 1e-6);
  EXPECT_NEAR(b->m_locationGauss.mean.z(), 0.0, 1e-6);
  EXPECT_NEAR(b->m_locationGauss.cov(0, 0), 0.0039024, 1e-6);
  EXPECT_LT(b->m_locationGauss.cov(0, 0), 0.01);
}

// =========================================================================
//  internal_insertObservation(): fuse into an existing MonteCarlo beacon.
//  A second range from a different robot pose must either prune unlikely
//  particles or collapse the beacon into a Gaussian.
// =========================================================================

TEST(CBeaconMap, InsertObservationMonteCarloFusePrunesOrCollapses)
{
  CBeaconMap m;
  m.insertionOptions.insertAsMonteCarlo = true;
  m.insertionOptions.MC_numSamplesPerMeter = 200;

  mrpt::obs::CObservationBeaconRanges obs1;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = 5.0f;
    obs1.sensedData.push_back(meas);
  }
  const mrpt::poses::CPose3D poseA(0.0, 0.0, 0.0, 0, 0, 0);
  ASSERT_TRUE(m.insertObservation(obs1, poseA));

  const CBeacon* b = m.getBeaconByID(1);
  ASSERT_NE(b, nullptr);
  ASSERT_EQ(b->m_typePDF, CBeacon::pdfMonteCarlo);
  const size_t nBefore = b->m_locationMC.size();
  ASSERT_GT(nBefore, 0u);

  // Second reading from a different location, constraining the ring of
  // candidate positions and pruning unlikely particles:
  mrpt::obs::CObservationBeaconRanges obs2;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = static_cast<float>(std::sqrt(25.0 + 25.0));
    obs2.sensedData.push_back(meas);
  }
  const mrpt::poses::CPose3D poseB(0.0, 5.0, 0.0, 0, 0, 0);
  EXPECT_TRUE(m.insertObservation(obs2, poseB));

  EXPECT_EQ(m.size(), 1u);
  const CBeacon* b2 = m.getBeaconByID(1);
  ASSERT_NE(b2, nullptr);
  const bool collapsedToGauss = (b2->m_typePDF == CBeacon::pdfGauss);
  const bool prunedParticles =
      (b2->m_typePDF == CBeacon::pdfMonteCarlo && b2->m_locationMC.size() < nBefore);
  EXPECT_TRUE(collapsedToGauss || prunedParticles);
}

// =========================================================================
//  internal_insertObservation(): fuse into an existing SOG beacon.
//  Analogous to the MonteCarlo case above.
// =========================================================================

TEST(CBeaconMap, InsertObservationSOGFusePrunesOrCollapses)
{
  CBeaconMap m;
  m.insertionOptions.insertAsMonteCarlo = false;

  mrpt::obs::CObservationBeaconRanges obs1;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = 5.0f;
    obs1.sensedData.push_back(meas);
  }
  const mrpt::poses::CPose3D poseA(0.0, 0.0, 0.0, 0, 0, 0);
  ASSERT_TRUE(m.insertObservation(obs1, poseA));

  const CBeacon* b = m.getBeaconByID(1);
  ASSERT_NE(b, nullptr);
  ASSERT_EQ(b->m_typePDF, CBeacon::pdfSOG);
  const size_t nBefore = b->m_locationSOG.size();
  ASSERT_GT(nBefore, 0u);

  mrpt::obs::CObservationBeaconRanges obs2;
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = static_cast<float>(std::sqrt(25.0 + 25.0));
    obs2.sensedData.push_back(meas);
  }
  const mrpt::poses::CPose3D poseB(0.0, 5.0, 0.0, 0, 0, 0);
  EXPECT_TRUE(m.insertObservation(obs2, poseB));

  EXPECT_EQ(m.size(), 1u);
  const CBeacon* b2 = m.getBeaconByID(1);
  ASSERT_NE(b2, nullptr);
  const bool collapsedToGauss = (b2->m_typePDF == CBeacon::pdfGauss);
  const bool prunedModes = (b2->m_typePDF == CBeacon::pdfSOG && b2->m_locationSOG.size() < nBefore);
  EXPECT_TRUE(collapsedToGauss || prunedModes);
}

// =========================================================================
//  internal_computeObservationLikelihood(): wrong observation type
// =========================================================================

TEST(CBeaconMap, ComputeObservationLikelihoodWrongTypeReturnsZero)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 5.0, 0.0, 0.0));

  mrpt::obs::CObservationOdometry obs;
  const double lik = m.computeObservationLikelihood(obs, mrpt::poses::CPose3D());
  EXPECT_DOUBLE_EQ(lik, 0.0);
}

// =========================================================================
//  internal_computeObservationLikelihood(): beacon ID not found falls back
//  to a uniform distribution term
// =========================================================================

TEST(CBeaconMap, ComputeObservationLikelihoodUnmatchedBeaconUniform)
{
  CBeaconMap m;  // Empty: no beacon will ever be found

  mrpt::obs::CObservationBeaconRanges obs;
  obs.minSensorDistance = 0.0f;
  obs.maxSensorDistance = 10.0f;

  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 42;
  meas.sensedDistance = 5.0f;
  obs.sensedData.push_back(meas);

  const double lik = m.computeObservationLikelihood(obs, mrpt::poses::CPose3D());
  EXPECT_NEAR(lik, std::log(1.0 / (10.0 - 0.0)), 1e-9);
}

// =========================================================================
//  internal_computeObservationLikelihood(): Gaussian beacon likelihood is
//  higher for the matching range than for a mismatching one
// =========================================================================

TEST(CBeaconMap, ComputeObservationLikelihoodGaussianDiscriminatesRange)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 5.0, 0.0, 0.0));

  auto makeObs = [](float sensedDistance)
  {
    mrpt::obs::CObservationBeaconRanges obs;
    mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
    meas.beaconID = 1;
    meas.sensedDistance = sensedDistance;
    obs.sensedData.push_back(meas);
    return obs;
  };

  const auto obsMatching = makeObs(5.0f);
  const auto obsMismatching = makeObs(9.0f);

  const double likMatching = m.computeObservationLikelihood(obsMatching, mrpt::poses::CPose3D());
  const double likMismatching =
      m.computeObservationLikelihood(obsMismatching, mrpt::poses::CPose3D());

  EXPECT_TRUE(std::isfinite(likMatching));
  EXPECT_TRUE(std::isfinite(likMismatching));
  EXPECT_GT(likMatching, likMismatching);
}

// =========================================================================
//  internal_computeObservationLikelihood(): MonteCarlo beacon
// =========================================================================

TEST(CBeaconMap, ComputeObservationLikelihoodMonteCarloFinite)
{
  CBeaconMap m;

  CBeacon b;
  b.m_ID = 1;
  b.m_typePDF = CBeacon::pdfMonteCarlo;
  b.m_locationMC.setSize(20);
  for (auto& p : b.m_locationMC.m_particles)
  {
    p.d->x = 5.0f;
    p.d->y = 0.0f;
    p.d->z = 0.0f;
    p.log_w = 0.0;
  }
  m.push_back(b);

  mrpt::obs::CObservationBeaconRanges obs;
  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 1;
  meas.sensedDistance = 5.0f;
  obs.sensedData.push_back(meas);

  const double lik = m.computeObservationLikelihood(obs, mrpt::poses::CPose3D());
  EXPECT_TRUE(std::isfinite(lik));
}

// =========================================================================
//  internal_computeObservationLikelihood(): SOG beacon
// =========================================================================

TEST(CBeaconMap, ComputeObservationLikelihoodSOGFinite)
{
  CBeaconMap m;

  CBeacon b;
  b.m_ID = 1;
  b.m_typePDF = CBeacon::pdfSOG;
  CBeacon::generateRingSOG(5.0f, b.m_locationSOG, &m, mrpt::poses::CPoint3D(0, 0, 0));
  m.push_back(b);

  mrpt::obs::CObservationBeaconRanges obs;
  mrpt::obs::CObservationBeaconRanges::TMeasurement meas;
  meas.beaconID = 1;
  meas.sensedDistance = 5.0f;
  obs.sensedData.push_back(meas);

  const double lik = m.computeObservationLikelihood(obs, mrpt::poses::CPose3D());
  EXPECT_TRUE(std::isfinite(lik));
}

// =========================================================================
//  computeMatchingWith3DLandmarks(): matches by beacon ID
// =========================================================================

TEST(CBeaconMap, ComputeMatchingWith3DLandmarks)
{
  CBeaconMap thisMap;
  thisMap.push_back(makeGaussianBeacon(1, 0.0, 0.0, 0.0));
  thisMap.push_back(makeGaussianBeacon(2, 1.0, 0.0, 0.0));
  thisMap.push_back(makeGaussianBeacon(3, 2.0, 0.0, 0.0));

  CBeaconMap otherMap;
  otherMap.push_back(makeGaussianBeacon(2, 10.0, 0.0, 0.0));
  otherMap.push_back(makeGaussianBeacon(3, 11.0, 0.0, 0.0));
  otherMap.push_back(makeGaussianBeacon(4, 12.0, 0.0, 0.0));

  mrpt::tfest::TMatchingPairList correspondences;
  float correspondencesRatio = 0;
  std::vector<bool> otherCorrespondences;

  thisMap.computeMatchingWith3DLandmarks(
      &otherMap, correspondences, correspondencesRatio, otherCorrespondences);

  EXPECT_EQ(correspondences.size(), 2u);
  EXPECT_NEAR(correspondencesRatio, 2.0f * 2.0f / (3.0f + 3.0f), 1e-6f);

  ASSERT_EQ(otherCorrespondences.size(), 3u);
  EXPECT_TRUE(otherCorrespondences[0]);
  EXPECT_TRUE(otherCorrespondences[1]);
  EXPECT_FALSE(otherCorrespondences[2]);
}

// =========================================================================
//  determineMatching2D(): dispatches to computeMatchingWith3DLandmarks()
// =========================================================================

TEST(CBeaconMap, DetermineMatching2DMatchesById)
{
  CBeaconMap thisMap;
  thisMap.push_back(makeGaussianBeacon(1, 0.0, 0.0, 0.0));
  thisMap.push_back(makeGaussianBeacon(2, 1.0, 0.0, 0.0));

  CBeaconMap otherMap;
  otherMap.push_back(makeGaussianBeacon(2, 5.0, 5.0, 0.0));
  otherMap.push_back(makeGaussianBeacon(3, 6.0, 5.0, 0.0));

  mrpt::tfest::TMatchingPairList correspondences;
  mrpt::maps::TMatchingParams params;
  mrpt::maps::TMatchingExtraResults extraResults;

  thisMap.determineMatching2D(
      &otherMap, mrpt::poses::CPose2D(), correspondences, params, extraResults);

  EXPECT_EQ(correspondences.size(), 1u);
  EXPECT_NEAR(extraResults.correspondencesRatio, 2.0f * 1.0f / (2.0f + 2.0f), 1e-6f);
}

// =========================================================================
//  compute3DMatchingRatio()
// =========================================================================

TEST(CBeaconMap, Compute3DMatchingRatioMatchingMaps)
{
  CBeaconMap thisMap;
  thisMap.push_back(makeGaussianBeacon(1, 1.0, 2.0, 3.0));

  CBeaconMap otherMap;
  otherMap.push_back(makeGaussianBeacon(1, 0.0, 0.0, 0.0));

  mrpt::maps::TMatchingRatioParams params;
  const float ratio = thisMap.compute3DMatchingRatio(&otherMap, mrpt::poses::CPose3D(), params);

  EXPECT_NEAR(ratio, 1.0f, 1e-6f);
}

TEST(CBeaconMap, Compute3DMatchingRatioWrongMapType)
{
  CBeaconMap thisMap;
  thisMap.push_back(makeGaussianBeacon(1, 1.0, 2.0, 3.0));

  mrpt::maps::CSimplePointsMap otherMap;
  otherMap.insertPoint(1.0f, 2.0f, 3.0f);

  mrpt::maps::TMatchingRatioParams params;
  const float ratio = thisMap.compute3DMatchingRatio(&otherMap, mrpt::poses::CPose3D(), params);

  EXPECT_EQ(ratio, 0.0f);
}

// =========================================================================
//  getBeaconByID(): const and non-const overloads
// =========================================================================

TEST(CBeaconMap, GetBeaconByIDFoundAndNotFound)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 3.0));

  const CBeaconMap& cm = m;
  ASSERT_NE(cm.getBeaconByID(1), nullptr);
  EXPECT_EQ(cm.getBeaconByID(1)->m_ID, 1);
  EXPECT_EQ(cm.getBeaconByID(99), nullptr);

  CBeacon* found = m.getBeaconByID(1);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->m_ID, 1);
  EXPECT_EQ(m.getBeaconByID(99), nullptr);
}

// =========================================================================
//  simulateBeaconReadings(): out-of-range beacons are skipped
// =========================================================================

TEST(CBeaconMap, SimulateBeaconReadingsRespectsRange)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 5.0, 0.0, 0.0));   // within range
  m.push_back(makeGaussianBeacon(2, 50.0, 0.0, 0.0));  // out of range

  mrpt::obs::CObservationBeaconRanges obs;
  obs.minSensorDistance = 0.0f;
  obs.maxSensorDistance = 10.0f;
  obs.stdError = 0.0f;  // No noise, for a deterministic check

  const mrpt::poses::CPose3D robotPose;
  const mrpt::poses::CPoint3D sensorOnRobot(0, 0, 0);

  m.simulateBeaconReadings(robotPose, sensorOnRobot, obs);

  ASSERT_EQ(obs.sensedData.size(), 1u);
  EXPECT_EQ(obs.sensedData[0].beaconID, 1);
  EXPECT_NEAR(obs.sensedData[0].sensedDistance, 5.0, 1e-3);
}

// =========================================================================
//  saveMetricMapRepresentationToFile()
// =========================================================================

TEST(CBeaconMap, SaveMetricMapRepresentationToFile)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));

  const auto prefix = (std::filesystem::temp_directory_path() / "test_beaconmap_repr").string();
  m.saveMetricMapRepresentationToFile(prefix);

  EXPECT_TRUE(std::filesystem::exists(prefix + "_3D.m"));
  EXPECT_TRUE(std::filesystem::exists(prefix + "_covs.txt"));
  EXPECT_TRUE(std::filesystem::exists(prefix + "_population.txt"));
}

// =========================================================================
//  saveToTextFile()
// =========================================================================

TEST(CBeaconMap, SaveToTextFile)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));

  const auto fname = (std::filesystem::temp_directory_path() / "test_beaconmap_covs.txt").string();
  m.saveToTextFile(fname);

  ASSERT_TRUE(std::filesystem::exists(fname));
  EXPECT_GT(std::filesystem::file_size(fname), 0u);
}

// =========================================================================
//  getVisualizationInto()
// =========================================================================

TEST(CBeaconMap, GetVisualizationInto)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));

  mrpt::viz::CSetOfObjects obj;
  m.getVisualizationInto(obj);
  EXPECT_GT(obj.size(), 0u);
}

TEST(CBeaconMap, GetVisualizationIntoDisabled)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));
  m.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects obj;
  m.getVisualizationInto(obj);
  EXPECT_EQ(obj.size(), 0u);
}

// =========================================================================
//  optionsByName()
// =========================================================================

TEST(CBeaconMap, OptionsByName)
{
  CBeaconMap m;
  auto opts = m.optionsByName();

  ASSERT_EQ(opts.count("insertionOptions"), 1u);
  ASSERT_EQ(opts.count("likelihoodOptions"), 1u);
  EXPECT_EQ(opts.at("insertionOptions"), &m.insertionOptions);
  EXPECT_EQ(opts.at("likelihoodOptions"), &m.likelihoodOptions);
}

// =========================================================================
//  TLikelihoodOptions / TInsertionOptions: loadFromConfigFile,
//  dumpToTextStream
// =========================================================================

TEST(CBeaconMap, LikelihoodOptionsLoadDump)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[lik]
rangeStd=0.25
)"""");

  CBeaconMap::TLikelihoodOptions opts;
  opts.loadFromConfigFile(cfg, "lik");
  EXPECT_NEAR(opts.rangeStd, 0.25, 1e-9);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("rangeStd"), std::string::npos);
}

TEST(CBeaconMap, InsertionOptionsLoadDump)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[ins]
insertAsMonteCarlo=false
MC_numSamplesPerMeter=500
SOG_thresholdNegligible=15
)"""");

  CBeaconMap::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");
  EXPECT_FALSE(opts.insertAsMonteCarlo);
  EXPECT_EQ(opts.MC_numSamplesPerMeter, 500u);
  EXPECT_NEAR(opts.SOG_thresholdNegligible, 15.0f, 1e-6f);

  std::stringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("insertAsMonteCarlo"), std::string::npos);
}

// =========================================================================
//  TMapDefinition: internal_CreateFromMapDefinition() and
//  loadFromConfigFile_map_specific()
// =========================================================================

TEST(CBeaconMap, MapDefinitionCreateFromDefinition)
{
  CBeaconMap::TMapDefinition def;
  def.insertionOpts.insertAsMonteCarlo = false;
  def.likelihoodOpts.rangeStd = 0.5;

  auto map = CBeaconMap::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_FALSE(map->insertionOptions.insertAsMonteCarlo);
  EXPECT_NEAR(map->likelihoodOptions.rangeStd, 0.5, 1e-9);
}

TEST(CBeaconMap, MapDefinitionLoadFromConfigFile)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map]
beaconMap_count=1

[map_beaconMap_00_insertOpts]
insertAsMonteCarlo=false

[map_beaconMap_00_likelihoodOpts]
rangeStd=0.3
)"""");

  mrpt::maps::TSetOfMetricMapInitializers map_inits;
  map_inits.loadFromConfigFile(cfg, "map");
  ASSERT_EQ(map_inits.size(), 1u);

  const auto& mi = *map_inits.begin();
  auto map = CBeaconMap::CreateFromMapDefinition(*mi);
  ASSERT_NE(map, nullptr);
  EXPECT_FALSE(map->insertionOptions.insertAsMonteCarlo);
  EXPECT_NEAR(map->likelihoodOptions.rangeStd, 0.3, 1e-6);
}

// =========================================================================
//  CBeacon: per-PDF-type dispatch methods (getMean, bayesianFusion,
//  drawSingleSample, copyFrom, saveToTextFile, changeCoordinatesReference,
//  getVisualizationInto, getAsMatlabDrawCommands)
// =========================================================================

namespace
{
CBeacon makeMonteCarloBeacon(mrpt::maps::CBeacon::TBeaconID id, float x, float y, float z)
{
  CBeacon b;
  b.m_ID = id;
  b.m_typePDF = CBeacon::pdfMonteCarlo;
  b.m_locationMC.setSize(10);
  for (auto& p : b.m_locationMC.m_particles)
  {
    p.d->x = x;
    p.d->y = y;
    p.d->z = z;
    p.log_w = 0.0;
  }
  return b;
}

CBeacon makeSOGBeacon(mrpt::maps::CBeacon::TBeaconID id, const CBeaconMap& map)
{
  CBeacon b;
  b.m_ID = id;
  b.m_typePDF = CBeacon::pdfSOG;
  CBeacon::generateRingSOG(5.0f, b.m_locationSOG, &map, mrpt::poses::CPoint3D(0, 0, 0));
  return b;
}
}  // namespace

TEST(CBeacon, GetMeanAllPdfTypes)
{
  CBeaconMap map;
  mrpt::poses::CPoint3D p;

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  gauss.getMean(p);
  EXPECT_NEAR(p.x(), 1.0, 1e-9);

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  mc.getMean(p);
  EXPECT_NEAR(p.x(), 5.0, 1e-3);

  CBeacon sog = makeSOGBeacon(3, map);
  EXPECT_NO_THROW(sog.getMean(p));
}

TEST(CBeacon, GetCovarianceAndMeanAllPdfTypes)
{
  CBeaconMap map;

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  {
    const auto [cov, mean] = gauss.getCovarianceAndMean();
    EXPECT_NEAR(mean.x(), 1.0, 1e-9);
    EXPECT_GT(cov(0, 0), 0.0);
  }

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  EXPECT_NO_THROW(mc.getCovarianceAndMean());

  CBeacon sog = makeSOGBeacon(3, map);
  EXPECT_NO_THROW(sog.getCovarianceAndMean());
}

TEST(CBeacon, DrawSingleSampleAllPdfTypes)
{
  CBeaconMap map;
  mrpt::poses::CPoint3D sample;

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  EXPECT_NO_THROW(gauss.drawSingleSample(sample));

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  EXPECT_NO_THROW(mc.drawSingleSample(sample));

  CBeacon sog = makeSOGBeacon(3, map);
  EXPECT_NO_THROW(sog.drawSingleSample(sample));
}

TEST(CBeacon, CopyFromAllPdfTypes)
{
  mrpt::poses::CPointPDFGaussian src;
  src.mean = mrpt::poses::CPoint3D(3.0, 4.0, 5.0);
  src.cov.setIdentity();

  CBeacon gauss = makeGaussianBeacon(1, 0, 0, 0);
  gauss.copyFrom(src);
  EXPECT_NEAR(gauss.m_locationGauss.mean.x(), 3.0, 1e-9);

  CBeacon mc = makeMonteCarloBeacon(2, 0, 0, 0);
  EXPECT_NO_THROW(mc.copyFrom(src));

  CBeaconMap map;
  CBeacon sog = makeSOGBeacon(3, map);
  EXPECT_NO_THROW(sog.copyFrom(src));
}

TEST(CBeacon, BayesianFusionGaussian)
{
  mrpt::poses::CPointPDFGaussian p1;
  p1.mean = mrpt::poses::CPoint3D(0.0, 0.0, 0.0);
  p1.cov.setIdentity();

  mrpt::poses::CPointPDFGaussian p2;
  p2.mean = mrpt::poses::CPoint3D(2.0, 0.0, 0.0);
  p2.cov.setIdentity();

  CBeacon gauss = makeGaussianBeacon(1, 0, 0, 0);
  gauss.bayesianFusion(p1, p2, 10.0);
  // Fusing two equally-uncertain Gaussians should land the mean roughly
  // between the two:
  EXPECT_GT(gauss.m_locationGauss.mean.x(), 0.0);
  EXPECT_LT(gauss.m_locationGauss.mean.x(), 2.0);
}

TEST(CBeacon, SaveToTextFileAllPdfTypes)
{
  CBeaconMap map;
  const auto dir = std::filesystem::temp_directory_path();

  const auto tryType = [&](CBeacon& b, const std::string& tag)
  {
    const std::string file =
        (dir / ("mrpt_CBeacon_unittest_" + std::to_string(static_cast<long>(getpid())) + "_" + tag +
                ".txt"))
            .string();
    EXPECT_TRUE(b.saveToTextFile(file));
    EXPECT_TRUE(std::filesystem::exists(file));
    std::filesystem::remove(file);
  };

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  tryType(gauss, "gauss");

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  tryType(mc, "mc");

  CBeacon sog = makeSOGBeacon(3, map);
  tryType(sog, "sog");
}

TEST(CBeacon, ChangeCoordinatesReferenceAllPdfTypes)
{
  CBeaconMap map;
  const mrpt::poses::CPose3D newBase(1.0, 0.0, 0.0, 0, 0, 0);

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  gauss.changeCoordinatesReference(newBase);
  EXPECT_NEAR(gauss.m_locationGauss.mean.x(), 2.0, 1e-6);

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  EXPECT_NO_THROW(mc.changeCoordinatesReference(newBase));

  CBeacon sog = makeSOGBeacon(3, map);
  EXPECT_NO_THROW(sog.changeCoordinatesReference(newBase));
}

TEST(CBeacon, GetVisualizationIntoAllPdfTypes)
{
  CBeaconMap map;

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  {
    auto scene = mrpt::viz::CSetOfObjects::Create();
    EXPECT_NO_THROW(gauss.getVisualizationInto(*scene));
    EXPECT_GT(scene->size(), 0u);
  }

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  {
    auto scene = mrpt::viz::CSetOfObjects::Create();
    EXPECT_NO_THROW(mc.getVisualizationInto(*scene));
    EXPECT_GT(scene->size(), 0u);
  }

  CBeacon sog = makeSOGBeacon(3, map);
  {
    auto scene = mrpt::viz::CSetOfObjects::Create();
    EXPECT_NO_THROW(sog.getVisualizationInto(*scene));
    EXPECT_GT(scene->size(), 0u);
  }
}

TEST(CBeacon, GetAsMatlabDrawCommandsAllPdfTypes)
{
  CBeaconMap map;
  std::vector<std::string> lines;

  CBeacon gauss = makeGaussianBeacon(1, 1.0, 2.0, 3.0);
  gauss.getAsMatlabDrawCommands(lines);
  EXPECT_FALSE(lines.empty());

  CBeacon mc = makeMonteCarloBeacon(2, 5.0f, 0.0f, 0.0f);
  mc.getAsMatlabDrawCommands(lines);
  EXPECT_FALSE(lines.empty());

  CBeacon sog = makeSOGBeacon(3, map);
  sog.getAsMatlabDrawCommands(lines);
  EXPECT_FALSE(lines.empty());
}

TEST(CBeacon, GenerateObservationModelDistributionGaussian)
{
  CBeaconMap map;
  CBeacon gauss = makeGaussianBeacon(1, 5.0, 0.0, 0.0);

  mrpt::poses::CPointPDFSOG outPDF;
  gauss.generateObservationModelDistribution(
      5.0f, outPDF, &map, mrpt::poses::CPoint3D(0, 0, 0), mrpt::poses::CPoint3D(0, 0, 0), 0.0f);

  EXPECT_GT(outPDF.size(), 0u);
}

TEST(CBeacon, GenerateObservationModelDistributionSOG)
{
  CBeaconMap map;
  CBeacon sog = makeSOGBeacon(1, map);

  mrpt::poses::CPointPDFSOG outPDF;
  sog.generateObservationModelDistribution(
      5.0f, outPDF, &map, mrpt::poses::CPoint3D(0, 0, 0), mrpt::poses::CPoint3D(0, 0, 0), 0.0f);

  EXPECT_GT(outPDF.size(), 0u);
}

TEST(CBeacon, GenerateRingSOGWithMaxDistanceFromCenterPrunesModes)
{
  CBeaconMap map;
  mrpt::poses::CPointPDFSOG unbounded;
  CBeacon::generateRingSOG(5.0f, unbounded, &map, mrpt::poses::CPoint3D(0, 0, 0));

  mrpt::poses::CPointPDFSOG bounded;
  CBeacon::generateRingSOG(
      5.0f, bounded, &map, mrpt::poses::CPoint3D(0, 0, 0), nullptr,
      /*clearPreviousContentsOutPDF=*/true, mrpt::poses::CPoint3D(5, 0, 0),
      /*maxDistanceFromCenter=*/0.5f);

  EXPECT_LT(bounded.size(), unbounded.size());
}

TEST(CBeacon, SerializeRoundTripAllPdfTypes)
{
  CBeaconMap map;

  const auto roundTrip = [](const CBeacon& src) -> CBeacon
  {
    mrpt::io::CMemoryStream buf;
    {
      auto ar = mrpt::serialization::archiveFrom(buf);
      ar << src;
    }
    buf.Seek(0);
    CBeacon dst;
    {
      auto ar = mrpt::serialization::archiveFrom(buf);
      ar >> dst;
    }
    return dst;
  };

  CBeacon gauss = makeGaussianBeacon(7, 1.0, 2.0, 3.0);
  CBeacon dstGauss = roundTrip(gauss);
  EXPECT_EQ(dstGauss.m_ID, 7u);
  EXPECT_EQ(dstGauss.m_typePDF, CBeacon::pdfGauss);
  EXPECT_NEAR(dstGauss.m_locationGauss.mean.x(), 1.0, 1e-9);

  CBeacon mc = makeMonteCarloBeacon(8, 5.0f, 0.0f, 0.0f);
  CBeacon dstMc = roundTrip(mc);
  EXPECT_EQ(dstMc.m_typePDF, CBeacon::pdfMonteCarlo);
  EXPECT_EQ(dstMc.m_locationMC.size(), mc.m_locationMC.size());

  CBeacon sog = makeSOGBeacon(9, map);
  CBeacon dstSog = roundTrip(sog);
  EXPECT_EQ(dstSog.m_typePDF, CBeacon::pdfSOG);
  EXPECT_EQ(dstSog.m_locationSOG.size(), sog.m_locationSOG.size());
}
