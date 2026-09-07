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
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>

#include <chrono>
#include <thread>

#include "mock_stream.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::MockStream;

namespace mrpt::hwdrivers
{
/** A minimal 2D scanner driver: it returns whatever scan the test stages,
 *  so that the generic machinery of the base class can be exercised without
 *  any real device protocol in the way. */
class FakeLaser : public C2DRangeFinderAbstract
{
  DEFINE_GENERIC_SENSOR(FakeLaser)

 public:
  FakeLaser() = default;

  void doProcessSimple(
      bool& outThereIsObservation,
      mrpt::obs::CObservation2DRangeScan& outObservation,
      bool& hardwareError) override
  {
    hardwareError = m_stagedHwError;
    outThereIsObservation = m_stagedThereIs;
    if (m_stagedThereIs)
    {
      outObservation = m_stagedScan;
      // Exercise the base class filters, as real drivers do:
      filterByExclusionAreas(outObservation);
      filterByExclusionAngles(outObservation);
      internal_notifyGoodScanNow();
    }
    else
    {
      m_lastNoScanWasOk = internal_notifyNoScanReceived();
    }
  }

  bool turnOn() override { return true; }
  bool turnOff() override { return true; }

  void stageScan(const mrpt::obs::CObservation2DRangeScan& s)
  {
    m_stagedScan = s;
    m_stagedThereIs = true;
    m_stagedHwError = false;
  }
  void stageNothing() { m_stagedThereIs = false; }
  void stageHardwareError()
  {
    m_stagedThereIs = false;
    m_stagedHwError = true;
  }
  bool lastNoScanWasOk() const { return m_lastNoScanWasOk; }

  // Expose the protected config loader for the tests:
  void loadCommon(const mrpt::config::CConfigFileBase& c, const std::string& s)
  {
    loadCommonParams(c, s);
  }

 protected:
  void loadConfig_sensorSpecific(
      const mrpt::config::CConfigFileBase& c, const std::string& s) override
  {
    loadCommonParams(c, s);
  }

 private:
  mrpt::obs::CObservation2DRangeScan m_stagedScan;
  bool m_stagedThereIs = false;
  bool m_stagedHwError = false;
  bool m_lastNoScanWasOk = true;
};
}  // namespace mrpt::hwdrivers

IMPLEMENTS_GENERIC_SENSOR(FakeLaser, mrpt::hwdrivers)

namespace
{
using mrpt::hwdrivers::FakeLaser;

/** A scan of `n` rays spanning 180 deg, all valid at `range` meters. */
mrpt::obs::CObservation2DRangeScan makeScan(size_t n, float range)
{
  mrpt::obs::CObservation2DRangeScan scan;
  scan.aperture = M_PIf;
  scan.rightToLeft = true;
  scan.maxRange = 30.0f;
  scan.sensorPose = mrpt::poses::CPose3D();
  scan.resizeScan(n);
  for (size_t i = 0; i < n; i++)
  {
    scan.setScanRange(i, range);
    scan.setScanRangeValidity(i, true);
  }
  return scan;
}
}  // namespace

TEST(C2DRangeFinderAbstract, bindIOAndGetObservation)
{
  FakeLaser laser;
  auto s = std::make_shared<MockStream>();
  laser.bindIO(s);

  // Nothing staged yet -> nothing to hand out:
  bool thereIs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.getObservation(thereIs, obs, hwError);
  EXPECT_FALSE(thereIs);

  laser.stageScan(makeScan(10, 2.0f));
  laser.doProcess();

  laser.getObservation(thereIs, obs, hwError);
  EXPECT_FALSE(hwError);
  ASSERT_TRUE(thereIs);
  EXPECT_EQ(obs.getScanSize(), 10U);

  // getObservation() consumes it: a second call finds nothing new.
  laser.getObservation(thereIs, obs, hwError);
  EXPECT_FALSE(thereIs);
}

TEST(C2DRangeFinderAbstract, doProcessQueuesObservations)
{
  FakeLaser laser;
  laser.stageScan(makeScan(4, 1.0f));

  laser.doProcess();
  laser.doProcess();

  auto obss = laser.getObservations();
  EXPECT_EQ(obss.size(), 2U);
  EXPECT_EQ(laser.getState(), CGenericSensor::ssWorking);
}

TEST(C2DRangeFinderAbstract, hardwareErrorSetsErrorState)
{
  FakeLaser laser;
  laser.stageHardwareError();

  laser.doProcess();

  EXPECT_EQ(laser.getState(), CGenericSensor::ssError);
  EXPECT_TRUE(laser.getObservations().empty());
}

TEST(C2DRangeFinderAbstract, repeatedMissingScansEventuallyReportAFailure)
{
  FakeLaser laser;
  const int maxMissed = 3;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LASER", "maxMissedScansToDeclareError", maxMissed);
  laser.loadCommon(cfg, "LASER");

  // Before any good scan, a missing scan is not yet an error:
  laser.stageNothing();
  laser.doProcess();
  EXPECT_TRUE(laser.lastNoScanWasOk());

  // Feed a burst of scans so the estimated scan period (which starts at 1 s)
  // converges down to the millisecond range this test can wait out:
  laser.stageScan(makeScan(4, 1.0f));
  for (int i = 0; i < 80; i++)
  {
    laser.doProcess();
  }
  const double period = laser.getEstimatedScanPeriod();
  EXPECT_GT(period, 0.0);
  EXPECT_LT(period, 0.01);

  // Now go quiet for clearly longer than that period, `maxMissed` times:
  laser.stageNothing();
  for (int i = 0; i < maxMissed; i++)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    laser.doProcess();
  }
  EXPECT_FALSE(laser.lastNoScanWasOk());
}

TEST(C2DRangeFinderAbstract, exclusionAreaInvalidatesEnclosedRays)
{
  FakeLaser laser;

  // A square covering everything in front of the sensor up to 5 m:
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LASER", "exclusionZone1_x", "[-5 5 5 -5]");
  cfg.write("LASER", "exclusionZone1_y", "[-5 -5 5 5]");
  laser.loadCommon(cfg, "LASER");

  // Rays at 2 m fall inside the zone; rays at 20 m do not.
  laser.stageScan(makeScan(20, 2.0f));
  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);
  ASSERT_TRUE(thereIs);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_FALSE(obs.getScanRangeValidity(i)) << "at i=" << i;
  }

  laser.stageScan(makeScan(20, 20.0f));
  laser.doProcessSimple(thereIs, obs, hwError);
  ASSERT_TRUE(thereIs);
  size_t nValid = 0;
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    if (obs.getScanRangeValidity(i))
    {
      nValid++;
    }
  }
  EXPECT_GT(nValid, 0U);
}

TEST(C2DRangeFinderAbstract, exclusionAreaWithZRangeIsParsed)
{
  FakeLaser laser;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LASER", "exclusionZone1_x", "[-5 5 5 -5]");
  cfg.write("LASER", "exclusionZone1_y", "[-5 -5 5 5]");
  cfg.write("LASER", "exclusionZone1_z", "[-1.0 1.0]");
  EXPECT_NO_THROW(laser.loadCommon(cfg, "LASER"));

  // A malformed z range must be rejected:
  mrpt::config::CConfigFileMemory bad;
  bad.write("LASER", "exclusionZone1_x", "[-5 5 5 -5]");
  bad.write("LASER", "exclusionZone1_y", "[-5 -5 5 5]");
  bad.write("LASER", "exclusionZone1_z", "[1.0 -1.0]");
  EXPECT_THROW(laser.loadCommon(bad, "LASER"), std::exception);
}

TEST(C2DRangeFinderAbstract, exclusionAnglesInvalidateTheirSector)
{
  FakeLaser laser;
  // The scan spans [-90, +90] deg; block the left half:
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LASER", "exclusionAngles1_ini", 0.0);
  cfg.write("LASER", "exclusionAngles1_end", 90.0);
  laser.loadCommon(cfg, "LASER");

  laser.stageScan(makeScan(180, 10.0f));

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);
  ASSERT_TRUE(thereIs);

  size_t nValid = 0;
  size_t nInvalid = 0;
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    if (obs.getScanRangeValidity(i))
    {
      nValid++;
    }
    else
    {
      nInvalid++;
    }
  }
  // Roughly half the scan must have been discarded:
  EXPECT_GT(nInvalid, 0U);
  EXPECT_GT(nValid, 0U);
}

TEST(C2DRangeFinderAbstract, multipleExclusionZonesAreAllLoaded)
{
  FakeLaser laser;
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LASER", "exclusionZone1_x", "[-1 1 1 -1]");
  cfg.write("LASER", "exclusionZone1_y", "[-1 -1 1 1]");
  cfg.write("LASER", "exclusionZone2_x", "[3 5 5 3]");
  cfg.write("LASER", "exclusionZone2_y", "[3 3 5 5]");
  cfg.write("LASER", "exclusionAngles1_ini", -10.0);
  cfg.write("LASER", "exclusionAngles1_end", 10.0);
  cfg.write("LASER", "exclusionAngles2_ini", 20.0);
  cfg.write("LASER", "exclusionAngles2_end", 30.0);

  EXPECT_NO_THROW(laser.loadCommon(cfg, "LASER"));

  // Mismatched vertex counts are an error:
  mrpt::config::CConfigFileMemory bad;
  bad.write("LASER", "exclusionZone1_x", "[-1 1 1 -1]");
  bad.write("LASER", "exclusionZone1_y", "[-1 -1 1]");
  EXPECT_THROW(laser.loadCommon(bad, "LASER"), std::exception);
}
