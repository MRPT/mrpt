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

// Black-box tests for the `rawlog-edit` CLI tool: they invoke the built
// executable as a subprocess and check its exit code and/or output, using
// the small sample datasets in mrpt_data/datasets/.

#include <gtest/gtest.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <test_mrpt_common.h>

#include <fstream>
#include <string>
#include <vector>

namespace
{
// Locates the `rawlog-edit` binary, either in the colcon build/install tree
// (next to this test's source tree) or in the PATH.
std::string findRawlogEditBinary()
{
#ifdef _WIN32
  const std::string exeName = "rawlog-edit.exe";
#else
  const std::string exeName = "rawlog-edit";
#endif

  const std::string base = mrpt::UNITTEST_BASEDIR();
  const std::vector<std::string> candidates = {
      base + "/../../install/mrpt_apps_cli/bin/" + exeName,
      base + "/../../build/mrpt_apps_cli/bin/" + exeName,
  };
  for (const auto& c : candidates)
  {
    if (mrpt::system::fileExists(c))
    {
      return c;
    }
  }

  // Fallback: search the PATH.
  std::string out;
#ifdef _WIN32
  const std::string findCmd = "where " + exeName;
#else
  const std::string findCmd = "command -v " + exeName;
#endif
  if (mrpt::system::executeCommand(findCmd, &out) == 0 && !out.empty())
  {
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
    {
      out.pop_back();
    }
    if (!out.empty())
    {
      return out;
    }
  }
  return {};
}

struct CmdResult
{
  int exitCode = -1;
  std::string output;
};

}  // namespace

class RawlogEditCLITest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    m_bin = findRawlogEditBinary();
    if (m_bin.empty())
    {
      GTEST_SKIP() << "rawlog-edit binary not found, skipping CLI tests.";
    }

    m_tempDir = mrpt::system::getTempFileName() + "_rawlog_edit_test";
    ASSERT_(mrpt::system::createDirectory(m_tempDir));
  }

  void TearDown() override
  {
    if (!m_tempDir.empty())
    {
      mrpt::system::deleteFilesInDirectory(m_tempDir, true /*and the dir*/);
    }
  }

  // Copies a dataset file from mrpt_data/datasets/ into the (writable) temp
  // dir, and returns the path to the copy.
  [[nodiscard]] std::string datasetCopy(const std::string& name) const
  {
    const std::string src = mrpt::system::pathJoin({mrpt::mrpt_data_dir(), "datasets", name});
    ASSERT_FILE_EXISTS_(src);

    const std::string dst = mrpt::system::pathJoin({m_tempDir, name});
    ASSERT_(mrpt::system::copyFile(src, dst));
    return dst;
  }

  // Runs rawlog-edit with the given arguments, capturing stdout+stderr.
  // Runs the binary with the current working directory set to m_tempDir,
  // since some operations (--export-gps-all, --externalize, ...) write
  // additional output files relative to the current directory.
  //
  // On Windows, stderr is already merged into the captured stdout pipe by
  // executeCommand(), so "2>&1" (which would otherwise be passed as a
  // literal argument to the program, since there is no shell involved) is
  // only added on Unix.
  [[nodiscard]] CmdResult run(const std::vector<std::string>& args) const
  {
    std::string cmd = "\"" + m_bin + "\"";
    for (const auto& a : args)
    {
      cmd += " \"" + a + "\"";
    }
#ifndef _WIN32
    cmd += " 2>&1";
#endif

    CmdResult r;
    r.exitCode = mrpt::system::executeCommand(cmd, &r.output, "r", m_tempDir);
    return r;
  }

  std::string m_bin;
  std::string m_tempDir;
};

// ===========================================================================
// Generic CLI behavior
// ===========================================================================

TEST_F(RawlogEditCLITest, Help_PrintsUsage)
{
  const auto r = run({"--help"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_NE(r.output.find("rawlog-edit"), std::string::npos);
}

TEST_F(RawlogEditCLITest, Version_PrintsVersion)
{
  const auto r = run({"--version"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, NoOperationSelected_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"-i", in, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, TwoOperationsSelected_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"-i", in, "--info", "--describe", "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, MissingInputFile_Fails)
{
  const auto r = run({"--info", "-i", m_tempDir + "/does_not_exist.rawlog", "-q"});
  EXPECT_NE(r.exitCode, 0);
}

// ===========================================================================
// --info / --describe
// ===========================================================================

TEST_F(RawlogEditCLITest, Info_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--info", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_NE(r.output.find("Overall number of objects"), std::string::npos);
  EXPECT_NE(r.output.find("GPS_RTK_FRONT_L"), std::string::npos);
  EXPECT_NE(r.output.find("GPS_RTK_FRONT_R"), std::string::npos);
  EXPECT_NE(r.output.find("GPS_RTK_REAR"), std::string::npos);
}

TEST_F(RawlogEditCLITest, Info_RotScan2DWithParseWarning)
{
  // This small dataset triggers an internal "EOF?" parsing exception that
  // is caught and reported, but the overall command must still succeed.
  const auto in = datasetCopy("rot_scan_2d.rawlog");
  const auto r = run({"--info", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_NE(r.output.find("Physical file size"), std::string::npos);
}

TEST_F(RawlogEditCLITest, Describe_LocalizationDemo)
{
  const auto in = datasetCopy("localization_demo.rawlog");
  const auto r = run({"--describe", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_NE(r.output.find("CObservation2DRangeScan"), std::string::npos);
}

// ===========================================================================
// --list-* operations
// ===========================================================================

TEST_F(RawlogEditCLITest, ListTimestamps_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/timestamps.txt";
  const auto r = run({"--list-timestamps", "-i", in, "--text-file-output", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));

  std::ifstream f(out);
  ASSERT_TRUE(f.is_open());
  std::string line;
  size_t nLines = 0;
  while (std::getline(f, line))
  {
    nLines++;
  }
  EXPECT_GT(nLines, 1U);
}

TEST_F(RawlogEditCLITest, ListPoses_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/poses.txt";
  const auto r = run({"--list-poses", "-i", in, "--text-file-output", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

TEST_F(RawlogEditCLITest, ListImages_LocalizationDemo)
{
  const auto in = datasetCopy("localization_demo.rawlog");
  const auto out = m_tempDir + "/images.txt";
  const auto r = run({"--list-images", "-i", in, "--text-file-output", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

TEST_F(RawlogEditCLITest, ListRangeBearing_KfSlamDemo)
{
  const auto in = datasetCopy("kf-slam_demo.rawlog");
  const auto out = m_tempDir + "/rangebearing.txt";
  const auto r = run({"--list-range-bearing", "-i", in, "--text-file-output", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));

  std::ifstream f(out);
  ASSERT_TRUE(f.is_open());
  std::string line;
  size_t nLines = 0;
  while (std::getline(f, line))
  {
    nLines++;
  }
  // Header (2 lines) + at least one data row:
  EXPECT_GT(nLines, 2U);
}

// ===========================================================================
// --cut
// ===========================================================================

TEST_F(RawlogEditCLITest, Cut_NoArgs_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/cut.rawlog";
  const auto r = run({"--cut", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, Cut_FromToIndex_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/cut.rawlog";
  const auto r =
      run({"--cut", "--from-index", "100", "--to-index", "200", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  ASSERT_TRUE(mrpt::system::fileExists(out));

  const auto info = run({"--info", "-i", out, "-q"});
  EXPECT_EQ(info.exitCode, 0);
  // 200 - 100 + 1 = 101 entries kept.
  EXPECT_NE(info.output.find("Overall number of objects         : 101"), std::string::npos);
}

// ===========================================================================
// --export-* operations
// ===========================================================================

TEST_F(RawlogEditCLITest, ExportTxt_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-txt", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, Export2DScansTxt_EmptyLabel_Fails)
{
  // localization_demo.rawlog has 2D range scans with an empty sensorLabel,
  // which the generic TXT exporter rejects.
  const auto in = datasetCopy("localization_demo.rawlog");
  const auto r = run({"--export-2d-scans-txt", "-i", in, "-q"});
  EXPECT_NE(r.exitCode, 0);
  EXPECT_NE(r.output.find("non-empty sensorLabels"), std::string::npos);
}

TEST_F(RawlogEditCLITest, ExportGpsAll_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-gps-all", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(
      mrpt::system::fileExists(m_tempDir + "/test_rtk_path_GPS_RTK_FRONT_L_MSG_NMEA_GGA.txt"));
}

TEST_F(RawlogEditCLITest, ExportGpsKml_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-gps-kml", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(m_tempDir + "/test_rtk_path.kml"));
}

TEST_F(RawlogEditCLITest, ExportGpsGasKml_NoMatchingObservations)
{
  // No gas sensor present: must not fail, simply produce no output.
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-gps-gas-kml", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, ExportImuTxt_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-imu-txt", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, ExportAnemometerTxt_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-anemometer-txt", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, ExportEnoseTxt_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-enose-txt", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, ExportRawDaqTxt_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto r = run({"--export-rawdaq-txt", "-i", in, "-q"});
  EXPECT_EQ(r.exitCode, 0);
}

// ===========================================================================
// --remove-label / --keep-label
// ===========================================================================

TEST_F(RawlogEditCLITest, RemoveLabel_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/removed.rawlog";
  const auto r = run({"--remove-label", "GPS_RTK_FRONT_L", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);

  const auto info = run({"--info", "-i", out, "-q"});
  EXPECT_EQ(info.exitCode, 0);
  EXPECT_EQ(info.output.find("GPS_RTK_FRONT_L"), std::string::npos);
  EXPECT_NE(info.output.find("GPS_RTK_FRONT_R"), std::string::npos);
  EXPECT_NE(info.output.find("GPS_RTK_REAR"), std::string::npos);
}

TEST_F(RawlogEditCLITest, RemoveLabel_EmptyArg_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/removed.rawlog";
  const auto r = run({"--remove-label", "", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, KeepLabel_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/kept.rawlog";
  const auto r = run({"--keep-label", "GPS_RTK_REAR", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);

  const auto info = run({"--info", "-i", out, "-q"});
  EXPECT_EQ(info.exitCode, 0);
  EXPECT_EQ(info.output.find("GPS_RTK_FRONT_L"), std::string::npos);
  EXPECT_EQ(info.output.find("GPS_RTK_FRONT_R"), std::string::npos);
  EXPECT_NE(info.output.find("GPS_RTK_REAR"), std::string::npos);
  // Only the GPS_RTK_REAR observations (77) should remain.
  EXPECT_NE(info.output.find("Overall number of objects         : 77"), std::string::npos);
}

// ===========================================================================
// --remap-timestamps
// ===========================================================================

TEST_F(RawlogEditCLITest, RemapTimestamps_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/remapped.rawlog";
  const auto r = run({"--remap-timestamps", "1.0;1000000000", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);

  const auto info = run({"--info", "-i", out, "-q"});
  EXPECT_EQ(info.exitCode, 0);
  // Original earliest timestamp is 1226225355.0, so the new one must be
  // 1226225355.0 + 1e9 = 2226225355.0
  EXPECT_NE(info.output.find("Earliest timestamp                : 2226225355"), std::string::npos);
}

TEST_F(RawlogEditCLITest, RemapTimestamps_BadFormat_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/remapped.rawlog";
  const auto r = run({"--remap-timestamps", "not-a-valid-expression", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

// ===========================================================================
// --rename-externals
// ===========================================================================

TEST_F(RawlogEditCLITest, RenameExternals_NoExternalFiles)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/renamed.rawlog";
  const auto r = run({"--rename-externals", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

// ===========================================================================
// --sensors-pose
// ===========================================================================

TEST_F(RawlogEditCLITest, SensorsPose_TestRtkPath)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/sensorpose.rawlog";

  const auto iniFile = m_tempDir + "/sensor_poses.ini";
  {
    std::ofstream f(iniFile);
    ASSERT_TRUE(f.is_open());
    f << "[FRONT_L]\n"
      << "sensorLabel=GPS_RTK_FRONT_L\n"
      << "pose_x=1.0\n"
      << "pose_y=2.0\n"
      << "pose_z=0.5\n"
      << "pose_yaw=0\n"
      << "pose_pitch=0\n"
      << "pose_roll=0\n";
  }

  const auto r = run({"--sensors-pose", iniFile, "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

TEST_F(RawlogEditCLITest, SensorsPose_MissingFile_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/sensorpose.rawlog";
  const auto r =
      run({"--sensors-pose", m_tempDir + "/does_not_exist.ini", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

// ===========================================================================
// --recalc-odometry
// ===========================================================================

TEST_F(RawlogEditCLITest, RecalcOdometry_MissingArgs_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/recalc.rawlog";
  const auto r = run({"--recalc-odometry", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, RecalcOdometry_NoOdometryEntries)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/recalc.rawlog";
  const auto r = run(
      {"--recalc-odometry", "--odo-KL", "0.1", "--odo-KR", "0.1", "--odo-D", "0.4", "-i", in, "-o",
       out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

// ===========================================================================
// --camera-params
// ===========================================================================

TEST_F(RawlogEditCLITest, CameraParams_BadFormat_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/camparams.rawlog";
  const auto r = run({"--camera-params", "JustOneToken", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r.exitCode, 0);
}

TEST_F(RawlogEditCLITest, CameraParams_MissingFile_Fails)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/camparams.rawlog";
  const auto r = run(
      {"--camera-params", "SOME_LABEL," + m_tempDir + "/does_not_exist.ini", "-i", in, "-o", out,
       "-q"});
  EXPECT_NE(r.exitCode, 0);
}

// ===========================================================================
// --externalize / --de-externalize
// ===========================================================================

TEST_F(RawlogEditCLITest, Externalize_NoConvertibleObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/externalized.rawlog";
  const auto r = run({"--externalize", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
  EXPECT_TRUE(mrpt::system::directoryExists(m_tempDir + "/externalized_Images"));
}

TEST_F(RawlogEditCLITest, DeExternalize_NoExternalReferences)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/deexternalized.rawlog";
  const auto r = run({"--de-externalize", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

// ===========================================================================
// --generate-3d-pointclouds / --undistort
// ===========================================================================

TEST_F(RawlogEditCLITest, GenerateThreeDPointClouds_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/pointclouds.rawlog";
  const auto r = run({"--generate-3d-pointclouds", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

TEST_F(RawlogEditCLITest, Undistort_NoMatchingObservations)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/undistorted.rawlog";
  const auto r = run({"--undistort", "-i", in, "-o", out, "-q"});
  EXPECT_EQ(r.exitCode, 0);
  EXPECT_TRUE(mrpt::system::fileExists(out));
}

// ===========================================================================
// Output file overwrite protection
// ===========================================================================

TEST_F(RawlogEditCLITest, OutputOverwrite_RequiresFlag)
{
  const auto in = datasetCopy("test_rtk_path.rawlog");
  const auto out = m_tempDir + "/overwrite.rawlog";

  const auto r1 = run({"--cut", "--from-index", "0", "-i", in, "-o", out, "-q", "-w"});
  EXPECT_EQ(r1.exitCode, 0);
  ASSERT_TRUE(mrpt::system::fileExists(out));

  // Without -w/--overwrite, re-running must fail since the output exists.
  const auto r2 = run({"--cut", "--from-index", "0", "-i", in, "-o", out, "-q"});
  EXPECT_NE(r2.exitCode, 0);

  // With -w/--overwrite, it must succeed.
  const auto r3 = run({"--cut", "--from-index", "0", "-i", in, "-o", out, "-q", "-w"});
  EXPECT_EQ(r3.exitCode, 0);
}
