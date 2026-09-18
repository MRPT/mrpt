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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

template class mrpt::CTraitsTest<mrpt::img::TStereoCamera>;

using namespace mrpt::img;

namespace
{
TStereoCamera makeSampleStereoCamera()
{
  TStereoCamera s;
  s.leftCamera.ncols = 640;
  s.leftCamera.nrows = 480;
  s.leftCamera.setIntrinsicParamsFromValues(500, 500, 320, 240);
  s.rightCamera.ncols = 640;
  s.rightCamera.nrows = 480;
  s.rightCamera.setIntrinsicParamsFromValues(500, 500, 320, 240);
  s.rightCameraPose = mrpt::math::TPose3DQuat(0.12, 0, 0, 1, 0, 0, 0);
  return s;
}
}  // namespace

TEST(TStereoCamera, SaveLoadConfigFileRoundTrip)
{
  const auto s1 = makeSampleStereoCamera();

  mrpt::config::CConfigFileMemory cfg;
  s1.saveToConfigFile("STEREO", cfg);

  TStereoCamera s2;
  s2.loadFromConfigFile("STEREO", cfg);

  EXPECT_EQ(s1.leftCamera.ncols, s2.leftCamera.ncols);
  EXPECT_EQ(s1.leftCamera.nrows, s2.leftCamera.nrows);
  EXPECT_EQ(s1.leftCamera.fx(), s2.leftCamera.fx());
  EXPECT_EQ(s1.rightCamera.fx(), s2.rightCamera.fx());
  EXPECT_NEAR(s1.rightCameraPose.x, s2.rightCameraPose.x, 1e-9);
}

TEST(TStereoCamera, LoadFromConfigFileOverloadOrderIsConsistent)
{
  const auto s1 = makeSampleStereoCamera();

  mrpt::config::CConfigFileMemory cfg;
  s1.saveToConfigFile("STEREO", cfg);

  TStereoCamera s2;
  // Alternative overload: (cfg, section)
  s2.loadFromConfigFile(cfg, "STEREO");

  EXPECT_EQ(s1.leftCamera.ncols, s2.leftCamera.ncols);
}

TEST(TStereoCamera, DumpAsTextContainsSections)
{
  const auto s = makeSampleStereoCamera();
  const std::string txt = s.dumpAsText();

  EXPECT_NE(txt.find("_LEFT"), std::string::npos);
  EXPECT_NE(txt.find("_RIGHT"), std::string::npos);
  EXPECT_NE(txt.find("_LEFT2RIGHT_POSE"), std::string::npos);
}

TEST(TStereoCamera, ScaleToResolution)
{
  auto s = makeSampleStereoCamera();
  s.scaleToResolution(1280, 960);

  EXPECT_EQ(s.leftCamera.ncols, 1280U);
  EXPECT_EQ(s.leftCamera.nrows, 960U);
  EXPECT_EQ(s.rightCamera.ncols, 1280U);
  EXPECT_EQ(s.rightCamera.nrows, 960U);
  EXPECT_NEAR(s.leftCamera.fx(), 1000.0, 1e-6);
}

TEST(TStereoCamera, BinarySerializationRoundTrip)
{
  const auto s1 = makeSampleStereoCamera();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << s1;

  buf.Seek(0);
  TStereoCamera s2;
  arch >> s2;

  EXPECT_EQ(s1.leftCamera.ncols, s2.leftCamera.ncols);
  EXPECT_EQ(s1.rightCamera.ncols, s2.rightCamera.ncols);
  EXPECT_NEAR(s1.rightCameraPose.x, s2.rightCameraPose.x, 1e-9);
}

TEST(TStereoCamera, DefaultConstructed)
{
  TStereoCamera s;
  // Just check it doesn't crash and produces some text.
  EXPECT_FALSE(s.dumpAsText().empty());
}
