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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/obs/TPixelLabelInfo.h>
#include <mrpt/serialization/CArchive.h>

#include <memory>
#include <sstream>

using namespace mrpt::obs;

namespace
{
template <unsigned int N>
void testPixelLabelInfoRoundtrip()
{
  TPixelLabelInfo<N> info;
  info.setSize(2, 3);
  info.setLabelName(0, "floor");
  info.setLabelName(1, "wall");

  info.setLabel(0, 0, 0);
  info.setLabel(0, 0, 1);
  EXPECT_TRUE(info.checkLabel(0, 0, 0));
  EXPECT_TRUE(info.checkLabel(0, 0, 1));

  uint8_t labels = 0;
  info.getLabels(0, 0, labels);
  EXPECT_NE(labels, 0);

  info.unsetLabel(0, 0, 1);
  EXPECT_FALSE(info.checkLabel(0, 0, 1));
  EXPECT_TRUE(info.checkLabel(0, 0, 0));

  info.unsetAll(0, 0, 0);
  EXPECT_FALSE(info.checkLabel(0, 0, 0));

  info.setLabel(1, 2, 0);

  EXPECT_EQ(info.getLabelName(0), "floor");
  EXPECT_THROW(info.getLabelName(99), std::exception);
  EXPECT_EQ(info.checkLabelNameExistence("wall"), 1);
  EXPECT_EQ(info.checkLabelNameExistence("nonexistent"), -1);

  std::stringstream ss;
  ss << info;  // exercises Print()
  EXPECT_FALSE(ss.str().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  info.writeToStream(arch);
  buf.Seek(0);

  std::unique_ptr<TPixelLabelInfoBase> readBack(TPixelLabelInfoBase::readAndBuildFromStream(arch));
  ASSERT_TRUE(readBack);
  EXPECT_EQ(readBack->BITFIELD_BYTES, N);
  EXPECT_TRUE(readBack->checkLabel(1, 2, 0));
  EXPECT_EQ(readBack->getLabelName(0), "floor");
}
}  // namespace

TEST(TPixelLabelInfo, Roundtrip1Byte) { testPixelLabelInfoRoundtrip<1>(); }
TEST(TPixelLabelInfo, Roundtrip2Bytes) { testPixelLabelInfoRoundtrip<2>(); }
TEST(TPixelLabelInfo, Roundtrip4Bytes) { testPixelLabelInfoRoundtrip<4>(); }
TEST(TPixelLabelInfo, Roundtrip8Bytes) { testPixelLabelInfoRoundtrip<8>(); }

TEST(TPixelLabelInfo, ReadAndBuildFromStreamUnknownBitfieldSizeThrows)
{
  // Manually craft a stream with version=1 and an invalid bitfield size:
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << static_cast<uint8_t>(1);    // version
  arch << static_cast<uint8_t>(200);  // invalid bitfield_bytes
  buf.Seek(0);

  EXPECT_THROW(TPixelLabelInfoBase::readAndBuildFromStream(arch), std::exception);
}

TEST(TPixelLabelInfo, ReadAndBuildFromStreamUnknownVersionThrows)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << static_cast<uint8_t>(200);  // invalid version
  buf.Seek(0);

  EXPECT_THROW(TPixelLabelInfoBase::readAndBuildFromStream(arch), std::exception);
}
