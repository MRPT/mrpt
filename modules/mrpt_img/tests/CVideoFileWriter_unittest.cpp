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
#include <mrpt/img/CImage.h>
#include <mrpt/img/CVideoFileWriter.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::img;
using namespace std::string_literals;

namespace
{
CImage makeTestFrame(int w, int h, TImageChannels ch)
{
  CImage img(w, h, ch);
  for (int y = 0; y < h; y++)
  {
    auto* row = img.ptrLine<uint8_t>(y);
    for (int x = 0; x < w * static_cast<int>(ch); x++)
    {
      row[x] = static_cast<uint8_t>((x + y) % 256);
    }
  }
  return img;
}
}  // namespace

TEST(CVideoFileWriter, DefaultConstructedIsNotOpen)
{
  CVideoFileWriter vid;
  EXPECT_FALSE(vid.isOpen());
  EXPECT_EQ(vid.frameCount(), 0U);
}

TEST(CVideoFileWriter, WriteImageWhenNotOpenFails)
{
  CVideoFileWriter vid;
  const CImage img = makeTestFrame(8, 8, CH_RGB);
  EXPECT_FALSE(vid.writeImage(img));
  EXPECT_THROW(vid << img, std::exception);
}

TEST(CVideoFileWriter, OpenWriteCloseMJPEG_Color)
{
  const auto file = mrpt::system::getTempFileName() + "_mjpeg_color.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 25.0, {16, 12}, VideoCodec::MJPEG, 80));
  EXPECT_TRUE(vid.isOpen());
  EXPECT_EQ(vid.codec(), VideoCodec::MJPEG);
  EXPECT_EQ(vid.frameSize().x, 16);
  EXPECT_EQ(vid.frameSize().y, 12);

  const CImage frame = makeTestFrame(16, 12, CH_RGB);
  EXPECT_TRUE(vid.writeImage(frame));
  EXPECT_NO_THROW(vid << frame);
  EXPECT_EQ(vid.frameCount(), 2U);

  vid.close();
  EXPECT_FALSE(vid.isOpen());
  EXPECT_TRUE(mrpt::system::fileExists(file));
  // Calling close() again should be a safe no-op.
  vid.close();
}

TEST(CVideoFileWriter, OpenWriteCloseMJPEG_Gray)
{
  const auto file = mrpt::system::getTempFileName() + "_mjpeg_gray.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 15.0, {20, 10}, VideoCodec::MJPEG));

  const CImage frame = makeTestFrame(20, 10, CH_GRAY);
  EXPECT_TRUE(vid.writeImage(frame));
  vid.close();
  EXPECT_TRUE(mrpt::system::fileExists(file));
}

TEST(CVideoFileWriter, OpenWriteCloseUncompressed_Color)
{
  const auto file = mrpt::system::getTempFileName() + "_raw_color.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 30.0, {18, 9}, VideoCodec::UncompressedRGB));
  EXPECT_EQ(vid.codec(), VideoCodec::UncompressedRGB);

  const CImage frame = makeTestFrame(18, 9, CH_RGB);
  EXPECT_TRUE(vid.writeImage(frame));
  EXPECT_TRUE(vid.writeImage(frame));
  vid.close();
  EXPECT_TRUE(mrpt::system::fileExists(file));
}

TEST(CVideoFileWriter, OpenWriteCloseUncompressed_Gray)
{
  const auto file = mrpt::system::getTempFileName() + "_raw_gray.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 30.0, {13, 7}, VideoCodec::UncompressedRGB));

  const CImage frame = makeTestFrame(13, 7, CH_GRAY);
  EXPECT_TRUE(vid.writeImage(frame));
  vid.close();
  EXPECT_TRUE(mrpt::system::fileExists(file));
}

TEST(CVideoFileWriter, RGBAFrameMJPEG)
{
  const auto file = mrpt::system::getTempFileName() + "_mjpeg_rgba.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 25.0, {16, 16}, VideoCodec::MJPEG));

  const CImage frame = makeTestFrame(16, 16, CH_RGBA);
  EXPECT_TRUE(vid.writeImage(frame));
  vid.close();
}

TEST(CVideoFileWriter, MismatchedFrameSizeFails)
{
  const auto file = mrpt::system::getTempFileName() + "_size_mismatch.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 25.0, {16, 16}, VideoCodec::MJPEG));

  // First frame establishes the color mode; wrong size should be rejected.
  const CImage wrongSize = makeTestFrame(8, 8, CH_RGB);
  EXPECT_FALSE(vid.writeImage(wrongSize));
  vid.close();
}

TEST(CVideoFileWriter, MismatchedChannelsAfterFirstFrameFails)
{
  const auto file = mrpt::system::getTempFileName() + "_channel_mismatch.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file, 25.0, {16, 16}, VideoCodec::MJPEG));

  const CImage colorFrame = makeTestFrame(16, 16, CH_RGB);
  ASSERT_TRUE(vid.writeImage(colorFrame));

  const CImage grayFrame = makeTestFrame(16, 16, CH_GRAY);
  EXPECT_FALSE(vid.writeImage(grayFrame));
  vid.close();
}

TEST(CVideoFileWriter, InvalidPathFailsToOpen)
{
  CVideoFileWriter vid;
  EXPECT_FALSE(vid.open("/this/path/does/not/exist/video.avi", 25.0, {8, 8}));
}

TEST(CVideoFileWriter, MoveConstructAndAssign)
{
  const auto file = mrpt::system::getTempFileName() + "_move.avi"s;

  CVideoFileWriter vid1;
  ASSERT_TRUE(vid1.open(file, 25.0, {8, 8}, VideoCodec::MJPEG));

  CVideoFileWriter vid2(std::move(vid1));
  EXPECT_TRUE(vid2.isOpen());

  const CImage frame = makeTestFrame(8, 8, CH_RGB);
  EXPECT_TRUE(vid2.writeImage(frame));

  CVideoFileWriter vid3;
  vid3 = std::move(vid2);
  EXPECT_TRUE(vid3.isOpen());
  vid3.close();
}

TEST(CVideoFileWriter, ReopenClosesPreviousFile)
{
  const auto file1 = mrpt::system::getTempFileName() + "_reopen1.avi"s;
  const auto file2 = mrpt::system::getTempFileName() + "_reopen2.avi"s;

  CVideoFileWriter vid;
  ASSERT_TRUE(vid.open(file1, 25.0, {8, 8}, VideoCodec::MJPEG));
  ASSERT_TRUE(vid.open(file2, 25.0, {8, 8}, VideoCodec::MJPEG));
  EXPECT_TRUE(vid.isOpen());
  vid.close();

  EXPECT_TRUE(mrpt::system::fileExists(file1));
  EXPECT_TRUE(mrpt::system::fileExists(file2));
}
