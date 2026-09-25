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
#include <mrpt/img/TColor.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/system/filesystem.h>

#include <cstdint>
#include <fstream>
#include <ios>
#include <vector>

using mrpt::img::CImage;
using mrpt::img::PixelDepth;

namespace
{
// Minimal PNG files, written by hand so the tests cover formats stb_image_write cannot produce.
// 3x2 RGB, 8 bit: pixel(x,y) = (10+x, 20+y, 30+x+y)
const std::vector<uint8_t> kPngRGB8_3x2 = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44,
    0x52, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x08, 0x02, 0x00, 0x00, 0x00, 0x12,
    0x16, 0xf1, 0x4d, 0x00, 0x00, 0x00, 0x1c, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0xe0,
    0x12, 0x91, 0xe3, 0x16, 0x91, 0xe7, 0x11, 0x51, 0x60, 0xe0, 0x12, 0x95, 0xe7, 0x16, 0x55,
    0xe0, 0x11, 0x55, 0x04, 0x00, 0x0e, 0x27, 0x01, 0x7b, 0xfc, 0x1e, 0x83, 0xdc, 0x00, 0x00,
    0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};
// 3x2 gray+alpha, 8 bit: gray = 50+x+10y, alpha = 200+x
const std::vector<uint8_t> kPngGrayAlpha8_3x2 = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x08, 0x04, 0x00, 0x00, 0x00, 0x37, 0x7d, 0xae,
    0x91, 0x00, 0x00, 0x00, 0x16, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0x30, 0x3a, 0x61, 0x7c,
    0xd2, 0xe4, 0x14, 0x83, 0xcd, 0x09, 0xdb, 0x93, 0x76, 0xa7, 0x00, 0x28, 0x0c, 0x06, 0x07, 0xef,
    0x6d, 0x8d, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};
// 3x2 gray, 16 bit: value = 1000*(x+1) + 300*y
const std::vector<uint8_t> kPngGray16_3x2 = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x8f, 0xe5,
    0x85, 0x00, 0x00, 0x00, 0x16, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0x60, 0x7e, 0xc1, 0x7e,
    0x81, 0x7b, 0x07, 0x03, 0xab, 0x08, 0xc7, 0x1f, 0x9e, 0x27, 0x00, 0x1e, 0x37, 0x04, 0x93, 0x59,
    0x20, 0x1f, 0x73, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};
}  // namespace

TEST(CImage, LoadFromEncodedBufferRGB8)
{
  CImage img;
  ASSERT_TRUE(img.loadFromEncodedBuffer(kPngRGB8_3x2.data(), kPngRGB8_3x2.size()));
  EXPECT_EQ(img.getWidth(), 3);
  EXPECT_EQ(img.getHeight(), 2);
  EXPECT_EQ(img.channels(), mrpt::img::CH_RGB);
  EXPECT_EQ(img.getPixelDepth(), PixelDepth::D8U);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      EXPECT_EQ(img.at<uint8_t>(x, y, 0), 10 + x);
      EXPECT_EQ(img.at<uint8_t>(x, y, 1), 20 + y);
      EXPECT_EQ(img.at<uint8_t>(x, y, 2), 30 + x + y);
    }
  }

  // Forced channel count:
  CImage gray;
  ASSERT_TRUE(
      gray.loadFromEncodedBuffer(kPngRGB8_3x2.data(), kPngRGB8_3x2.size(), mrpt::img::CH_GRAY));
  EXPECT_EQ(gray.channels(), mrpt::img::CH_GRAY);
}

TEST(CImage, LoadFromEncodedBufferGrayAlphaIsExpandedToRGBA)
{
  CImage img;
  ASSERT_TRUE(img.loadFromEncodedBuffer(kPngGrayAlpha8_3x2.data(), kPngGrayAlpha8_3x2.size()));
  EXPECT_EQ(img.channels(), mrpt::img::CH_RGBA);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      for (int8_t ch = 0; ch < 3; ch++)
      {
        EXPECT_EQ(img.at<uint8_t>(x, y, ch), 50 + x + 10 * y);
      }
      EXPECT_EQ(img.at<uint8_t>(x, y, 3), 200 + x);
    }
  }

  CImage gray;
  ASSERT_TRUE(gray.loadFromEncodedBuffer(
      kPngGrayAlpha8_3x2.data(), kPngGrayAlpha8_3x2.size(), mrpt::img::CH_GRAY));
  EXPECT_EQ(gray.channels(), mrpt::img::CH_GRAY);
  EXPECT_EQ(gray.at<uint8_t>(2, 1), 50 + 2 + 10);
}

TEST(CImage, LoadFromFileGrayAlphaIsExpandedToRGBA)
{
  const auto f = mrpt::system::getTempFileName() + ".png";
  {
    std::ofstream out(f, std::ios::binary);
    out.write(
        reinterpret_cast<const char*>(kPngGrayAlpha8_3x2.data()),  // NOLINT
        static_cast<std::streamsize>(kPngGrayAlpha8_3x2.size()));
  }
  CImage img;
  ASSERT_TRUE(img.loadFromFile(f));
  EXPECT_EQ(img.channels(), mrpt::img::CH_RGBA);
  EXPECT_EQ(img.at<uint8_t>(1, 1, 3), 201);
}

TEST(CImage, LoadFromEncodedBufferKeeps16bit)
{
  CImage img;
  ASSERT_TRUE(img.loadFromEncodedBuffer(kPngGray16_3x2.data(), kPngGray16_3x2.size()));
  EXPECT_EQ(img.channels(), mrpt::img::CH_GRAY);
  EXPECT_EQ(img.getPixelDepth(), PixelDepth::D16U);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      EXPECT_EQ(img.ptrLine<uint16_t>(y)[x], 1000 * (x + 1) + 300 * y);
    }
  }
}

TEST(CImage, LoadFromEncodedBufferForcedDepth)
{
  CImage narrow;
  ASSERT_TRUE(narrow.loadFromEncodedBuffer(
      kPngGray16_3x2.data(), kPngGray16_3x2.size(), mrpt::img::CH_AS_IS, PixelDepth::D8U));
  EXPECT_EQ(narrow.getPixelDepth(), PixelDepth::D8U);
  EXPECT_EQ(narrow.at<uint8_t>(2, 1), (3000 + 300) >> 8);

  CImage wide;
  ASSERT_TRUE(wide.loadFromEncodedBuffer(
      kPngRGB8_3x2.data(), kPngRGB8_3x2.size(), mrpt::img::CH_AS_IS, PixelDepth::D16U));
  EXPECT_EQ(wide.getPixelDepth(), PixelDepth::D16U);
  EXPECT_EQ(wide.channels(), mrpt::img::CH_RGB);
  // 8-bit values are scaled to the full 16-bit range:
  EXPECT_EQ(wide.ptrLine<uint16_t>(1)[3 * 2 + 2], (30 + 2 + 1) * 257);
}

TEST(CImage, LoadFromStreamIs8bit)
{
  mrpt::io::CMemoryStream buf;
  buf.Write(kPngGray16_3x2.data(), kPngGray16_3x2.size());
  buf.Seek(0);

  CImage img;
  img.loadFromStreamAsJPEG(buf);
  EXPECT_EQ(img.getPixelDepth(), PixelDepth::D8U);
  EXPECT_EQ(img.at<uint8_t>(2, 1), (3000 + 300) >> 8);
}

TEST(CImage, LoadFromEncodedBufferJPEG)
{
  CImage src(16, 12, mrpt::img::CH_RGB);
  src.filledRectangle({0, 0}, {15, 11}, mrpt::img::TColor(200, 100, 50));

  mrpt::io::CMemoryStream buf;
  src.saveToStreamAsJPEG(buf, 95);

  CImage img;
  ASSERT_TRUE(img.loadFromEncodedBuffer(
      static_cast<const uint8_t*>(buf.getRawBufferData()), buf.getTotalBytesCount()));
  EXPECT_EQ(img.getWidth(), 16);
  EXPECT_EQ(img.getHeight(), 12);
  EXPECT_EQ(img.channels(), mrpt::img::CH_RGB);
  // Lossy, but channels must not be swapped:
  EXPECT_NEAR(img.at<uint8_t>(8, 6, 0), 200, 4);
  EXPECT_NEAR(img.at<uint8_t>(8, 6, 1), 100, 4);
  EXPECT_NEAR(img.at<uint8_t>(8, 6, 2), 50, 4);
}

TEST(CImage, LoadFromEncodedBufferInvalidDataFails)
{
  CImage img(4, 4, mrpt::img::CH_RGB);
  const std::vector<uint8_t> garbage = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(img.loadFromEncodedBuffer(garbage.data(), garbage.size()));
  EXPECT_TRUE(img.isEmpty());

  EXPECT_FALSE(img.loadFromEncodedBuffer(nullptr, 10));
  EXPECT_FALSE(img.loadFromEncodedBuffer(garbage.data(), 0));

  // Truncated file:
  EXPECT_FALSE(img.loadFromEncodedBuffer(kPngRGB8_3x2.data(), kPngRGB8_3x2.size() / 2));
}
