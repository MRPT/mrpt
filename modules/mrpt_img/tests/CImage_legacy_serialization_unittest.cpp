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

// Exercises the MRPT 2.x backwards-compatibility branches of
// CImage::serializeFrom() (streaming versions 0..9 and 100), which no binary
// fixture file in the repo reaches.

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/serialization/CArchive.h>

#include <cstdint>
#include <vector>

#include "legacy_serialization.h"

using mrpt::img::CImage;
using mrpt::io::CMemoryStream;
using mrpt_test::writeLegacyObjectFrame;

namespace
{
CImage readBack(CMemoryStream& buf)
{
  auto arch = mrpt::serialization::archiveFrom(buf);
  CImage img;
  arch >> img;
  return img;
}

/** A small, deterministic RGB test image. */
CImage makeRGB(int w, int h)
{
  CImage img(w, h, mrpt::img::CH_RGB);
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      auto* p = img.ptr<uint8_t>(x, y);
      p[0] = static_cast<uint8_t>(10 * x);
      p[1] = static_cast<uint8_t>(20 * y);
      p[2] = static_cast<uint8_t>(30 + x + y);
    }
  }
  return img;
}

std::vector<uint8_t> asJPEG(const CImage& img)
{
  CMemoryStream s;
  img.saveToStreamAsJPEG(s, 95);
  const auto* p = static_cast<const uint8_t*>(s.getRawBufferData());
  return {p, p + s.getTotalBytesCount()};
}
}  // namespace

// v0: raw, uncompressed image data with explicit dimensions.
TEST(CImageLegacySerialization, Version0Grayscale)
{
  CMemoryStream buf;
  constexpr uint32_t W = 4;
  constexpr uint32_t H = 3;

  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 0,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << W << H << uint32_t(1) << uint8_t(1) << uint32_t(W * H);
        for (uint32_t i = 0; i < W * H; i++)
        {
          arch << static_cast<uint8_t>(i * 3);
        }
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 4);
  ASSERT_EQ(img.getHeight(), 3);
  EXPECT_FALSE(img.isColor());
  EXPECT_EQ(img.at<uint8_t>(0, 0), 0);
  EXPECT_EQ(img.at<uint8_t>(1, 0), 3);
  EXPECT_EQ(img.at<uint8_t>(3, 2), 33);
}

// v0, 3-channel: data was stored BGR and must be swapped to RGB on read.
TEST(CImageLegacySerialization, Version0ColorSwapsRedBlue)
{
  CMemoryStream buf;
  constexpr uint32_t W = 2;
  constexpr uint32_t H = 1;

  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 0,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << W << H << uint32_t(3) << uint8_t(1) << uint32_t(W * H * 3);
        // BGR pixels: (B,G,R)
        const uint8_t bgr[] = {1, 2, 3, 4, 5, 6};
        for (unsigned char v : bgr)
        {
          arch << v;
        }
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 2);
  EXPECT_TRUE(img.isColor());
  EXPECT_EQ(img.ptr<uint8_t>(0, 0)[0], 3);  // R
  EXPECT_EQ(img.ptr<uint8_t>(0, 0)[1], 2);  // G
  EXPECT_EQ(img.ptr<uint8_t>(0, 0)[2], 1);  // B
  EXPECT_EQ(img.ptr<uint8_t>(1, 0)[0], 6);
}

// v1: the whole image as a JPEG blob.
TEST(CImageLegacySerialization, Version1Jpeg)
{
  const auto jpeg = asJPEG(makeRGB(16, 8));

  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 1,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << static_cast<uint32_t>(jpeg.size());
        arch.WriteBuffer(jpeg.data(), jpeg.size());
      });

  const CImage img = readBack(buf);
  EXPECT_EQ(img.getWidth(), 16);
  EXPECT_EQ(img.getHeight(), 8);
  EXPECT_TRUE(img.isColor());
}

TEST(CImageLegacySerialization, Version1CorruptJpegThrows)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 1,
      [&](mrpt::serialization::CArchive& arch)
      {
        const std::vector<uint8_t> garbage(32, 0x5A);
        arch << static_cast<uint32_t>(garbage.size());
        arch.WriteBuffer(garbage.data(), garbage.size());
      });

  EXPECT_THROW(readBack(buf), std::exception);
}

// v2: grayscale as raw bytes, no external-storage flag yet.
TEST(CImageLegacySerialization, Version2Grayscale)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 2,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << uint8_t(0);                              // hasColor
        arch << int32_t(4) << int32_t(2) << int32_t(1);  // width, height, origin
        arch << int32_t(8);                              // imageSize
        for (int i = 0; i < 8; i++)
        {
          arch << static_cast<uint8_t>(100 + i);
        }
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 4);
  ASSERT_EQ(img.getHeight(), 2);
  EXPECT_EQ(img.at<uint8_t>(0, 0), 100);
  EXPECT_EQ(img.at<uint8_t>(3, 1), 107);
}

// v4: images of 16 kB or less were never ZIP-compressed, and the flag is not
// stored in the stream.
TEST(CImageLegacySerialization, Version4SmallGrayscaleIsNotZipped)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 4,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << uint8_t(0);
        arch << int32_t(2) << int32_t(2) << int32_t(1);
        arch << int32_t(4);
        for (int i = 0; i < 4; i++)
        {
          arch << static_cast<uint8_t>(7 * i);
        }
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.at<uint8_t>(1, 1), 21);
}

// v5+: the "is ZIP compressed" flag is explicit in the stream. MRPT 3.x cannot
// decompress those, and must say so rather than returning garbage.
TEST(CImageLegacySerialization, ZippedGrayscaleThrowsExplanatoryError)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 5,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << uint8_t(0);
        arch << int32_t(2) << int32_t(2) << int32_t(1);
        arch << int32_t(4);
        arch << true;         // imageIsZIP
        arch << uint32_t(3);  // zipDataLen
        const uint8_t zip[] = {1, 2, 3};
        arch.WriteBuffer(zip, sizeof(zip));
      });

  EXPECT_THROW(readBack(buf), std::exception);
}

// v5: explicit flag saying the data is *not* zipped.
TEST(CImageLegacySerialization, Version5UncompressedGrayscale)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 5,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << uint8_t(0);
        arch << int32_t(2) << int32_t(2) << int32_t(1);
        arch << int32_t(4);
        arch << false;  // imageIsZIP
        for (int i = 0; i < 4; i++)
        {
          arch << static_cast<uint8_t>(200 + i);
        }
      });

  const CImage img = readBack(buf);
  EXPECT_EQ(img.at<uint8_t>(0, 0), 200);
  EXPECT_EQ(img.at<uint8_t>(1, 1), 203);
}

// v6+ added the external-storage flag before anything else.
TEST(CImageLegacySerialization, Version6ExternalStorage)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 6,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << true;  // imgIsExternalStorage
        arch << std::string("some_image.png");
      });

  const CImage img = readBack(buf);
  EXPECT_TRUE(img.isExternallyStored());
  EXPECT_EQ(img.getExternalStorageFile(), "some_image.png");
}

// v9 added an explicit pixel-depth field for grayscale images.
TEST(CImageLegacySerialization, Version9GrayscaleWith16bitDepth)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 9,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << false;       // imgIsExternalStorage
        arch << uint8_t(0);  // hasColor
        arch << int32_t(2) << int32_t(2) << int32_t(1);
        arch << int32_t(2 * 2 * 2);  // imageSize, 2 bytes/pixel
        arch << int32_t(2);          // PixelDepth::D16U
        arch << false;               // imageIsZIP
        for (int i = 0; i < 4; i++)
        {
          arch << static_cast<uint16_t>(1000 + i);
        }
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.getPixelDepth(), mrpt::img::PixelDepth::D16U);
  EXPECT_EQ(img.at<uint16_t>(0, 0), 1000);
  EXPECT_EQ(img.at<uint16_t>(1, 1), 1003);
}

// v7+ color: positive dimensions mean "a JPEG blob follows".
TEST(CImageLegacySerialization, Version7ColorJpeg)
{
  const auto jpeg = asJPEG(makeRGB(16, 8));

  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 8,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << false;       // imgIsExternalStorage
        arch << uint8_t(1);  // hasColor
        arch << int32_t(16) << int32_t(8);
        arch << static_cast<uint32_t>(jpeg.size());
        arch.WriteBuffer(jpeg.data(), jpeg.size());
      });

  const CImage img = readBack(buf);
  EXPECT_EQ(img.getWidth(), 16);
  EXPECT_EQ(img.getHeight(), 8);
  EXPECT_TRUE(img.isColor());
}

// v8 color: *negative* dimensions signal raw, uncompressed BGR rows.
TEST(CImageLegacySerialization, Version8ColorRawBGR)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 8,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << false;
        arch << uint8_t(1);
        arch << int32_t(-2) << int32_t(-2);  // 2x2, raw BGR
        const uint8_t bgr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
        arch.WriteBuffer(bgr, sizeof(bgr));
      });

  const CImage img = readBack(buf);
  ASSERT_EQ(img.getWidth(), 2);
  ASSERT_EQ(img.getHeight(), 2);
  EXPECT_TRUE(img.isColor());
  EXPECT_EQ(img.ptr<uint8_t>(0, 0)[0], 3);  // R <- B slot
  EXPECT_EQ(img.ptr<uint8_t>(0, 0)[2], 1);  // B <- R slot
  EXPECT_EQ(img.ptr<uint8_t>(1, 1), img.ptr<uint8_t>(1, 1));
  EXPECT_EQ(img.ptr<uint8_t>(1, 1)[0], 12);
}

// A truncated raw-BGR payload must be reported, not silently accepted.
TEST(CImageLegacySerialization, Version8TruncatedRawBGRThrows)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 8,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << false;
        arch << uint8_t(1);
        arch << int32_t(-4) << int32_t(-4);
        const uint8_t bgr[] = {1, 2, 3};  // way too short
        arch.WriteBuffer(bgr, sizeof(bgr));
      });

  EXPECT_THROW(readBack(buf), std::exception);
}

// A degenerate 0xN color image: neither JPEG nor raw data follows.
TEST(CImageLegacySerialization, Version8ZeroSizedColorImage)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 8,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << false;
        arch << uint8_t(1);
        arch << int32_t(0) << int32_t(5);
      });

  const CImage img = readBack(buf);
  EXPECT_TRUE(img.isEmpty());
}

// v100 was written by MRPT 2.x builds without OpenCV: only the external-storage
// path carries any data.
TEST(CImageLegacySerialization, Version100ExternalStorage)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 100,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << true;
        arch << std::string("no_opencv.png");
      });

  const CImage img = readBack(buf);
  EXPECT_TRUE(img.isExternallyStored());
  EXPECT_EQ(img.getExternalStorageFile(), "no_opencv.png");
}

TEST(CImageLegacySerialization, Version100NoImageData)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::CImage", 100, [&](mrpt::serialization::CArchive& arch) { arch << false; });

  const CImage img = readBack(buf);
  EXPECT_FALSE(img.isExternallyStored());
  EXPECT_TRUE(img.isEmpty());
}

TEST(CImageLegacySerialization, UnknownVersionThrows)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(buf, "mrpt::img::CImage", 42, [](mrpt::serialization::CArchive&) {});

  EXPECT_THROW(readBack(buf), std::exception);
}

// TStereoCamera's legacy formats: v0 carried a distortion-model byte that is
// no longer used, and v1 is not readable at all.
TEST(TStereoCameraLegacySerialization, Version0SkipsModelByte)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::TStereoCamera", 0,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << uint8_t(0);  // unused distortion-model byte
        mrpt::img::TCamera l;
        mrpt::img::TCamera r;
        l.ncols = 320;
        r.ncols = 320;
        arch << l << r << mrpt::math::TPose3DQuat();
      });

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TStereoCamera sc;
  arch >> sc;
  EXPECT_EQ(sc.leftCamera.ncols, 320);
  EXPECT_EQ(sc.rightCamera.ncols, 320);
}

// v1 stored the right-camera pose as a CPose3DQuat object, which MRPT 3.x
// does not know how to read back.
TEST(TStereoCameraLegacySerialization, Version1IsRejected)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(buf, "mrpt::img::TStereoCamera", 1, [](mrpt::serialization::CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TStereoCamera sc;
  EXPECT_THROW(arch >> sc, std::exception);
}

TEST(TStereoCameraLegacySerialization, UnknownVersionThrows)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::TStereoCamera", 77, [](mrpt::serialization::CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TStereoCamera sc;
  EXPECT_THROW(arch >> sc, std::exception);
}

// ---------------------------------------------------------------------------
// TCamera legacy formats: v0 carried an extra CMatrixDouble15 block, v<2 had no
// image resolution (defaulted to 640x480), v<4 stored the full 3x3 intrinsics
// matrix, v<5 had no camera name and v<6 no distortion-model field.
// ---------------------------------------------------------------------------
TEST(TCameraLegacySerialization, Version0)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::TCamera", 0,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << 0.015;  // focalLengthMeters
        for (int k = 0; k < 5; k++)
        {
          arch << (0.1 * k);  // 5 distortion params
        }
        mrpt::math::CMatrixDouble33 K;
        K.setIdentity();
        K(0, 0) = 500.0;
        K(1, 1) = 510.0;
        K(0, 2) = 320.0;
        K(1, 2) = 240.0;
        // v0 wrote values that must be forced back to 0/1 on read:
        K(0, 1) = 7.0;
        K(2, 2) = 3.0;
        arch << K;
        arch << mrpt::math::CMatrixFixed<double, 1, 5>();  // the v0-only dummy block
      });

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TCamera cam;
  arch >> cam;

  EXPECT_NEAR(cam.focalLengthMeters, 0.015, 1e-9);
  EXPECT_NEAR(cam.fx(), 500.0, 1e-9);
  EXPECT_NEAR(cam.cy(), 240.0, 1e-9);
  EXPECT_NEAR(cam.intrinsicParams(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(cam.intrinsicParams(2, 2), 1.0, 1e-12);
  // No resolution was stored before v2:
  EXPECT_EQ(cam.ncols, 640U);
  EXPECT_EQ(cam.nrows, 480U);
  // No distortion model was stored before v6:
  EXPECT_EQ(cam.distortion, mrpt::img::DistortionModel::plumb_bob);
  EXPECT_NEAR(cam.dist[1], 0.1, 1e-12);
  EXPECT_NEAR(cam.dist[5], 0.0, 1e-12);  // 6..8 only exist since v3
}

TEST(TCameraLegacySerialization, Version3AddsThreeMoreDistParams)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::img::TCamera", 3,
      [&](mrpt::serialization::CArchive& arch)
      {
        arch << 0.0;
        for (int k = 0; k < 8; k++)
        {
          arch << (1.0 + k);
        }
        mrpt::math::CMatrixDouble33 K;
        K.setIdentity();
        arch << K;
        arch << uint32_t(100) << uint32_t(200);  // nrows, ncols (v2+)
      });

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TCamera cam;
  cam.cameraName = "stale_name_from_a_previous_read";
  arch >> cam;

  EXPECT_NEAR(cam.dist[7], 8.0, 1e-12);
  EXPECT_EQ(cam.nrows, 100U);
  EXPECT_EQ(cam.ncols, 200U);
  // cameraName was added in v5: it must be reset to its default, not kept
  // from whatever the reused destination object held.
  EXPECT_EQ(cam.cameraName, "camera1");
}

TEST(TCameraLegacySerialization, UnknownVersionThrows)
{
  CMemoryStream buf;
  writeLegacyObjectFrame(buf, "mrpt::img::TCamera", 200, [](mrpt::serialization::CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  mrpt::img::TCamera cam;
  EXPECT_THROW(arch >> cam, std::exception);
}
