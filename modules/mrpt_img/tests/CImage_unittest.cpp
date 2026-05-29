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
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>
#include <test_mrpt_common.h>

template class mrpt::CTraitsTest<mrpt::img::CImage>;

using namespace std::string_literals;
const auto tstImgFileColor =
    mrpt::UNITTEST_BASEDIR() + "/../../mrpt_examples_cpp/img_basic_example/frame_color.jpg"s;

// Generate random img:
namespace
{
void fillImagePseudoRandom(int32_t seed, mrpt::img::CImage& img)
{
  mrpt::random::Randomize(static_cast<unsigned int>(seed));
  auto& rnd = mrpt::random::getRandomGenerator();

  for (int y = 0; y < img.getHeight(); y++)
  {
    for (int x = 0; x < img.getWidth(); x++)
    {
      const uint8_t c = static_cast<uint8_t>(rnd.drawUniform32bit());
      img.at<uint8_t>(x, y) = c;
    }
  }
}

// Expect images to be identical:
bool expect_identical(
    const mrpt::img::CImage& a, const mrpt::img::CImage& b, const std::string& s = std::string())
{
  EXPECT_EQ(a.getWidth(), b.getWidth());
  EXPECT_EQ(a.getHeight(), b.getHeight());
  for (int y = 0; y < a.getHeight(); y++)
  {
    for (int x = 0; x < a.getWidth(); x++)
    {
      EXPECT_EQ(a.at<uint8_t>(x, y), b.at<uint8_t>(x, y)) << s;
      if (a.at<uint8_t>(x, y) != b.at<uint8_t>(x, y))
      {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

TEST(CImage, CtorDefault)
{
  mrpt::img::CImage img;
  EXPECT_TRUE(img.isEmpty());

  // clang-format off
	EXPECT_THROW([&img]() { auto b = img.isColor();  ((void)b); }(), std::exception);
	EXPECT_THROW([&img]() { auto b = img.getWidth(); ((void)b); }(), std::exception);
	EXPECT_THROW([&img]() { auto b = img.getHeight(); ((void)b);}(), std::exception);
  // clang-format on

  mrpt::img::PixelDepth p = mrpt::img::PixelDepth::D8U;
  EXPECT_THROW(p = img.getPixelDepth(), std::exception);
  (void)p;
}

namespace
{
void CtorSized_gray(int w, int h)
{
  using namespace mrpt::img;
  CImage img(w, h, CH_GRAY);
  EXPECT_EQ(img.getWidth(), w);
  EXPECT_EQ(img.getHeight(), 48U);
  EXPECT_EQ(img.channels(), 1U);
  EXPECT_EQ(img.getPixelDepth(), PixelDepth::D8U);
  EXPECT_FALSE(img.isColor());
}
}  // namespace

TEST(CImage, CtorSized)
{
  using namespace mrpt::img;
  {
    CImage img(64, 48, CH_RGB);
    EXPECT_EQ(img.getWidth(), 64U);
    EXPECT_EQ(img.getHeight(), 48U);
    EXPECT_EQ(img.channels(), 3U);
    EXPECT_EQ(img.getPixelDepth(), PixelDepth::D8U);
    EXPECT_TRUE(img.isColor());
  }
  for (int w = 64; w < 70; w++)
  {
    CtorSized_gray(w, 48);
  }
}

TEST(CImage, GetSetPixel)
{
  using namespace mrpt::img;
  CImage img(20, 10, CH_GRAY);
  img.setPixel({10, 2}, TColor(0x80, 0x80, 0x80));
  EXPECT_EQ(img.at<uint8_t>(10, 2), 0x80);

  img.setPixel({11, 2}, TColor(0x0, 0x0, 0x0));
  EXPECT_EQ(img.at<uint8_t>(11, 2), 0x00);

  img.setPixel({12, 2}, TColor(0xff, 0xff, 0xff));
  EXPECT_EQ(img.at<uint8_t>(12, 2), 0xff);

  img.at<uint8_t>(13, 2) = 0x70;
  EXPECT_EQ(img.at<uint8_t>(13, 2), 0x70);

  auto* line = img.ptrLine<uint8_t>(5);
  for (uint8_t i = 0; i < 20; i++)
  {
    line[i] = i;
    EXPECT_EQ(img.at<uint8_t>(i, 5), i);
  }

  mrpt::math::CMatrixFloat M;
  img.getAsMatrix(M, true, 0, 0, -1, -1, false /* dont normalize (0,1) */);
  for (uint8_t i = 0; i < 20; i++)
  {
    EXPECT_NEAR(static_cast<double>(M(5, i)), i, 1e-8);
  }
}

TEST(CImage, CopyMoveSwap)
{
  using namespace mrpt::img;
  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;
    // Shallow copy:
    CImage b = a;
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);

    a.at<uint8_t>(1, 3) = 0x81;
    EXPECT_EQ(b.at<uint8_t>(1, 3), 0x81);

    // Deep copy:
    CImage c = a.makeDeepCopy();
    EXPECT_EQ(c.at<uint8_t>(1, 2), 0x80);

    c.at<uint8_t>(1, 3) = 0x0;
    a.at<uint8_t>(1, 3) = 0x81;
    EXPECT_NE(c.at<uint8_t>(1, 3), 0x81);
  }

  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;
    // Shallow copy ctor:
    CImage b(a, mrpt::img::SHALLOW_COPY);
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);

    a.at<uint8_t>(1, 3) = 0x81;
    EXPECT_EQ(b.at<uint8_t>(1, 3), 0x81);

    // Deep copy ctor:
    CImage c(a, mrpt::img::DEEP_COPY);
    EXPECT_EQ(c.at<uint8_t>(1, 2), 0x80);

    c.at<uint8_t>(1, 3) = 0x0;
    a.at<uint8_t>(1, 3) = 0x81;
    EXPECT_NE(c.at<uint8_t>(1, 3), 0x81);
  }

  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;
    // Deep copy:
    CImage b = a.makeDeepCopy();
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);

    a.clear();
    a.resize(30, 30, CH_RGB);
    b.at<uint8_t>(1, 3) = 0x0;
    a.at<uint8_t>(1, 3) = 0x81;
    EXPECT_NE(b.at<uint8_t>(1, 3), 0x81);
  }

  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;
    // move:
    CImage b = std::move(a);
    EXPECT_EQ(b.getWidth(), 20U);
    EXPECT_EQ(b.getHeight(), 10U);
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);
  }

  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;
    // swap:
    CImage b;
    a.swap(b);
    EXPECT_EQ(b.getWidth(), 20U);
    EXPECT_EQ(b.getHeight(), 10U);
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);
  }
}

TEST(CImage, ExternalImage)
{
  using namespace mrpt::img;
  {
    CImage a;
    a.setExternalStorage(tstImgFileColor);
    // Test automatic load-on-the-fly:
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }

  {
    CImage a;
    a.setExternalStorage("./foo_61717181.png");
    // Test exception on not found
    EXPECT_THROW(
        [&a]
        {
          bool w = a.getWidth();
          ((void)w);
        }(),
        mrpt::img::CExceptionExternalImageNotFound);
  }
}

TEST(CImage, ConvertGray)
{
  using namespace mrpt::img;
  {
    CImage a;
    bool load_ok = a.loadFromFile(tstImgFileColor);
    EXPECT_TRUE(load_ok);

    CImage b = a.grayscale();
    EXPECT_EQ(b.getWidth(), a.getWidth());
    EXPECT_EQ(b.getHeight(), a.getHeight());
    EXPECT_FALSE(b.isColor());
  }
}

TEST(CImage, CtorRefOrGray)
{
  using namespace mrpt::img;
  {
    CImage a;
    bool load_ok = a.loadFromFile(tstImgFileColor);
    EXPECT_TRUE(load_ok);

    const CImage b(a, FAST_REF_OR_CONVERT_TO_GRAY);
    EXPECT_EQ(b.getWidth(), a.getWidth());
    EXPECT_EQ(b.getHeight(), a.getHeight());
    EXPECT_FALSE(b.isColor());
  }
  {
    CImage a(20, 10, CH_GRAY);
    a.at<uint8_t>(1, 2) = 0x80;

    const CImage b(a, FAST_REF_OR_CONVERT_TO_GRAY);
    EXPECT_EQ(b.getWidth(), a.getWidth());
    EXPECT_EQ(b.getHeight(), a.getHeight());
    EXPECT_FALSE(b.isColor());
    EXPECT_EQ(b.at<uint8_t>(1, 2), 0x80);
  }
}

TEST(CImage, HalfAndDouble)
{
  using namespace mrpt::img;

  CImage a(32, 10, CH_GRAY);
  a.at<uint8_t>(0, 0) = 0x80;
  a.at<uint8_t>(0, 1) = 0x80;
  a.at<uint8_t>(1, 0) = 0x80;
  a.at<uint8_t>(1, 1) = 0x80;

  // Half:
  {
    const CImage imgH = a.scaleHalf(mrpt::img::IMG_INTERP_NN);
    EXPECT_EQ(imgH.getWidth(), a.getWidth() / 2);
    EXPECT_EQ(imgH.getHeight(), a.getHeight() / 2);
    EXPECT_EQ(imgH.isColor(), a.isColor());
    EXPECT_EQ(imgH.at<uint8_t>(0, 0), a.at<uint8_t>(0, 0));
  }
  // Double:
  {
    const CImage imgD = a.scaleDouble(mrpt::img::IMG_INTERP_NN);
    EXPECT_EQ(imgD.getWidth(), a.getWidth() * 2);
    EXPECT_EQ(imgD.getHeight(), a.getHeight() * 2);
    EXPECT_EQ(imgD.isColor(), a.isColor());
  }
}
TEST(CImage, getChannelsOrder)
{
  using namespace mrpt::img;
  {
    CImage a;
    bool load_ok = a.loadFromFile(tstImgFileColor);
    EXPECT_TRUE(load_ok);
    EXPECT_EQ(std::string("RGB"), a.getChannelsOrder());
  }
  {
    CImage a(32, 10, CH_GRAY);
    EXPECT_EQ(std::string("GRAY"), a.getChannelsOrder());
  }
}

TEST(CImage, ScaleImage)
{
  using namespace mrpt::img;
  CImage a;
  bool load_ok = a.loadFromFile(tstImgFileColor);
  EXPECT_TRUE(load_ok);

  {
    CImage b(600, 400);
    a.scaleImage(b, 600, 400);
    EXPECT_EQ(b.getWidth(), 600U);
    EXPECT_EQ(b.getHeight(), 400U);
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }
  {
    CImage b;
    a.scaleImage(b, 600, 400);
    EXPECT_EQ(b.getWidth(), 600U);
    EXPECT_EQ(b.getHeight(), 400U);
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }

  for (int pass = 0; pass < 2; pass++)
  {
    CImage c;
    if (pass == 0)
    {
      c = a.makeDeepCopy();
    }
    else
    {
      a.scaleImage(c, 311, 211);
    }
    const auto cw = c.getWidth();
    const auto ch = c.getHeight();

    {
      CImage b;
      c.scaleHalf(b, IMG_INTERP_NN);
      EXPECT_EQ(b.getWidth(), cw / 2);
      EXPECT_EQ(b.getHeight(), ch / 2);
      EXPECT_EQ(c.getWidth(), cw);
      EXPECT_EQ(c.getHeight(), ch);
    }
    {
      CImage ag = c.grayscale();
      CImage b;
      ag.scaleHalf(b, IMG_INTERP_LINEAR);
      EXPECT_EQ(b.getWidth(), cw / 2);
      EXPECT_EQ(b.getHeight(), ch / 2);
      EXPECT_EQ(ag.getWidth(), cw);
      EXPECT_EQ(ag.getHeight(), ch);
    }
    {
      CImage ag = c.grayscale();
      CImage b;
      ag.scaleHalf(b, IMG_INTERP_NN);
      EXPECT_EQ(b.getWidth(), cw / 2);
      EXPECT_EQ(b.getHeight(), ch / 2);
      EXPECT_EQ(ag.getWidth(), cw);
      EXPECT_EQ(ag.getHeight(), ch);
    }
  }  // two passes

  {
    CImage b;
    a.scaleHalf(b, IMG_INTERP_LINEAR);
    EXPECT_EQ(b.getWidth(), a.getWidth() / 2);
    EXPECT_EQ(b.getHeight(), a.getHeight() / 2);
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }
  {
    CImage b;
    a.scaleDouble(b, IMG_INTERP_NN);
    EXPECT_EQ(b.getWidth(), a.getWidth() * 2);
    EXPECT_EQ(b.getHeight(), a.getHeight() * 2);
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }
  {
    CImage b;
    a.scaleDouble(b, IMG_INTERP_LINEAR);
    EXPECT_EQ(b.getWidth(), a.getWidth() * 2);
    EXPECT_EQ(b.getHeight(), a.getHeight() * 2);
    EXPECT_EQ(a.getWidth(), 320U);
    EXPECT_EQ(a.getHeight(), 240U);
  }
}

TEST(CImage, Serialize)
{
  using namespace mrpt::img;
  CImage a;
  bool load_ok = a.loadFromFile(tstImgFileColor);
  EXPECT_TRUE(load_ok);

  mrpt::math::CMatrixFloat am;
  a.getAsMatrix(am, true, 0, 0, -1, -1, false /* dont normalize to [0,1] */);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << a;
  buf.Seek(0);
  CImage b;
  arch >> b;

  mrpt::math::CMatrixFloat bm;
  b.getAsMatrix(bm, true, 0, 0, -1, -1, false /* dont normalize to [0,1] */);

  EXPECT_EQ(am, bm);
}

TEST(CImage, KLT_response)
{
  using namespace mrpt::img;

  {
    CImage a(100, 90, CH_GRAY);
    a.filledRectangle({0, 0}, {99, 99}, TColor(0, 0, 0x10));
    a.filledRectangle({40, 30}, {41, 31}, TColor(0, 0, 0x90));

    for (int w = 2; w < 12; w++)
    {
      const auto resp = a.KLT_response({40, 30}, w);
      EXPECT_GT(resp, 0.9f) << " w=" << w;

      const auto flatResp = a.KLT_response({20, 20}, w);
      EXPECT_LT(flatResp, 0.1f) << " w=" << w;
    }
  }
}

TEST(CImage, LoadAndComparePseudoRnd)
{
  using namespace mrpt::img;
  using namespace std::string_literals;

  const auto tstimg = mrpt::mrpt_data_dir() + "/tests/test_pseudorandom_img_seed70.png"s;

  CImage a;
  bool load_ok = a.loadFromFile(tstimg);
  EXPECT_TRUE(load_ok) << "Cannot load: " << tstimg;

  CImage b(10, 7, CH_GRAY);
  fillImagePseudoRandom(70, b);

  expect_identical(a, b, "LoadAndComparePseudoRnd"s);
}

TEST(CImage, LoadAndSave)
{
  using namespace mrpt::img;
  using namespace std::string_literals;

  for (int h = 7; h < 20; h += 17)
  {
    for (int w = 10; w < 33; w++)
    {
      CImage a(w, h, CH_GRAY);
      fillImagePseudoRandom(w * h, a);

      const auto f = mrpt::system::getTempFileName() + ".png"s;

      const auto tstName = mrpt::format("From: LoadAndSave test w=%u h=%u", w, h);

      bool saved_ok = a.saveToFile(f);
      EXPECT_TRUE(saved_ok) << tstName;

      CImage b;
      bool load_ok = b.loadFromFile(f);
      EXPECT_TRUE(load_ok) << tstName;

      if (!expect_identical(a, b, tstName))
      {
        GTEST_FAIL();
      }
    }
  }
}

TEST(CImage, DifferentAccessMethodsColor)
{
  using namespace mrpt::img;
  CImage a;
  bool load_ok = a.loadFromFile(tstImgFileColor);
  EXPECT_TRUE(load_ok);
  EXPECT_TRUE(a.isColor());

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      for (int8_t ch = 0; ch < 3; ch++)
      {
        EXPECT_EQ(a.at<uint8_t>(c, r, ch), *a.ptr<uint8_t>(c, r, ch)) << "ch=" << ch << "\n";
        EXPECT_EQ(a.at<uint8_t>(c, r, ch), a.ptrLine<uint8_t>(r)[(c * 3) + ch])
            << "(c,r,ch)=(" << c << "," << r << "," << ch << ")"
            << "\n &a.ptrLine<uint8_t>(r)[c * 3 + ch] = "
            << static_cast<void*>(&a.ptrLine<uint8_t>(r)[(c * 3) + ch])
            << "\n a.ptrLine<uint8_t>(r) = " << static_cast<void*>(a.ptrLine<uint8_t>(r)) << "\n";
      }
    }
  }
}

TEST(CImage, DifferentAccessMethodsGray)
{
  using namespace mrpt::img;
  CImage a;
  bool load_ok = a.loadFromFile(tstImgFileColor);
  EXPECT_TRUE(load_ok);
  a = a.grayscale();
  EXPECT_FALSE(a.isColor());

  for (int r = 5; r < 7; r++)
  {
    for (int c = 10; c < 12; c++)
    {
      EXPECT_EQ(a.at<uint8_t>(c, r), *a.ptr<uint8_t>(c, r));
      EXPECT_EQ(a.at<uint8_t>(c, r), a.ptrLine<uint8_t>(r)[c]);
    }
  }
}

TEST(CImage, NormalizeStretchesRange)
{
  using namespace mrpt::img;
  CImage img(10, 10, CH_GRAY);
  // Set all pixels to 50 except one at 10 and one at 200
  for (int y = 0; y < 10; y++)
  {
    for (int x = 0; x < 10; x++)
    {
      *img.ptr<uint8_t>(x, y) = 50;
    }
  }
  *img.ptr<uint8_t>(0, 0) = 10;
  *img.ptr<uint8_t>(9, 9) = 200;

  img.normalize();

  EXPECT_EQ(*img.ptr<uint8_t>(0, 0), 0);
  EXPECT_EQ(*img.ptr<uint8_t>(9, 9), 255);
}

TEST(CImage, FilterMedianRemovesImpulseNoise)
{
  using namespace mrpt::img;
  // Create a constant gray image with one salt pixel
  CImage img(10, 10, CH_GRAY);
  for (int y = 0; y < 10; y++)
  {
    for (int x = 0; x < 10; x++)
    {
      *img.ptr<uint8_t>(x, y) = 100;
    }
  }
  *img.ptr<uint8_t>(5, 5) = 255;  // impulse noise

  CImage out;
  img.filterMedian(out, 3);

  // The center pixel should be corrected back to ~100
  EXPECT_LE(*out.ptr<uint8_t>(5, 5), 110u);
}

TEST(CImage, FilterGaussianSmoothing)
{
  using namespace mrpt::img;
  // Impulse image: single white pixel in the center of a black image
  CImage img(11, 11, CH_GRAY);
  for (int y = 0; y < 11; y++)
  {
    for (int x = 0; x < 11; x++)
    {
      *img.ptr<uint8_t>(x, y) = 0;
    }
  }
  *img.ptr<uint8_t>(5, 5) = 255;

  CImage out;
  img.filterGaussian(out, 5, 5, 1.5);

  // After Gaussian filter, the center should still be brightest
  EXPECT_GT(*out.ptr<uint8_t>(5, 5), *out.ptr<uint8_t>(0, 0));
  // Neighboring pixels should be non-zero
  EXPECT_GT(*out.ptr<uint8_t>(4, 5), 0u);
}

TEST(CImage, RotateImageIdentity)
{
  using namespace mrpt::img;
  CImage img;
  bool load_ok = img.loadFromFile(tstImgFileColor);
  EXPECT_TRUE(load_ok);
  img = img.grayscale();

  const int w = static_cast<int>(img.getWidth());
  const int h = static_cast<int>(img.getHeight());

  CImage out;
  // Rotate by 0 radians -> should be identical
  img.rotateImage(out, 0.0, {w / 2, h / 2}, 1.0);

  EXPECT_EQ(out.getWidth(), img.getWidth());
  EXPECT_EQ(out.getHeight(), img.getHeight());
  // Center pixel should be unchanged
  EXPECT_EQ(*out.ptr<uint8_t>(w / 2, h / 2), *img.ptr<uint8_t>(w / 2, h / 2));
}

TEST(CImage, DrawChessboardCorners)
{
  using namespace mrpt::img;
  CImage img(200, 200, CH_RGB);
  img.filledRectangle({0, 0}, {199, 199}, TColor(200, 200, 200));

  // 3x3 grid of corners
  std::vector<TPixelCoordf> corners;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      corners.push_back({static_cast<float>(40 + c * 60), static_cast<float>(40 + r * 60)});
    }
  }
  bool ok = img.drawChessboardCorners(corners, 3, 3, 1, 5);
  EXPECT_TRUE(ok);

  // Wrong count -> should return false
  corners.pop_back();
  bool ok2 = img.drawChessboardCorners(corners, 3, 3, 1, 5);
  EXPECT_FALSE(ok2);
}
