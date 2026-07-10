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

TEST(CImage, KLT_responseLargeWindowSizes)
{
  using namespace mrpt::img;

  // Big enough image so that even a half_window_size=32 fits comfortably.
  CImage a(200, 200, CH_GRAY);
  a.filledRectangle({0, 0}, {199, 199}, TColor(0, 0, 0x10));
  a.filledRectangle({99, 99}, {101, 101}, TColor(0, 0, 0x90));

  // These correspond to the explicit switch-case template instantiations.
  for (int w : {12, 13, 14, 15, 16, 32})
  {
    const auto resp = a.KLT_response({100, 100}, w);
    EXPECT_GT(resp, 0.0f) << " w=" << w;
  }

  // A window size not explicitly listed in the switch falls back to the
  // generic (non-template) code path.
  const auto respGeneric = a.KLT_response({100, 100}, 20);
  EXPECT_GT(respGeneric, 0.0f);
}

TEST(CImage, KLT_responseOutOfBoundsWindowThrows)
{
  using namespace mrpt::img;
  CImage a(20, 20, CH_GRAY);
  EXPECT_THROW((void)a.KLT_response({1, 1}, 5), std::exception);
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

TEST(CImage, SetPixelGray)
{
  using namespace mrpt::img;
  CImage img(5, 5, CH_GRAY);
  img.setPixelGray({2, 2}, 123);
  EXPECT_EQ(img.at<uint8_t>(2, 2), 123);

  // Out-of-bounds is a silent no-op.
  img.setPixelGray({100, 100}, 7);
  img.setPixelGray({-1, -1}, 7);

  // Only valid on grayscale images.
  CImage color(5, 5, CH_RGB);
  EXPECT_THROW(color.setPixelGray({0, 0}, 1), std::exception);
}

TEST(CImage, SetPixelChannelCounts)
{
  using namespace mrpt::img;
  {
    CImage img(3, 3, CH_GRAY);
    img.setPixel({1, 1}, TColor(10, 20, 30));
    EXPECT_EQ(img.at<uint8_t>(1, 1), 30);  // gray uses the Blue channel
  }
  {
    CImage img(3, 3, CH_RGB);
    img.setPixel({1, 1}, TColor(10, 20, 30));
    EXPECT_EQ(img.at<uint8_t>(1, 1, 0), 10);
    EXPECT_EQ(img.at<uint8_t>(1, 1, 1), 20);
    EXPECT_EQ(img.at<uint8_t>(1, 1, 2), 30);
  }
  {
    CImage img(3, 3, CH_RGBA);
    img.setPixel({1, 1}, TColor(10, 20, 30, 40));
    EXPECT_EQ(img.at<uint8_t>(1, 1, 0), 10);
    EXPECT_EQ(img.at<uint8_t>(1, 1, 1), 20);
    EXPECT_EQ(img.at<uint8_t>(1, 1, 2), 30);
    EXPECT_EQ(img.at<uint8_t>(1, 1, 3), 40);
  }
}

TEST(CImage, GetAsFloat)
{
  using namespace mrpt::img;
  CImage img(4, 4, CH_RGB);
  img.filledRectangle({0, 0}, {3, 3}, TColor(0, 0, 0));
  img.setPixel({1, 1}, TColor(255, 0, 0));

  EXPECT_NEAR(img.getAsFloat({1, 1}, 0), 1.0f, 1e-4f);
  EXPECT_NEAR(img.getAsFloat({1, 1}, 1), 0.0f, 1e-4f);

  // Overload without a channel: for color images, the gray-equivalent.
  EXPECT_GT(img.getAsFloat({1, 1}), 0.0f);

  CImage gray(4, 4, CH_GRAY);
  gray.setPixel({0, 0}, TColor(128, 128, 128));
  EXPECT_NEAR(gray.getAsFloat({0, 0}), 128.0f / 255.0f, 1e-3f);

  EXPECT_THROW((void)img.getAsFloat({100, 100}, 0), std::exception);
}

TEST(CImage, ExtractPatch)
{
  using namespace mrpt::img;
  CImage img(10, 10, CH_GRAY);
  for (int y = 0; y < 10; y++)
  {
    for (int x = 0; x < 10; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>(x + y * 10);
    }
  }

  CImage patch;
  img.extract_patch(patch, {2, 3}, {4, 2});

  EXPECT_EQ(patch.getWidth(), 4);
  EXPECT_EQ(patch.getHeight(), 2);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      EXPECT_EQ(patch.at<uint8_t>(x, y), img.at<uint8_t>(x + 2, y + 3));
    }
  }
}

TEST(CImage, GetAsMatrixVariants)
{
  using namespace mrpt::img;
  CImage img(4, 3, CH_GRAY);
  for (int y = 0; y < 3; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>((x + y * 4) * 10);
    }
  }

  mrpt::math::CMatrixFloat mf;
  img.getAsMatrix(mf, true, 0, 0, -1, -1, true /* normalize */);
  EXPECT_EQ(mf.rows(), 3);
  EXPECT_EQ(mf.cols(), 4);
  EXPECT_NEAR(mf(0, 0), 0.0f, 1e-3f);
  EXPECT_GT(mf(2, 3), 0.0f);

  mrpt::math::CMatrix_u8 mu;
  img.getAsMatrix(mu, true, 0, 0, -1, -1);
  EXPECT_EQ(mu(2, 3), img.at<uint8_t>(3, 2));

  // Sub-region extraction.
  mrpt::math::CMatrixFloat sub;
  img.getAsMatrix(sub, true, 1, 1, 2, 2, false);
  EXPECT_EQ(sub.rows(), 2);
  EXPECT_EQ(sub.cols(), 2);
  EXPECT_EQ(sub(0, 0), img.at<uint8_t>(1, 1));
}

TEST(CImage, GetAsMatrixBytesColorImage)
{
  using namespace mrpt::img;
  CImage img(2, 2, CH_RGB);
  img.setPixel({0, 0}, TColor(255, 0, 0));

  mrpt::math::CMatrix_u8 mu;
  img.getAsMatrix(mu, true, 0, 0, -1, -1);
  // Luminance of pure red: (255*299)/1000 = 76
  EXPECT_EQ(mu(0, 0), 76);
}

TEST(CImage, GetSize)
{
  using namespace mrpt::img;
  CImage img(7, 5, CH_GRAY);
  const auto sz = img.getSize();
  EXPECT_EQ(sz.x, 7);
  EXPECT_EQ(sz.y, 5);

  CImage empty;
  const auto szEmpty = empty.getSize();
  EXPECT_EQ(szEmpty.x, 0);
  EXPECT_EQ(szEmpty.y, 0);
}

TEST(CImage, ImagesPathBase)
{
  using namespace mrpt::img;
  const auto prev = CImage::getImagesPathBase();
  CImage::setImagesPathBase("/tmp/some_test_path");
  EXPECT_EQ(CImage::getImagesPathBase(), "/tmp/some_test_path");
  CImage::setImagesPathBase(prev);
}

TEST(CImage, LoadFromStreamAsJPEGInvalidDataThrows)
{
  using namespace mrpt::img;
  mrpt::io::CMemoryStream buf;
  const uint8_t garbage[] = {0x00, 0x01, 0x02, 0x03, 0x04};
  buf.Write(garbage, sizeof(garbage));
  buf.Seek(0);

  CImage img;
  EXPECT_THROW(img.loadFromStreamAsJPEG(buf), std::exception);
}

TEST(CImage, SaveToStreamAsJPEGEmptyImageThrows)
{
  using namespace mrpt::img;
  CImage img;
  mrpt::io::CMemoryStream buf;
  EXPECT_THROW(img.saveToStreamAsJPEG(buf), std::exception);
}

TEST(CImage, FilterMedianInPlace)
{
  using namespace mrpt::img;
  CImage img(10, 10, CH_GRAY);
  for (int y = 0; y < 10; y++)
  {
    for (int x = 0; x < 10; x++)
    {
      *img.ptr<uint8_t>(x, y) = 100;
    }
  }
  *img.ptr<uint8_t>(5, 5) = 255;  // impulse noise

  img.filterMedian(img, 3);  // in-place: out_img aliases *this
  EXPECT_LE(*img.ptr<uint8_t>(5, 5), 110u);
}

TEST(CImage, FilterGaussianInPlace)
{
  using namespace mrpt::img;
  CImage img(11, 11, CH_GRAY);
  for (int y = 0; y < 11; y++)
  {
    for (int x = 0; x < 11; x++)
    {
      *img.ptr<uint8_t>(x, y) = 0;
    }
  }
  *img.ptr<uint8_t>(5, 5) = 255;

  img.filterGaussian(img, 5, 5, 1.5);  // in-place
  EXPECT_GT(*img.ptr<uint8_t>(5, 5), *img.ptr<uint8_t>(0, 0));
}

TEST(CImage, RotateImageInPlace)
{
  using namespace mrpt::img;
  CImage img(20, 20, CH_GRAY);
  for (int y = 0; y < 20; y++)
  {
    for (int x = 0; x < 20; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>(x + y);
    }
  }

  img.rotateImage(img, 0.0, {10, 10}, 1.0);  // in-place, zero rotation
  EXPECT_EQ(img.getWidth(), 20);
  EXPECT_EQ(img.getHeight(), 20);
}

TEST(CImage, GetAsRGBMatrices)
{
  using namespace mrpt::img;
  {
    CImage img(4, 3, CH_RGB);
    for (int y = 0; y < 3; y++)
    {
      for (int x = 0; x < 4; x++)
      {
        img.setPixel(
            {x, y}, TColor(
                        static_cast<uint8_t>(x * 10), static_cast<uint8_t>(y * 10),
                        static_cast<uint8_t>((x + y) * 5)));
      }
    }

    auto [rf, gf, bf] = img.getAsRGBMatricesFloat();
    EXPECT_EQ(rf.rows(), 3);
    EXPECT_EQ(rf.cols(), 4);
    EXPECT_NEAR(rf(1, 2), 20.0f / 255.0f, 1e-3f);
    EXPECT_NEAR(gf(1, 2), 10.0f / 255.0f, 1e-3f);

    auto [rb, gb, bb] = img.getAsRGBMatricesBytes();
    EXPECT_EQ(rb(1, 2), 20);
    EXPECT_EQ(gb(1, 2), 10);
    EXPECT_EQ(bb(1, 2), 15);
  }
  {
    // Grayscale source: R=G=B replicate the gray level.
    CImage gray(3, 3, CH_GRAY);
    gray.setPixel({0, 0}, TColor(42, 42, 42));
    auto [rf, gf, bf] = gray.getAsRGBMatricesFloat();
    EXPECT_NEAR(rf(0, 0), gf(0, 0), 1e-6f);
    EXPECT_NEAR(gf(0, 0), bf(0, 0), 1e-6f);

    auto [rb, gb, bb] = gray.getAsRGBMatricesBytes();
    EXPECT_EQ(rb(0, 0), gb(0, 0));
    EXPECT_EQ(gb(0, 0), bb(0, 0));
  }
}

TEST(CImage, CrossCorrelationFFT)
{
  using namespace mrpt::img;
  CImage img(32, 32, CH_GRAY);
  for (int y = 0; y < 32; y++)
  {
    for (int x = 0; x < 32; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>((x * 7 + y * 3) % 256);
    }
  }

  CImage patch;
  img.extract_patch(patch, {10, 10}, {8, 8});

  mrpt::math::CMatrixFloat corr;
  img.cross_correlation_FFT(patch, corr);

  EXPECT_GT(corr.rows(), 0);
  EXPECT_GT(corr.cols(), 0);

  // This is a phase-correlation style implementation: for an exact match
  // (the patch was extracted verbatim from the image) it should produce a
  // single, sharp peak (much larger than the average response) rather than
  // a flat/degenerate response.
  float sum = 0;
  float peak = corr(0, 0);
  for (int r = 0; r < corr.rows(); r++)
  {
    for (int c = 0; c < corr.cols(); c++)
    {
      sum += corr(r, c);
      peak = std::max(peak, corr(r, c));
    }
  }
  const float mean = sum / static_cast<float>(corr.rows() * corr.cols());
  EXPECT_GT(peak, mean * 2.0f);
}

TEST(CImage, CrossCorrelationFFTWithSearchWindowAndBias)
{
  using namespace mrpt::img;
  CImage img(32, 32, CH_GRAY);
  for (int y = 0; y < 32; y++)
  {
    for (int x = 0; x < 32; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>((x * 7 + y * 3) % 256);
    }
  }

  CImage patch;
  img.extract_patch(patch, {4, 4}, {8, 8});

  mrpt::math::CMatrixFloat corr;
  // Exercise the optional search-window and bias parameters.
  img.cross_correlation_FFT(patch, corr, 0, 0, 16, 16, 10.0f, 5.0f);

  EXPECT_GT(corr.rows(), 0);
  EXPECT_GT(corr.cols(), 0);
}

TEST(CImage, UnloadOnNonExternalIsNoOp)
{
  using namespace mrpt::img;
  CImage img(4, 4, CH_GRAY);
  img.unload();  // Not external-storage: should just do nothing.
  EXPECT_FALSE(img.isEmpty());
  EXPECT_EQ(img.getWidth(), 4);
}

TEST(CImage, ExternalStorageForceLoadAndUnload)
{
  using namespace mrpt::img;
  using namespace std::string_literals;

  CImage img(6, 6, CH_GRAY);
  img.setPixel({0, 0}, TColor(9, 9, 9));
  const auto f = mrpt::system::getTempFileName() + ".png"s;
  ASSERT_TRUE(img.saveToFile(f));

  CImage ext;
  ext.setExternalStorage(f);
  EXPECT_TRUE(ext.isExternallyStored());
  EXPECT_EQ(ext.getExternalStorageFile(), f);
  EXPECT_FALSE(ext.getExternalStorageFileAbsolutePath().empty());

  // Accessing the image triggers a lazy load.
  EXPECT_EQ(ext.getWidth(), 6);
  ext.forceLoad();
  ext.unload();
  // Width should still be queryable after unload (re-loads on demand).
  EXPECT_EQ(ext.getWidth(), 6);
}

TEST(CImage, FlipVerticalAndHorizontal)
{
  using namespace mrpt::img;
  CImage img(4, 3, CH_GRAY);
  for (int y = 0; y < 3; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>(x + y * 10);
    }
  }

  // NOTE: CImage's copy ctor/operator= are shallow copies (shared pixel
  // buffer), so an independent deep copy is required before mutating one of
  // the two images in place.
  CImage flippedV(img, DEEP_COPY);
  flippedV.flipVertical();
  for (int y = 0; y < 3; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      EXPECT_EQ(flippedV.at<uint8_t>(x, y), img.at<uint8_t>(x, 2 - y));
    }
  }

  CImage flippedH(img, DEEP_COPY);
  flippedH.flipHorizontal();
  for (int y = 0; y < 3; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      EXPECT_EQ(flippedH.at<uint8_t>(x, y), img.at<uint8_t>(3 - x, y));
    }
  }
}

TEST(CImage, SwapRB)
{
  using namespace mrpt::img;
  CImage img(3, 3, CH_RGB);
  img.setPixel({1, 1}, TColor(10, 20, 30));
  img.swapRB();
  EXPECT_EQ(img.at<uint8_t>(1, 1, 0), 30);
  EXPECT_EQ(img.at<uint8_t>(1, 1, 1), 20);
  EXPECT_EQ(img.at<uint8_t>(1, 1, 2), 10);
}

TEST(CImage, ColorImageFromGray)
{
  using namespace mrpt::img;
  CImage gray(4, 4, CH_GRAY);
  gray.setPixel({0, 0}, TColor(77, 77, 77));

  CImage color = gray.colorImage();
  EXPECT_TRUE(color.isColor());
  EXPECT_EQ(color.at<uint8_t>(0, 0, 0), 77);
  EXPECT_EQ(color.at<uint8_t>(0, 0, 1), 77);
  EXPECT_EQ(color.at<uint8_t>(0, 0, 2), 77);

  // Already-color image: colorImage() is a (shallow) no-op copy.
  CImage stillColor = color.colorImage();
  EXPECT_TRUE(stillColor.isColor());

  // In-place overload, including the aliased (ret==*this) case.
  CImage gray2(3, 3, CH_GRAY);
  gray2.setPixel({0, 0}, TColor(5, 5, 5));
  gray2.colorImage(gray2);
  EXPECT_TRUE(gray2.isColor());
  EXPECT_EQ(gray2.at<uint8_t>(0, 0, 0), 5);
}

TEST(CImage, JoinImagesHorz)
{
  using namespace mrpt::img;
  CImage left(3, 2, CH_GRAY);
  CImage right(2, 2, CH_GRAY);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      left.at<uint8_t>(x, y) = static_cast<uint8_t>(10 + x + y);
    }
    for (int x = 0; x < 2; x++)
    {
      right.at<uint8_t>(x, y) = static_cast<uint8_t>(90 + x + y);
    }
  }

  CImage joined;
  joined.joinImagesHorz(left, right);

  EXPECT_EQ(joined.getWidth(), 5);
  EXPECT_EQ(joined.getHeight(), 2);
  for (int y = 0; y < 2; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      EXPECT_EQ(joined.at<uint8_t>(x, y), left.at<uint8_t>(x, y));
    }
    for (int x = 0; x < 2; x++)
    {
      EXPECT_EQ(joined.at<uint8_t>(3 + x, y), right.at<uint8_t>(x, y));
    }
  }
}

TEST(CImage, JoinImagesHorzMismatchedHeightThrows)
{
  using namespace mrpt::img;
  CImage a(3, 2, CH_GRAY);
  CImage b(3, 5, CH_GRAY);
  CImage joined;
  EXPECT_THROW(joined.joinImagesHorz(a, b), std::exception);
}

TEST(CImage, LoadFromMemoryBuffer)
{
  using namespace mrpt::img;
  std::vector<uint8_t> buf = {
      255, 0,   0,    // pixel(0,0): red
      0,   255, 0,    // pixel(1,0): green
      0,   0,   255,  // pixel(0,1): blue
      10,  20,  30,   // pixel(1,1)
  };

  CImage img;
  img.loadFromMemoryBuffer(2, 2, CH_RGB, buf.data(), false);
  EXPECT_EQ(img.getWidth(), 2);
  EXPECT_EQ(img.getHeight(), 2);
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 255);
  EXPECT_EQ(img.at<uint8_t>(1, 0, 1), 255);

  CImage imgSwapped;
  imgSwapped.loadFromMemoryBuffer(2, 2, CH_RGB, buf.data(), true);
  EXPECT_EQ(imgSwapped.at<uint8_t>(0, 0, 0), 0);
  EXPECT_EQ(imgSwapped.at<uint8_t>(0, 0, 2), 255);
}

TEST(CImage, SaveAndLoadStreamJPEG)
{
  using namespace mrpt::img;
  CImage img(16, 12, CH_RGB);
  img.filledRectangle({0, 0}, {15, 11}, TColor(10, 20, 30));
  img.filledRectangle({2, 2}, {5, 5}, TColor(200, 100, 50));

  mrpt::io::CMemoryStream buf;
  img.saveToStreamAsJPEG(buf, 90);
  EXPECT_GT(buf.getTotalBytesCount(), 0U);

  buf.Seek(0);
  CImage loaded;
  loaded.loadFromStreamAsJPEG(buf);
  EXPECT_EQ(loaded.getWidth(), 16);
  EXPECT_EQ(loaded.getHeight(), 12);
  EXPECT_TRUE(loaded.isColor());
}

TEST(CImage, SaveToFileMultipleFormats)
{
  using namespace mrpt::img;
  using namespace std::string_literals;

  CImage img(8, 6, CH_RGB);
  img.filledRectangle({0, 0}, {7, 5}, TColor(30, 60, 90));

  for (const std::string& ext : {"jpg"s, "bmp"s, "tga"s, "png"s})
  {
    const auto f = mrpt::system::getTempFileName() + "."s + ext;
    EXPECT_TRUE(img.saveToFile(f, 90)) << ext;

    CImage loaded;
    EXPECT_TRUE(loaded.loadFromFile(f)) << ext;
    EXPECT_EQ(loaded.getWidth(), 8) << ext;
    EXPECT_EQ(loaded.getHeight(), 6) << ext;
  }
}

TEST(CImage, SaveToFileUnknownExtensionFails)
{
  using namespace mrpt::img;
  using namespace std::string_literals;
  CImage img(4, 4, CH_RGB);
  const auto f = mrpt::system::getTempFileName() + ".this_is_not_a_format"s;
  EXPECT_FALSE(img.saveToFile(f));
}

TEST(CImage, LoadFromFileMissingFileFails)
{
  using namespace mrpt::img;
  using namespace std::string_literals;
  CImage img;
  EXPECT_FALSE(img.loadFromFile(mrpt::system::getTempFileName() + "_does_not_exist.png"s));
}

TEST(CImage, LoadFromFileStaticFactory)
{
  using namespace mrpt::img;
  CImage img = CImage::LoadFromFile(tstImgFileColor);
  EXPECT_FALSE(img.isEmpty());
}

TEST(CImage, LoadFromFileStaticFactoryThrowsOnError)
{
  using namespace mrpt::img;
  using namespace std::string_literals;
  EXPECT_THROW(
      CImage::LoadFromFile(mrpt::system::getTempFileName() + "_does_not_exist.png"s),
      std::exception);
}

TEST(CImage, CopyFromForceLoad)
{
  using namespace mrpt::img;
  using namespace std::string_literals;

  CImage src(5, 5, CH_GRAY);
  src.setPixel({0, 0}, TColor(88, 88, 88));
  const auto f = mrpt::system::getTempFileName() + ".png"s;
  ASSERT_TRUE(src.saveToFile(f));

  CImage ext;
  ext.setExternalStorage(f);

  CImage dst;
  dst.copyFromForceLoad(ext);
  // copyFromForceLoad() does a shallow copy (dst shares state with ext), so
  // the "externally stored" flag carries over; what forceLoad() guarantees
  // is that the pixel data is resident in memory, accessible without
  // further disk I/O.
  EXPECT_TRUE(dst.isExternallyStored());
  EXPECT_EQ(dst.getWidth(), 5);
  EXPECT_EQ(dst.at<uint8_t>(0, 0), 88);
}

TEST(CImage, GetRowStrideAndChannelsOrder)
{
  using namespace mrpt::img;
  CImage img(10, 5, CH_RGB);
  EXPECT_EQ(img.getRowStride(), 30U);
  EXPECT_EQ(img.getChannelsOrder(), "RGB");

  CImage imgGray(10, 5, CH_GRAY);
  EXPECT_EQ(imgGray.getChannelsOrder(), "GRAY");

  CImage imgRgba(10, 5, CH_RGBA);
  EXPECT_EQ(imgRgba.getChannelsOrder(), "RGBA");
}

TEST(CImage, ScaleDouble)
{
  using namespace mrpt::img;
  CImage img(4, 4, CH_GRAY);
  for (int y = 0; y < 4; y++)
  {
    for (int x = 0; x < 4; x++)
    {
      img.at<uint8_t>(x, y) = static_cast<uint8_t>(x * 10 + y);
    }
  }
  CImage out;
  img.scaleDouble(out, IMG_INTERP_NN);
  EXPECT_EQ(out.getWidth(), 8);
  EXPECT_EQ(out.getHeight(), 8);

  const CImage out2 = img.scaleDouble(IMG_INTERP_LINEAR);
  EXPECT_EQ(out2.getWidth(), 8);
  EXPECT_EQ(out2.getHeight(), 8);
}

TEST(CImage, DrawImageChannelConversions)
{
  using namespace mrpt::img;

  // Grayscale source drawn onto an RGB destination (gray replicated to R,G,B).
  {
    CImage dst(6, 6, CH_RGB);
    dst.filledRectangle({0, 0}, {5, 5}, TColor::black());
    CImage src(2, 2, CH_GRAY);
    src.filledRectangle({0, 0}, {1, 1}, TColor(42, 42, 42));
    dst.drawImage({1, 1}, src);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 0), 42);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 1), 42);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 2), 42);
  }
  // Grayscale source drawn onto an RGBA destination (alpha filled with 255).
  {
    CImage dst(6, 6, CH_RGBA);
    dst.filledRectangle({0, 0}, {5, 5}, TColor::black());
    CImage src(2, 2, CH_GRAY);
    src.filledRectangle({0, 0}, {1, 1}, TColor(9, 9, 9));
    dst.drawImage({1, 1}, src);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 0), 9);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 3), 255);
  }
  // Color source drawn onto a grayscale destination (luminance conversion).
  {
    CImage dst(6, 6, CH_GRAY);
    dst.filledRectangle({0, 0}, {5, 5}, TColor::black());
    CImage src(2, 2, CH_RGB);
    src.filledRectangle({0, 0}, {1, 1}, TColor(255, 0, 0));
    dst.drawImage({1, 1}, src);
    EXPECT_GT(dst.at<uint8_t>(1, 1), 0);
  }
  // Same channel-count copy (RGB -> RGB) takes the direct memcpy path.
  {
    CImage dst(6, 6, CH_RGB);
    dst.filledRectangle({0, 0}, {5, 5}, TColor::black());
    CImage src(2, 2, CH_RGB);
    src.filledRectangle({0, 0}, {1, 1}, TColor(4, 5, 6));
    dst.drawImage({1, 1}, src);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 0), 4);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 1), 5);
    EXPECT_EQ(dst.at<uint8_t>(1, 1, 2), 6);
  }
  // RGB <-> RGBA channel-count mismatch is not a supported conversion.
  {
    CImage dst(6, 6, CH_RGB);
    CImage src(2, 2, CH_RGBA);
    EXPECT_THROW(dst.drawImage({1, 1}, src), std::exception);
  }
  // Out-of-bounds placement: partially clipped, should not crash.
  {
    CImage dst(4, 4, CH_GRAY);
    CImage src(4, 4, CH_GRAY);
    src.filledRectangle({0, 0}, {3, 3}, TColor::white());
    dst.drawImage({2, 2}, src);
    dst.drawImage({-2, -2}, src);
    EXPECT_EQ(dst.at<uint8_t>(0, 0), 255);
  }
}

TEST(CImage, ScaleHalfWithOddSizeTruncates)
{
  using namespace mrpt::img;
  // Odd dimensions are truncated (integer division by 2), not rejected.
  CImage img(5, 5, CH_GRAY);
  CImage out;
  const bool usedSse = img.scaleHalf(out, IMG_INTERP_NN);
  EXPECT_FALSE(usedSse);
  EXPECT_EQ(out.getWidth(), 2);
  EXPECT_EQ(out.getHeight(), 2);
}
