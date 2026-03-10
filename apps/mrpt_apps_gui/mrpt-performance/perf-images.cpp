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

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/cpu.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/CStereoRectifyMap.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::random;
using namespace std;

extern void getTestImage(unsigned int img_index, mrpt::img::CImage& out_img);

// ------------------------------------------------------
//				Benchmark: image loading/saving
// ------------------------------------------------------
double image_test_1(int w, int img_quality)
{
  int h = 0;
  switch (w)
  {
    case 640:
      h = 480;
      break;
    case 800:
      h = 600;
      break;
    case 1024:
      h = 768;
      break;
    case 1280:
      h = 1024;
      break;
    default:
      THROW_EXCEPTION("Invalid 'w'!");
  }

  CImage img(w, h, mrpt::img::CH_RGB);

  for (int i = 0; i < 5000; i++)
    img.line(
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        TColor(getRandomGenerator().drawUniform32bit()));

  CTicTac tictac;

  const string fil = mrpt::system::getTempFileName() + ".jpg";

  const size_t N = 30;

  tictac.Tic();
  for (size_t i = 0; i < N; i++)
  {
    bool savedOk = img.saveToFile(fil, img_quality);
    ASSERT_(savedOk);
  }

  const double T = tictac.Tac() / N;
  mrpt::system::deleteFile(fil);
  return T;
}

// ------------------------------------------------------
//				Benchmark: save/load to disk vs shared mem
// ------------------------------------------------------
template <bool perf_load>
double image_saveload(int iFormat, int to_shm)
{
  const char* format;
  switch (iFormat)
  {
    case 0:
      format = "bmp";
      break;
    case 1:
      format = "png";
      break;
    case 2:
      format = "jpg";
      break;
    default:
      THROW_EXCEPTION("Wrong format");
  }

  const int w = 800, h = 600;
  CImage img(w, h, mrpt::img::CH_RGB);
  for (int i = 0; i < 5000; i++)
    img.line(
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        TColor(getRandomGenerator().drawUniform32bit()));

  CTicTac tictac;

  string fil;
  if (to_shm)
  {
    fil = string("/dev/shm/mrpt_perf_test.") + string(format);
  }
  else
  {
    fil = mrpt::system::getTempFileName() + string(".") + string(format);
  }

  double T;
  if (perf_load)
  {
    // LOAD:
    bool savedOk = img.saveToFile(fil);
    ASSERT_(savedOk);

    const size_t N = 30;

    tictac.Tic();
    for (size_t i = 0; i < N; i++)
    {
      CImage img_new;
      bool loadOk = img.loadFromFile(fil);
      ASSERT_(loadOk);
    }

    T = tictac.Tac() / N;
  }
  else
  {
    // WRITE:
    const size_t N = 30;

    tictac.Tic();
    for (size_t i = 0; i < N; i++)
    {
      bool savedOk = img.saveToFile(fil);
      ASSERT_(savedOk);
    }

    T = tictac.Tac() / N;
  }

  mrpt::system::deleteFile(fil);
  return T;
}

double image_test_2(int w, int h)
{
  CImage img(w, h, mrpt::img::CH_RGB), img2;

  for (int i = 0; i < 5000; i++)
    img.line(
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        TColor(getRandomGenerator().drawUniform32bit()));

  CTicTac tictac;

  const size_t N = 50;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) img.filterGaussian(img2, 7, 7);

  double R = tictac.Tac() / N;

  return R;
}

template <TImageChannels IMG_CHANNELS, bool DISABLE_SIMD = false>
double image_halfsample(int w, int h)
{
  CImage img(w, h, IMG_CHANNELS), img2;
  CTicTac tictac;

  const bool savedFeatSSE2 = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  const bool savedFeatSSSE3 = mrpt::cpu::supports(mrpt::cpu::feature::SSSE3);
  if (DISABLE_SIMD)
  {
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, false);
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSSE3, false);
  }

  const size_t N = 300;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) img.scaleHalf(img2, mrpt::img::IMG_INTERP_NN);

  if (DISABLE_SIMD)
  {
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, savedFeatSSE2);
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSSE3, savedFeatSSSE3);
  }
  return tictac.Tac() / N;
}

template <TImageChannels IMG_CHANNELS, bool DISABLE_SIMD = false>
double image_halfsample_smooth(int w, int h)
{
  CImage img(w, h, IMG_CHANNELS), img2;

  const bool savedFeatSSE2 = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  const bool savedFeatSSSE3 = mrpt::cpu::supports(mrpt::cpu::feature::SSSE3);
  if (DISABLE_SIMD)
  {
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, false);
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSSE3, false);
  }

  CTicTac tictac;

  const size_t N = 300;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) img.scaleHalf(img2, mrpt::img::IMG_INTERP_LINEAR);

  if (DISABLE_SIMD)
  {
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, savedFeatSSE2);
    mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSSE3, savedFeatSSSE3);
  }

  return tictac.Tac() / N;
}

double image_rgb2gray_8u(int w, int h)
{
  CImage img(w, h, CH_RGB), img2;

  CTicTac tictac;

  const size_t N = 300;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) img.grayscale(img2);

  return tictac.Tac() / N;
}

double image_KLTscore(int WIN, int N)
{
  static const size_t w = 800;
  static const size_t h = 800;
  CImage img(w, h, CH_GRAY);

  for (int i = 0; i < 5000; i++)
    img.line(
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        mrpt::img::TPixelCoord{
            (int)getRandomGenerator().drawUniform(0, w - 1),
            (int)getRandomGenerator().drawUniform(0, h - 1)},
        TColor(getRandomGenerator().drawUniform32bit()));

  ASSERT_LT_(WIN, 128);
  int x = 0;
  int y = 0;

  CTicTac tictac;
  tictac.Tic();
  for (int i = 0; i < N; i++)
  {
    float r = img.KLT_response(mrpt::img::TPixelCoord{x | 128, y | 128}, WIN);
    (void)r;
    x++;
    x &= 0x1FF;
    y++;
    y &= 0x1FF;
  }

  double R = tictac.Tac() / N;

  return R;
}

// ------------------------------------------------------
// register_tests_image
// ------------------------------------------------------
void register_tests_image()
{
  lstTests.emplace_back("images: Save as JPEG (640x480, quality=95%)", image_test_1, 640, 95);
  lstTests.emplace_back("images: Save as JPEG (800x600, quality=95%)", image_test_1, 800, 95);
  lstTests.emplace_back("images: Save as JPEG (1024x768, quality=95%)", image_test_1, 1024, 95);

  lstTests.emplace_back("images: Save as JPEG (640x480, quality=75%)", image_test_1, 640, 75);
  lstTests.emplace_back("images: Save as JPEG (800x600, quality=75%)", image_test_1, 800, 75);
  lstTests.emplace_back("images: Save as JPEG (1024x768, quality=75%)", image_test_1, 1024, 75);

  lstTests.emplace_back("images: Save BMP 800x600 disk", image_saveload<false>, 0, 0);
  lstTests.emplace_back("images: Save PNG 800x600 disk", image_saveload<false>, 1, 0);
  lstTests.emplace_back("images: Save JPG 800x600 disk", image_saveload<false>, 2, 0);
  lstTests.emplace_back("images: Load BMP 800x600 disk", image_saveload<true>, 0, 0);
  lstTests.emplace_back("images: Load PNG 800x600 disk", image_saveload<true>, 1, 0);
  lstTests.emplace_back("images: Load JPG 800x600 disk", image_saveload<true>, 2, 0);

  if (mrpt::system::directoryExists("/dev/shm"))
  {
    lstTests.emplace_back("images: Save BMP 800x600 shared mem", image_saveload<false>, 0, 1);
    lstTests.emplace_back("images: Save PNG 800x600 shared mem", image_saveload<false>, 1, 1);
    lstTests.emplace_back("images: Save JPG 800x600 shared mem", image_saveload<false>, 2, 1);
    lstTests.emplace_back("images: Load BMP 800x600 shared mem", image_saveload<true>, 0, 1);
    lstTests.emplace_back("images: Load PNG 800x600 shared mem", image_saveload<true>, 1, 1);
    lstTests.emplace_back("images: Load JPG 800x600 shared mem", image_saveload<true>, 2, 1);
  }

  lstTests.emplace_back("images: Gauss filter (640x480)", image_test_2, 640, 480);
  lstTests.emplace_back("images: Gauss filter (800x600)", image_test_2, 800, 600);
  lstTests.emplace_back("images: Gauss filter (1024x768)", image_test_2, 1024, 768);

  lstTests.emplace_back("images: Half sample GRAY (160x120)", image_halfsample<CH_GRAY>, 160, 120);
  lstTests.emplace_back("images: Half sample GRAY (320x240)", image_halfsample<CH_GRAY>, 320, 240);
  lstTests.emplace_back("images: Half sample GRAY (640x480)", image_halfsample<CH_GRAY>, 640, 480);
  lstTests.emplace_back("images: Half sample GRAY (800x600)", image_halfsample<CH_GRAY>, 800, 600);
  lstTests.emplace_back(
      "images: Half sample GRAY (1024x768)", image_halfsample<CH_GRAY>, 1024, 768);

  lstTests.emplace_back(
      "images: Half sample GRAY (1280x1024)", image_halfsample<CH_GRAY>, 1280, 1024);
  lstTests.emplace_back(
      "images: Half sample GRAY (1280x1024) [SSE2 disabled]", image_halfsample<CH_GRAY, true>, 1280,
      1024);

  lstTests.emplace_back("images: Half sample RGB (160x120)", image_halfsample<CH_RGB>, 160, 120);
  lstTests.emplace_back("images: Half sample RGB (320x240)", image_halfsample<CH_RGB>, 320, 240);
  lstTests.emplace_back("images: Half sample RGB (640x480)", image_halfsample<CH_RGB>, 640, 480);
  lstTests.emplace_back("images: Half sample RGB (800x600)", image_halfsample<CH_RGB>, 800, 600);
  lstTests.emplace_back("images: Half sample RGB (1024x768)", image_halfsample<CH_RGB>, 1024, 768);
  lstTests.emplace_back(
      "images: Half sample RGB (1280x1024)", image_halfsample<CH_RGB>, 1280, 1024);
  lstTests.emplace_back(
      "images: Half sample RGB (1280x1024) [SSSE3 disabled]", image_halfsample<CH_RGB>, 1280, 1024);

  lstTests.emplace_back(
      "images: Half sample smooth GRAY (160x120)", image_halfsample_smooth<CH_GRAY>, 160, 120);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (320x240)", image_halfsample_smooth<CH_GRAY>, 320, 240);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (640x480)", image_halfsample_smooth<CH_GRAY>, 640, 480);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (800x600)", image_halfsample_smooth<CH_GRAY>, 800, 600);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (1024x768)", image_halfsample_smooth<CH_GRAY>, 1024, 768);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (1280x1024)", image_halfsample_smooth<CH_GRAY>, 1280, 1024);
  lstTests.emplace_back(
      "images: Half sample smooth GRAY (1280x1024) [SSSE3 disabled]",
      image_halfsample_smooth<CH_GRAY, true>, 1280, 1024);

  lstTests.emplace_back(
      "images: Half sample smooth RGB (160x120)", image_halfsample_smooth<CH_RGB>, 160, 120);
  lstTests.emplace_back(
      "images: Half sample smooth RGB (320x240)", image_halfsample_smooth<CH_RGB>, 320, 240);
  lstTests.emplace_back(
      "images: Half sample smooth RGB (640x480)", image_halfsample_smooth<CH_RGB>, 640, 480);
  lstTests.emplace_back(
      "images: Half sample smooth RGB (800x600)", image_halfsample_smooth<CH_RGB>, 800, 600);
  lstTests.emplace_back(
      "images: Half sample smooth RGB (1024x768)", image_halfsample_smooth<CH_RGB>, 1024, 768);
  lstTests.emplace_back(
      "images: Half sample smooth RGB (1280x1024)", image_halfsample_smooth<CH_RGB>, 1280, 1024);

  lstTests.emplace_back("images: RGB->GRAY 8u (40x30)", image_rgb2gray_8u, 40, 30);
  lstTests.emplace_back("images: RGB->GRAY 8u (80x60)", image_rgb2gray_8u, 80, 60);
  lstTests.emplace_back("images: RGB->GRAY 8u (160x120)", image_rgb2gray_8u, 160, 120);
  lstTests.emplace_back("images: RGB->GRAY 8u (320x240)", image_rgb2gray_8u, 320, 240);
  lstTests.emplace_back("images: RGB->GRAY 8u (640x480)", image_rgb2gray_8u, 640, 480);
  lstTests.emplace_back("images: RGB->GRAY 8u (800x600)", image_rgb2gray_8u, 800, 600);
  lstTests.emplace_back("images: RGB->GRAY 8u (1024x768)", image_rgb2gray_8u, 1024, 768);
  lstTests.emplace_back("images: RGB->GRAY 8u (1280x1024)", image_rgb2gray_8u, 1280, 1024);

  lstTests.emplace_back("images: KLT score (WIN=2 5x5)", image_KLTscore, 2, 1e7);
  lstTests.emplace_back("images: KLT score (WIN=3 7x7)", image_KLTscore, 3, 1e7);
  lstTests.emplace_back("images: KLT score (WIN=4 9x9)", image_KLTscore, 4, 1e7);
  lstTests.emplace_back("images: KLT score (WIN=5 10x10)", image_KLTscore, 5, 1e7);
  lstTests.emplace_back("images: KLT score (WIN=6 13x13)", image_KLTscore, 6, 1e7);
  lstTests.emplace_back("images: KLT score (WIN=7 15x15)", image_KLTscore, 7, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=8 17x17)", image_KLTscore, 8, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=9 19x19)", image_KLTscore, 9, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=10 21x21)", image_KLTscore, 10, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=11 23x23)", image_KLTscore, 11, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=12 25x25)", image_KLTscore, 12, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=13 27x27)", image_KLTscore, 13, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=14 29x29)", image_KLTscore, 14, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=15 31x31)", image_KLTscore, 15, 1e6);
  lstTests.emplace_back("images: KLT score (WIN=16 33x33)", image_KLTscore, 16, 1e6);
}
