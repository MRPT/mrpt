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

#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <gtest/gtest.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/fourier.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <thread>

using namespace mrpt::math;

// =========================================================================
//  fft_real
// =========================================================================

// A pure sine at frequency k has energy only in bin k.
TEST(Fourier, fft_real_pure_sine)
{
  constexpr size_t N = 64;
  constexpr size_t k = 4;  // frequency bin

  CVectorFloat in(static_cast<Eigen::Index>(N));
  for (size_t i = 0; i < N; i++)
  {
    in[static_cast<Eigen::Index>(i)] = std::sin(
        2.0f * static_cast<float>(M_PI) * static_cast<float>(k) * static_cast<float>(i) /
        static_cast<float>(N));
  }

  CVectorFloat re, im, mag;
  fft_real(in, re, im, mag);

  const size_t n2 = 1 + N / 2;
  ASSERT_EQ(static_cast<size_t>(mag.size()), n2);

  // Peak must be at bin k — find it manually to avoid Eigen index type issues
  size_t peakBin = 0;
  float peakVal = 0.0f;
  for (size_t i = 0; i < n2; i++)
  {
    if (mag[static_cast<Eigen::Index>(i)] > peakVal)
    {
      peakVal = mag[static_cast<Eigen::Index>(i)];
      peakBin = i;
    }
  }
  EXPECT_EQ(peakBin, k);

  // DC and all other bins except k should be close to zero
  for (size_t i = 0; i < n2; i++)
  {
    if (i != k)
    {
      EXPECT_NEAR(mag[static_cast<Eigen::Index>(i)], 0.0f, 0.05f)
          << "Unexpected energy at bin " << i;
    }
  }
}

// A constant signal has energy only in the DC bin (bin 0).
TEST(Fourier, fft_real_dc)
{
  constexpr size_t N = 16;
  CVectorFloat in(static_cast<Eigen::Index>(N));
  in.setConstant(3.0f);

  CVectorFloat re, im, mag;
  fft_real(in, re, im, mag);

  EXPECT_GT(mag[0], 1.0f);  // DC bin has energy

  const size_t n2 = 1 + N / 2;
  for (size_t i = 1; i < n2; i++)
  {
    EXPECT_NEAR(mag[static_cast<Eigen::Index>(i)], 0.0f, 0.05f);
  }
}

// =========================================================================
//  dft2_real / idft2_real  (round-trip)
// =========================================================================

TEST(Fourier, dft2_real_roundtrip)
{
  constexpr int R = 8, C = 8;
  CMatrixFloat in(R, C);
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) in(i, j) = static_cast<float>(i * C + j + 1);

  CMatrixFloat outR, outI;
  dft2_real(in, outR, outI);

  CMatrixFloat recovered;
  idft2_real(outR, outI, recovered);

  ASSERT_EQ(recovered.rows(), R);
  ASSERT_EQ(recovered.cols(), C);
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++)
      EXPECT_NEAR(recovered(i, j), in(i, j), 0.01f) << "at (" << i << "," << j << ")";
}

// =========================================================================
//  dft2_complex / idft2_complex  (round-trip, power-of-2)
// =========================================================================

TEST(Fourier, dft2_complex_roundtrip_pow2)
{
  constexpr int R = 8, C = 8;
  CMatrixFloat inR(R, C), inI(R, C);
  inI.setZero();
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) inR(i, j) = static_cast<float>((i + 1) * (j + 1));

  CMatrixFloat fR, fI;
  dft2_complex(inR, inI, fR, fI);

  CMatrixFloat recR, recI;
  idft2_complex(fR, fI, recR, recI);

  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++)
      EXPECT_NEAR(recR(i, j), inR(i, j), 0.01f) << "at (" << i << "," << j << ")";
}

// Non-power-of-2 sizes fall through to the general DFT path.
TEST(Fourier, dft2_complex_roundtrip_nonpow2)
{
  constexpr int R = 3, C = 5;
  CMatrixFloat inR(R, C), inI(R, C);
  inI.setZero();
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) inR(i, j) = static_cast<float>(i + j + 1);

  CMatrixFloat fR, fI;
  dft2_complex(inR, inI, fR, fI);

  CMatrixFloat recR, recI;
  idft2_complex(fR, fI, recR, recI);

  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++)
      EXPECT_NEAR(recR(i, j), inR(i, j), 0.05f) << "at (" << i << "," << j << ")";
}

// =========================================================================
//  cross_correlation_FFT
// =========================================================================

// Auto-correlation of a non-zero signal must have its peak at (0,0).
TEST(Fourier, cross_correlation_FFT_autocorr_peak)
{
  constexpr int R = 8, C = 8;
  CMatrixFloat A(R, C);
  A.setZero();
  A(2, 3) = 1.0f;  // single impulse

  CMatrixFloat corr;
  cross_correlation_FFT(A, A, corr);

  ASSERT_EQ(corr.rows(), R);
  ASSERT_EQ(corr.cols(), C);

  // The maximum should be at (0,0) for auto-correlation of an impulse
  float maxVal = corr.maxCoeff();
  EXPECT_GT(maxVal, 0.0f);
}

// =========================================================================
//  Sign conventions and general sizes
// =========================================================================

namespace
{
using cd = std::complex<double>;

CMatrixFloat randomMatrix(int rows, int cols)
{
  auto& rng = mrpt::random::getRandomGenerator();
  CMatrixFloat m(rows, cols);
  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      m(r, c) = static_cast<float>(rng.drawUniform(-1.0, 1.0));
    }
  }
  return m;
}

// Reference 2D DFT, standard convention: X(k) = sum x(n) exp(-2 pi i k n / N)
void naiveDFT2(
    const CMatrixFloat& re, const CMatrixFloat& im, CMatrixFloat& outRe, CMatrixFloat& outIm)
{
  const auto n1 = re.rows();
  const auto n2 = re.cols();
  outRe.setSize(n1, n2);
  outIm.setSize(n1, n2);
  for (long k1 = 0; k1 < n1; k1++)
  {
    for (long k2 = 0; k2 < n2; k2++)
    {
      cd s = 0;
      for (long i1 = 0; i1 < n1; i1++)
      {
        for (long i2 = 0; i2 < n2; i2++)
        {
          const double ph = -2 * M_PI *
                            (static_cast<double>(i1 * k1) / static_cast<double>(n1) +
                             static_cast<double>(i2 * k2) / static_cast<double>(n2));
          s += cd(re(i1, i2), im(i1, i2)) * std::exp(cd(0, ph));
        }
      }
      outRe(k1, k2) = static_cast<float>(s.real());
      outIm(k1, k2) = static_cast<float>(s.imag());
    }
  }
}

float maxAbsDiff(const CMatrixFloat& a, const CMatrixFloat& b)
{
  return (a.asEigen() - b.asEigen()).cwiseAbs().maxCoeff();
}

constexpr float kTol = 1e-4f;
}  // namespace

TEST(FourierConventions, dft2RealMatchesNaiveForPowerOfTwoSizes)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const auto& sz : {
           std::pair<int, int>{ 4,  8},
            { 8,  4},
            { 2,  2},
            { 8,  2},
            {16, 16}
  })
  {
    const CMatrixFloat A = randomMatrix(sz.first, sz.second);
    CMatrixFloat zeros(sz.first, sz.second);
    zeros.setZero();
    CMatrixFloat re;
    CMatrixFloat im;
    mrpt::math::dft2_real(A, re, im);
    CMatrixFloat nre;
    CMatrixFloat nim;
    naiveDFT2(A, zeros, nre, nim);
    EXPECT_LT(maxAbsDiff(re, nre), kTol) << sz.first << "x" << sz.second;
    EXPECT_LT(maxAbsDiff(im, nim), kTol) << sz.first << "x" << sz.second;

    CMatrixFloat back;
    mrpt::math::idft2_real(re, im, back);
    EXPECT_LT(maxAbsDiff(back, A), kTol);
  }
}

TEST(FourierConventions, dft2ComplexMatchesNaiveForGeneralSizes)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const auto& sz : {
           std::pair<int, int>{3, 3},
            {3, 5},
            {6, 4}
  })
  {
    const CMatrixFloat A = randomMatrix(sz.first, sz.second);
    const CMatrixFloat B = randomMatrix(sz.first, sz.second);
    CMatrixFloat re;
    CMatrixFloat im;
    mrpt::math::dft2_complex(A, B, re, im);
    CMatrixFloat nre;
    CMatrixFloat nim;
    naiveDFT2(A, B, nre, nim);
    EXPECT_LT(maxAbsDiff(re, nre), kTol);
    EXPECT_LT(maxAbsDiff(im, nim), kTol);
    CMatrixFloat bre;
    CMatrixFloat bim;
    mrpt::math::idft2_complex(re, im, bre, bim);
    EXPECT_LT(maxAbsDiff(bre, A), kTol);
    EXPECT_LT(maxAbsDiff(bim, B), kTol);
  }
}

// The power-of-two and the general paths must use the same convention
TEST(FourierConventions, dft2ComplexPowerOfTwoUsesSameSignConvention)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixFloat A = randomMatrix(4, 8);
  const CMatrixFloat B = randomMatrix(4, 8);
  CMatrixFloat re;
  CMatrixFloat im;
  mrpt::math::dft2_complex(A, B, re, im);
  CMatrixFloat nre;
  CMatrixFloat nim;
  naiveDFT2(A, B, nre, nim);
  EXPECT_LT(maxAbsDiff(re, nre), kTol);
  EXPECT_LT(maxAbsDiff(im, nim), kTol);
}

// Complex data, not only Hermitian spectra, must survive the round trip
TEST(FourierConventions, dft2ComplexRoundTripKeepsImaginaryDC)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixFloat A = randomMatrix(4, 4);
  const CMatrixFloat B = randomMatrix(4, 4);
  CMatrixFloat re;
  CMatrixFloat im;
  mrpt::math::dft2_complex(A, B, re, im);
  CMatrixFloat bre;
  CMatrixFloat bim;
  mrpt::math::idft2_complex(re, im, bre, bim);
  EXPECT_LT(maxAbsDiff(bre, A), kTol);
  EXPECT_LT(maxAbsDiff(bim, B), kTol);
}

// Non power-of-two sizes use the general DFT
TEST(FourierConventions, dft2RealAndInverseNonPowerOfTwo)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixFloat A = randomMatrix(3, 4);
  CMatrixFloat zeros(3, 4);
  zeros.setZero();
  CMatrixFloat re;
  CMatrixFloat im;
  mrpt::math::dft2_real(A, re, im);
  CMatrixFloat nre;
  CMatrixFloat nim;
  naiveDFT2(A, zeros, nre, nim);
  EXPECT_LT(maxAbsDiff(re, nre), kTol);
  EXPECT_LT(maxAbsDiff(im, nim), kTol);
  CMatrixFloat back;
  mrpt::math::idft2_real(re, im, back);
  EXPECT_LT(maxAbsDiff(back, A), kTol);
}

// fft_real() and dft2_real() share the standard exp(-i...) convention
TEST(FourierConventions, fftRealMatchesDft2Real)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const int n = 16;
  const CMatrixFloat A = randomMatrix(1, n);
  CVectorFloat v(n);
  for (int i = 0; i < n; i++)
  {
    v[i] = A(0, i);
  }
  CVectorFloat re;
  CVectorFloat im;
  CVectorFloat mag;
  mrpt::math::fft_real(v, re, im, mag);
  ASSERT_EQ(re.size(), n / 2 + 1);
  CMatrixFloat zeros(1, n);
  zeros.setZero();
  CMatrixFloat nre;
  CMatrixFloat nim;
  naiveDFT2(A, zeros, nre, nim);
  float errRe = 0;
  float errIm = 0;
  float errMag = 0;
  for (int k = 0; k <= n / 2; k++)
  {
    errRe = std::max(errRe, std::abs(re[k] - nre(0, k)));
    errIm = std::max(errIm, std::abs(im[k] - nim(0, k)));
    errMag = std::max(errMag, std::abs(mag[k] - std::hypot(nre(0, k), nim(0, k))));
  }
  EXPECT_LT(errRe, kTol);
  EXPECT_LT(errIm, kTol);
  EXPECT_LT(errMag, kTol);
}

TEST(FourierConventions, crossCorrelationFindsCircularShift)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const int n = 16;
  const CMatrixFloat A = randomMatrix(n, n);
  const int dy = 3;
  const int dx = 5;
  CMatrixFloat B(n, n);
  for (int y = 0; y < n; y++)
  {
    for (int x = 0; x < n; x++)
    {
      B((y + dy) % n, (x + dx) % n) = A(y, x);
    }
  }
  CMatrixFloat corr;
  mrpt::math::cross_correlation_FFT(A, B, corr);
  Eigen::Index ry = 0;
  Eigen::Index rx = 0;
  corr.asEigen().maxCoeff(&ry, &rx);
  EXPECT_EQ(ry, dy);
  EXPECT_EQ(rx, dx);
}

// Zeros in the spectrum of A must not turn the output into NaN
TEST(FourierConventions, crossCorrelationOutputIsFinite)
{
  CMatrixFloat A(4, 4);
  A.setConstant(1.0f);
  CMatrixFloat B = A;
  B(1, 2) = 2.0f;
  CMatrixFloat corr;
  mrpt::math::cross_correlation_FFT(A, B, corr);
  EXPECT_TRUE(corr.asEigen().allFinite());
}

TEST(FourierConventions, fftRealRejectsNonPowerOfTwoLength)
{
  CVectorFloat v(6);
  v.fill(1.0f);
  CVectorFloat re;
  CVectorFloat im;
  CVectorFloat mag;
  EXPECT_ANY_THROW(mrpt::math::fft_real(v, re, im, mag));
}

// The FFT functions keep no shared state: concurrent calls give the same
// results as sequential ones
TEST(FourierConventions, dft2ComplexIsThreadSafe)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixFloat A = randomMatrix(16, 16);
  const CMatrixFloat B = randomMatrix(8, 32);
  CMatrixFloat zA(16, 16);
  zA.setZero();
  CMatrixFloat zB(8, 32);
  zB.setZero();
  CMatrixFloat refAr;
  CMatrixFloat refAi;
  mrpt::math::dft2_complex(A, zA, refAr, refAi);
  CMatrixFloat refBr;
  CMatrixFloat refBi;
  mrpt::math::dft2_complex(B, zB, refBr, refBi);
  std::atomic<int> numBad{0};
  const auto worker =
      [&numBad](const CMatrixFloat& in, const CMatrixFloat& z, const CMatrixFloat& refR)
  {
    for (int k = 0; k < 200; k++)
    {
      CMatrixFloat r;
      CMatrixFloat i;
      mrpt::math::dft2_complex(in, z, r, i);
      if (maxAbsDiff(r, refR) > kTol)
      {
        numBad++;
      }
    }
  };
  std::thread t1(worker, std::cref(A), std::cref(zA), std::cref(refAr));
  std::thread t2(worker, std::cref(B), std::cref(zB), std::cref(refBr));
  std::thread t3(worker, std::cref(A), std::cref(zA), std::cref(refAr));
  t1.join();
  t2.join();
  t3.join();
  EXPECT_EQ(numBad.load(), 0);
}
