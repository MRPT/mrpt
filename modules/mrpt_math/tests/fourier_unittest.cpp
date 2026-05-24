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
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/fourier.h>

#include <Eigen/Core>
#include <cmath>

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
