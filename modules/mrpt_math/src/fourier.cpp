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

#include <mrpt/core/bits_math.h>
#include <mrpt/math/fourier.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace mrpt;
using namespace mrpt::math;

namespace
{
// Type definitions
using FftType = float;

// Constants
constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 6.28318530717959;

/**
 * @brief Bit-reversal permutation for FFT
 *
 * Performs in-place bit-reversal of complex array elements.
 * This is required for the Cooley-Tukey FFT algorithm.
 *
 * @param numPoints Number of complex values (array length = 2*numPoints)
 * @param data Complex array [real0, imag0, real1, imag1, ...]
 */
void bitReversal(size_t numPoints, float data[])
{
  size_t j = 1;
  const size_t n = numPoints << 1;

  for (size_t i = 1; i < n; i += 2)
  {
    if (j > i)
    {
      std::swap(data[j], data[i]);
      std::swap(data[j + 1], data[i + 1]);
    }

    size_t m = numPoints;
    while (m >= 2 && j > m)
    {
      j -= m;
      m >>= 1;
    }
    j += m;
  }
}

/**
 * @brief Cooley-Tukey FFT (radix-2 decimation-in-time)
 *
 * Computes FFT or inverse FFT of complex data.
 * Data must be bit-reversed before calling (unless already in correct order).
 *
 * @param data Complex array [real0, imag0, real1, imag1, ...], length 2*numPoints
 * @param numPoints Number of complex values (must be power of 2)
 * @param isign +1 for forward FFT, -1 for inverse FFT
 */
void four1(float data[], size_t numPoints, int isign)
{
  const size_t n = numPoints << 1;
  size_t j = 1;

  // Bit-reversal section
  for (size_t i = 1; i < n; i += 2)
  {
    if (j > i)
    {
      std::swap(data[j], data[i]);
      std::swap(data[j + 1], data[i + 1]);
    }
    size_t m = numPoints;
    while (m >= 2 && j > m)
    {
      j -= m;
      m >>= 1;
    }
    j += m;
  }

  // Danielson-Lanczos section
  size_t mmax = 2;
  while (n > mmax)
  {
    const size_t istep = mmax << 1;
    const double theta = static_cast<double>(isign) * (kTwoPi / static_cast<double>(mmax));
    const double wtemp = std::sin(0.5 * theta);
    const double wpr = -2.0 * wtemp * wtemp;
    const double wpi = std::sin(theta);
    double wr = 1.0;
    double wi = 0.0;

    for (size_t m = 1; m < mmax; m += 2)
    {
      for (size_t i = m; i <= n; i += istep)
      {
        const size_t j = i + mmax;
        const float tempr = static_cast<float>(wr * data[j] - wi * data[j + 1]);
        const float tempi = static_cast<float>(wr * data[j + 1] + wi * data[j]);
        data[j] = data[i] - tempr;
        data[j + 1] = data[i + 1] - tempi;
        data[i] += tempr;
        data[i + 1] += tempi;
      }
      const double wrOld = wr;
      wr = wrOld * wpr - wi * wpi + wr;
      wi = wi * wpr + wrOld * wpi + wi;
    }
    mmax = istep;
  }
}

/**
 * @brief Real FFT using complex FFT
 *
 * Computes FFT of real-valued data or inverse FFT.
 * For forward transform: input is real data, output is half-complex format.
 * For inverse transform: input is half-complex format, output is real data.
 *
 * @param data Real data array (length n), modified in-place
 * @param n Number of real values (must be power of 2)
 */
void realft(float data[], size_t n)
{
  const double theta = kPi / static_cast<double>(n >> 1);
  constexpr float c1 = 0.5f;
  constexpr float c2 = -0.5f;

  // Forward transform
  four1(data, n >> 1, 1);

  const double wtemp = std::sin(0.5 * theta);
  const double wpr = -2.0 * wtemp * wtemp;
  const double wpi = std::sin(theta);
  double wr = 1.0 + wpr;
  double wi = wpi;
  const size_t np3 = n + 3;

  for (size_t i = 2; i <= (n >> 2); i++)
  {
    const size_t i1 = i + i - 1;
    const size_t i2 = 1 + i1;
    const size_t i3 = np3 - i2;
    const size_t i4 = 1 + i3;

    const float h1r = c1 * (data[i1] + data[i3]);
    const float h1i = c1 * (data[i2] - data[i4]);
    const float h2r = -c2 * (data[i2] + data[i4]);
    const float h2i = c2 * (data[i1] - data[i3]);

    data[i1] = static_cast<float>(h1r + wr * h2r - wi * h2i);
    data[i2] = static_cast<float>(h1i + wr * h2i + wi * h2r);
    data[i3] = static_cast<float>(h1r - wr * h2r + wi * h2i);
    data[i4] = static_cast<float>(-h1i + wr * h2i + wi * h2r);

    const double wrOld = wr;
    wr = wrOld * wpr - wi * wpi + wr;
    wi = wi * wpr + wrOld * wpi + wi;
  }

  const float h1r = data[1];
  data[1] = h1r + data[2];
  data[2] = h1r - data[2];
}

// ============================================================================
// Ooura FFT routines (Copyright(C) 1997 Takuya OOURA)
// You may use, copy, modify this code for any purpose without fee.
// ============================================================================

void makewt(int nw, int* ip, FftType* w);
void bitrv2(int n, int* ip, FftType* a);
void cftbsub(int n, FftType* a, FftType* w);
void cftfsub(int n, FftType* a, FftType* w);
void rftfsub(int n, FftType* a, int nc, FftType* c);
void rftbsub(int n, FftType* a, int nc, FftType* c);

/**
 * @brief Complex FFT (backward transform)
 */
void cftbsub(int n, FftType* a, FftType* w)
{
  int l = 2;
  while ((l << 1) < n)
  {
    const int m = l << 2;
    for (int j = 0; j <= l - 2; j += 2)
    {
      const int j1 = j + l;
      const int j2 = j1 + l;
      const int j3 = j2 + l;
      const FftType x0r = a[j] + a[j1];
      const FftType x0i = a[j + 1] + a[j1 + 1];
      const FftType x1r = a[j] - a[j1];
      const FftType x1i = a[j + 1] - a[j1 + 1];
      const FftType x2r = a[j2] + a[j3];
      const FftType x2i = a[j2 + 1] + a[j3 + 1];
      const FftType x3r = a[j2] - a[j3];
      const FftType x3i = a[j2 + 1] - a[j3 + 1];
      a[j] = x0r + x2r;
      a[j + 1] = x0i + x2i;
      a[j2] = x0r - x2r;
      a[j2 + 1] = x0i - x2i;
      a[j1] = x1r - x3i;
      a[j1 + 1] = x1i + x3r;
      a[j3] = x1r + x3i;
      a[j3 + 1] = x1i - x3r;
    }
    if (m < n)
    {
      const FftType wk1r = w[2];
      for (int j = m; j <= l + m - 2; j += 2)
      {
        const int j1 = j + l;
        const int j2 = j1 + l;
        const int j3 = j2 + l;
        const FftType x0r = a[j] + a[j1];
        const FftType x0i = a[j + 1] + a[j1 + 1];
        const FftType x1r = a[j] - a[j1];
        const FftType x1i = a[j + 1] - a[j1 + 1];
        const FftType x2r = a[j2] + a[j3];
        const FftType x2i = a[j2 + 1] + a[j3 + 1];
        const FftType x3r = a[j2] - a[j3];
        const FftType x3i = a[j2 + 1] - a[j3 + 1];
        a[j] = x0r + x2r;
        a[j + 1] = x0i + x2i;
        a[j2] = x2i - x0i;
        a[j2 + 1] = x0r - x2r;
        FftType x0rTemp = x1r - x3i;
        FftType x0iTemp = x1i + x3r;
        a[j1] = wk1r * (x0rTemp - x0iTemp);
        a[j1 + 1] = wk1r * (x0rTemp + x0iTemp);
        x0rTemp = x3i + x1r;
        x0iTemp = x3r - x1i;
        a[j3] = wk1r * (x0iTemp - x0rTemp);
        a[j3 + 1] = wk1r * (x0iTemp + x0rTemp);
      }
      int k1 = 1;
      int ks = -1;
      for (int k = (m << 1); k <= n - m; k += m)
      {
        k1++;
        ks = -ks;
        const FftType wk1r = w[k1 << 1];
        const FftType wk1i = w[(k1 << 1) + 1];
        const FftType wk2r = static_cast<FftType>(ks) * w[k1];
        const FftType wk2i = w[k1 + ks];
        const FftType wk3r = wk1r - 2.0f * wk2i * wk1i;
        const FftType wk3i = 2.0f * wk2i * wk1r - wk1i;
        for (int j = k; j <= l + k - 2; j += 2)
        {
          const int j1 = j + l;
          const int j2 = j1 + l;
          const int j3 = j2 + l;
          const FftType x0r = a[j] + a[j1];
          const FftType x0i = a[j + 1] + a[j1 + 1];
          const FftType x1r = a[j] - a[j1];
          const FftType x1i = a[j + 1] - a[j1 + 1];
          const FftType x2r = a[j2] + a[j3];
          const FftType x2i = a[j2 + 1] + a[j3 + 1];
          const FftType x3r = a[j2] - a[j3];
          const FftType x3i = a[j2 + 1] - a[j3 + 1];
          a[j] = x0r + x2r;
          a[j + 1] = x0i + x2i;
          FftType x0rTemp = x0r - x2r;
          FftType x0iTemp = x0i - x2i;
          a[j2] = wk2r * x0rTemp - wk2i * x0iTemp;
          a[j2 + 1] = wk2r * x0iTemp + wk2i * x0rTemp;
          x0rTemp = x1r - x3i;
          x0iTemp = x1i + x3r;
          a[j1] = wk1r * x0rTemp - wk1i * x0iTemp;
          a[j1 + 1] = wk1r * x0iTemp + wk1i * x0rTemp;
          x0rTemp = x1r + x3i;
          x0iTemp = x1i - x3r;
          a[j3] = wk3r * x0rTemp - wk3i * x0iTemp;
          a[j3 + 1] = wk3r * x0iTemp + wk3i * x0rTemp;
        }
      }
    }
    l = m;
  }
  if (l < n)
  {
    for (int j = 0; j <= l - 2; j += 2)
    {
      const int j1 = j + l;
      const FftType x0r = a[j] - a[j1];
      const FftType x0i = a[j + 1] - a[j1 + 1];
      a[j] += a[j1];
      a[j + 1] += a[j1 + 1];
      a[j1] = x0r;
      a[j1 + 1] = x0i;
    }
  }
}

/**
 * @brief Complex FFT (forward transform)
 */
void cftfsub(int n, FftType* a, FftType* w)
{
  int l = 2;
  while ((l << 1) < n)
  {
    const int m = l << 2;
    for (int j = 0; j <= l - 2; j += 2)
    {
      const int j1 = j + l;
      const int j2 = j1 + l;
      const int j3 = j2 + l;
      const FftType x0r = a[j] + a[j1];
      const FftType x0i = a[j + 1] + a[j1 + 1];
      const FftType x1r = a[j] - a[j1];
      const FftType x1i = a[j + 1] - a[j1 + 1];
      const FftType x2r = a[j2] + a[j3];
      const FftType x2i = a[j2 + 1] + a[j3 + 1];
      const FftType x3r = a[j2] - a[j3];
      const FftType x3i = a[j2 + 1] - a[j3 + 1];
      a[j] = x0r + x2r;
      a[j + 1] = x0i + x2i;
      a[j2] = x0r - x2r;
      a[j2 + 1] = x0i - x2i;
      a[j1] = x1r + x3i;
      a[j1 + 1] = x1i - x3r;
      a[j3] = x1r - x3i;
      a[j3 + 1] = x1i + x3r;
    }
    if (m < n)
    {
      const FftType wk1r = w[2];
      for (int j = m; j <= l + m - 2; j += 2)
      {
        const int j1 = j + l;
        const int j2 = j1 + l;
        const int j3 = j2 + l;
        const FftType x0r = a[j] + a[j1];
        const FftType x0i = a[j + 1] + a[j1 + 1];
        const FftType x1r = a[j] - a[j1];
        const FftType x1i = a[j + 1] - a[j1 + 1];
        const FftType x2r = a[j2] + a[j3];
        const FftType x2i = a[j2 + 1] + a[j3 + 1];
        const FftType x3r = a[j2] - a[j3];
        const FftType x3i = a[j2 + 1] - a[j3 + 1];
        a[j] = x0r + x2r;
        a[j + 1] = x0i + x2i;
        a[j2] = x0i - x2i;
        a[j2 + 1] = x2r - x0r;
        FftType x0rTemp = x1r + x3i;
        FftType x0iTemp = x1i - x3r;
        a[j1] = wk1r * (x0iTemp + x0rTemp);
        a[j1 + 1] = wk1r * (x0iTemp - x0rTemp);
        x0rTemp = x3i - x1r;
        x0iTemp = x3r + x1i;
        a[j3] = wk1r * (x0rTemp + x0iTemp);
        a[j3 + 1] = wk1r * (x0rTemp - x0iTemp);
      }
      int k1 = 1;
      int ks = -1;
      for (int k = (m << 1); k <= n - m; k += m)
      {
        k1++;
        ks = -ks;
        const FftType wk1r = w[k1 << 1];
        const FftType wk1i = w[(k1 << 1) + 1];
        const FftType wk2r = static_cast<FftType>(ks) * w[k1];
        const FftType wk2i = w[k1 + ks];
        const FftType wk3r = wk1r - 2.0f * wk2i * wk1i;
        const FftType wk3i = 2.0f * wk2i * wk1r - wk1i;
        for (int j = k; j <= l + k - 2; j += 2)
        {
          const int j1 = j + l;
          const int j2 = j1 + l;
          const int j3 = j2 + l;
          const FftType x0r = a[j] + a[j1];
          const FftType x0i = a[j + 1] + a[j1 + 1];
          const FftType x1r = a[j] - a[j1];
          const FftType x1i = a[j + 1] - a[j1 + 1];
          const FftType x2r = a[j2] + a[j3];
          const FftType x2i = a[j2 + 1] + a[j3 + 1];
          const FftType x3r = a[j2] - a[j3];
          const FftType x3i = a[j2 + 1] - a[j3 + 1];
          a[j] = x0r + x2r;
          a[j + 1] = x0i + x2i;
          FftType x0rTemp = x0r - x2r;
          FftType x0iTemp = x0i - x2i;
          a[j2] = wk2r * x0rTemp + wk2i * x0iTemp;
          a[j2 + 1] = wk2r * x0iTemp - wk2i * x0rTemp;
          x0rTemp = x1r + x3i;
          x0iTemp = x1i - x3r;
          a[j1] = wk1r * x0rTemp + wk1i * x0iTemp;
          a[j1 + 1] = wk1r * x0iTemp - wk1i * x0rTemp;
          x0rTemp = x1r - x3i;
          x0iTemp = x1i + x3r;
          a[j3] = wk3r * x0rTemp + wk3i * x0iTemp;
          a[j3 + 1] = wk3r * x0iTemp - wk3i * x0rTemp;
        }
      }
    }
    l = m;
  }
  if (l < n)
  {
    for (int j = 0; j <= l - 2; j += 2)
    {
      const int j1 = j + l;
      const FftType x0r = a[j] - a[j1];
      const FftType x0i = a[j + 1] - a[j1 + 1];
      a[j] += a[j1];
      a[j + 1] += a[j1 + 1];
      a[j1] = x0r;
      a[j1 + 1] = x0i;
    }
  }
}

void makewt(int nw, int* ip, FftType* w)
{
  ip[0] = nw;
  ip[1] = 1;
  if (nw > 2)
  {
    const int nwh = nw >> 1;
    const FftType delta = std::atan(1.0f) / static_cast<FftType>(nwh);
    w[0] = 1.0f;
    w[1] = 0.0f;
    w[nwh] = std::cos(delta * static_cast<FftType>(nwh));
    w[nwh + 1] = w[nwh];
    for (int j = 2; j <= nwh - 2; j += 2)
    {
      const FftType x = std::cos(delta * static_cast<FftType>(j));
      const FftType y = std::sin(delta * static_cast<FftType>(j));
      w[j] = x;
      w[j + 1] = y;
      w[nw - j] = y;
      w[nw - j + 1] = x;
    }
    bitrv2(nw, ip + 2, w);
  }
}

void makect(int nc, int* ip, FftType* c)
{
  ip[1] = nc;
  if (nc > 1)
  {
    const int nch = nc >> 1;
    const FftType delta = std::atan(1.0f) / static_cast<FftType>(nch);
    c[0] = 0.5f;
    c[nch] = 0.5f * std::cos(delta * static_cast<FftType>(nch));
    for (int j = 1; j <= nch - 1; j++)
    {
      c[j] = 0.5f * std::cos(delta * static_cast<FftType>(j));
      c[nc - j] = 0.5f * std::sin(delta * static_cast<FftType>(j));
    }
  }
}

void bitrv2(int n, int* ip, FftType* a)
{
  ip[0] = 0;
  int l = n;
  int m = 1;
  while ((m << 2) < l)
  {
    l >>= 1;
    for (int j = 0; j <= m - 1; j++)
    {
      ip[m + j] = ip[j] + l;
    }
    m <<= 1;
  }
  if ((m << 2) > l)
  {
    for (int k = 1; k <= m - 1; k++)
    {
      for (int j = 0; j <= k - 1; j++)
      {
        const int j1 = (j << 1) + ip[k];
        const int k1 = (k << 1) + ip[j];
        const FftType xr = a[j1];
        const FftType xi = a[j1 + 1];
        a[j1] = a[k1];
        a[j1 + 1] = a[k1 + 1];
        a[k1] = xr;
        a[k1 + 1] = xi;
      }
    }
  }
  else
  {
    const int m2 = m << 1;
    for (int k = 1; k <= m - 1; k++)
    {
      for (int j = 0; j <= k - 1; j++)
      {
        int j1 = (j << 1) + ip[k];
        int k1 = (k << 1) + ip[j];
        FftType xr = a[j1];
        FftType xi = a[j1 + 1];
        a[j1] = a[k1];
        a[j1 + 1] = a[k1 + 1];
        a[k1] = xr;
        a[k1 + 1] = xi;
        j1 += m2;
        k1 += m2;
        xr = a[j1];
        xi = a[j1 + 1];
        a[j1] = a[k1];
        a[j1 + 1] = a[k1 + 1];
        a[k1] = xr;
        a[k1 + 1] = xi;
      }
    }
  }
}

void cdft(int n, int isgn, FftType* a, int* ip, FftType* w)
{
  if (n > (ip[0] << 2))
  {
    makewt(n >> 2, ip, w);
  }
  if (n > 4)
  {
    bitrv2(n, ip + 2, a);
  }
  if (isgn < 0)
  {
    cftfsub(n, a, w);
  }
  else
  {
    cftbsub(n, a, w);
  }
}

void rftfsub(int n, FftType* a, int nc, FftType* c)
{
  const int ks = (nc << 2) / n;
  int kk = 0;
  for (int k = (n >> 1) - 2; k >= 2; k -= 2)
  {
    const int j = n - k;
    kk += ks;
    const FftType wkr = 0.5f - c[kk];
    const FftType wki = c[nc - kk];
    const FftType xr = a[k] - a[j];
    const FftType xi = a[k + 1] + a[j + 1];
    const FftType yr = wkr * xr + wki * xi;
    const FftType yi = wkr * xi - wki * xr;
    a[k] -= yr;
    a[k + 1] -= yi;
    a[j] += yr;
    a[j + 1] -= yi;
  }
}

void rftbsub(int n, FftType* a, int nc, FftType* c)
{
  const int ks = (nc << 2) / n;
  int kk = 0;
  for (int k = (n >> 1) - 2; k >= 2; k -= 2)
  {
    const int j = n - k;
    kk += ks;
    const FftType wkr = 0.5f - c[kk];
    const FftType wki = c[nc - kk];
    const FftType xr = a[k] - a[j];
    const FftType xi = a[k + 1] + a[j + 1];
    const FftType yr = wkr * xr - wki * xi;
    const FftType yi = wkr * xi + wki * xr;
    a[k] -= yr;
    a[k + 1] -= yi;
    a[j] += yr;
    a[j + 1] -= yi;
  }
}

void rdft(int n, int isgn, FftType* a, int* ip, FftType* w)
{
  int nw = ip[0];
  if (n > (nw << 2))
  {
    nw = n >> 2;
    makewt(nw, ip, w);
  }
  int nc = ip[1];
  if (n > (nc << 2))
  {
    nc = n >> 2;
    makect(nc, ip, w + nw);
  }
  if (isgn < 0)
  {
    a[1] = 0.5f * (a[0] - a[1]);
    a[0] -= a[1];
    if (n > 4)
    {
      rftfsub(n, a, nc, w + nw);
      bitrv2(n, ip + 2, a);
    }
    cftfsub(n, a, w);
  }
  else
  {
    if (n > 4)
    {
      bitrv2(n, ip + 2, a);
    }
    cftbsub(n, a, w);
    if (n > 4)
    {
      rftbsub(n, a, nc, w + nw);
    }
    const FftType xi = a[0] - a[1];
    a[0] += a[1];
    a[1] = xi;
  }
}

void rdft2d(int n1, int n2, int isgn, FftType** a, FftType* t, int* ip, FftType* w)
{
  int n = n1 << 1;
  if (n < n2)
  {
    n = n2;
  }
  int nw = ip[0];
  if (n > (nw << 2))
  {
    nw = n >> 2;
    makewt(nw, ip, w);
  }
  int nc = ip[1];
  if (n2 > (nc << 2))
  {
    nc = n2 >> 2;
    makect(nc, ip, w + nw);
  }
  const int n1h = n1 >> 1;
  if (isgn < 0)
  {
    for (int i = 1; i <= n1h - 1; i++)
    {
      const int j = n1 - i;
      FftType xi = a[i][0] - a[j][0];
      a[i][0] += a[j][0];
      a[j][0] = xi;
      xi = a[j][1] - a[i][1];
      a[i][1] += a[j][1];
      a[j][1] = xi;
    }
    for (int j = 0; j <= n2 - 2; j += 2)
    {
      for (int i = 0; i <= n1 - 1; i++)
      {
        const int i2 = i << 1;
        t[i2] = a[i][j];
        t[i2 + 1] = a[i][j + 1];
      }
      cdft(n1 << 1, isgn, t, ip, w);
      for (int i = 0; i <= n1 - 1; i++)
      {
        const int i2 = i << 1;
        a[i][j] = t[i2];
        a[i][j + 1] = t[i2 + 1];
      }
    }
    for (int i = 0; i <= n1 - 1; i++)
    {
      rdft(n2, isgn, a[i], ip, w);
    }
  }
  else
  {
    for (int i = 0; i <= n1 - 1; i++)
    {
      rdft(n2, isgn, a[i], ip, w);
    }
    for (int j = 0; j <= n2 - 2; j += 2)
    {
      for (int i = 0; i <= n1 - 1; i++)
      {
        const int i2 = i << 1;
        t[i2] = a[i][j];
        t[i2 + 1] = a[i][j + 1];
      }
      cdft(n1 << 1, isgn, t, ip, w);
      for (int i = 0; i <= n1 - 1; i++)
      {
        const int i2 = i << 1;
        a[i][j] = t[i2];
        a[i][j + 1] = t[i2 + 1];
      }
    }
    for (int i = 1; i <= n1h - 1; i++)
    {
      const int j = n1 - i;
      a[j][0] = 0.5f * (a[i][0] - a[j][0]);
      a[i][0] -= a[j][0];
      a[j][1] = 0.5f * (a[i][1] + a[j][1]);
      a[i][1] -= a[j][1];
    }
  }
}

void cdft2d(int n1, int n2, int isgn, FftType** a, FftType* t, int* ip, FftType* w)
{
  int n = n1 << 1;
  if (n < n2)
  {
    n = n2;
  }
  if (n > (ip[0] << 2))
  {
    makewt(n >> 2, ip, w);
  }
  for (int i = 0; i <= n1 - 1; i++)
  {
    cdft(n2, isgn, a[i], ip, w);
  }
  for (int j = 0; j <= n2 - 2; j += 2)
  {
    for (int i = 0; i <= n1 - 1; i++)
    {
      const int i2 = i << 1;
      t[i2] = a[i][j];
      t[i2 + 1] = a[i][j + 1];
    }
    cdft(n1 << 1, isgn, t, ip, w);
    for (int i = 0; i <= n1 - 1; i++)
    {
      const int i2 = i << 1;
      a[i][j] = t[i2];
      a[i][j + 1] = t[i2 + 1];
    }
  }
}

/**
 * @brief General DFT implementation (non-power-of-2 sizes)
 *
 * @param sign -1 for DFT, +1 for IDFT
 * @param inReal Real part of input
 * @param inImag Imaginary part of input
 * @param outReal Real part of output
 * @param outImag Imaginary part of output
 */
void generalDFT(
    int sign,
    const CMatrixFloat& inReal,
    const CMatrixFloat& inImag,
    CMatrixFloat& outReal,
    CMatrixFloat& outImag)
{
  ASSERT_(inReal.rows() == inImag.rows());
  ASSERT_(inReal.cols() == inImag.cols());

  const size_t dim1 = static_cast<size_t>(inReal.rows());
  const size_t dim2 = static_cast<size_t>(inReal.cols());

  const float ang1 =
      static_cast<float>(sign) * static_cast<float>(M_2PI) / static_cast<float>(dim1);
  const float ang2 =
      static_cast<float>(sign) * static_cast<float>(M_2PI) / static_cast<float>(dim2);
  const float scale = (sign == 1) ? (1.0f / static_cast<float>(dim1 * dim2)) : 1.0f;

  outReal.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));
  outImag.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));

  for (size_t k1 = 0; k1 < dim1; k1++)
  {
    for (size_t k2 = 0; k2 < dim2; k2++)
    {
      float sumReal = 0.0f;
      float sumImag = 0.0f;

      for (size_t n1 = 0; n1 < dim1; n1++)
      {
        float phase = ang1 * static_cast<float>(n1 * k1);
        for (size_t n2 = 0; n2 < dim2; n2++)
        {
          const float wr = std::cos(phase);
          const float wi = std::sin(phase);

          sumReal += wr * inReal(static_cast<Eigen::Index>(n1), static_cast<Eigen::Index>(n2)) -
                     wi * inImag(static_cast<Eigen::Index>(n1), static_cast<Eigen::Index>(n2));
          sumImag += wi * inReal(static_cast<Eigen::Index>(n1), static_cast<Eigen::Index>(n2)) +
                     wr * inImag(static_cast<Eigen::Index>(n1), static_cast<Eigen::Index>(n2));

          phase += ang2 * static_cast<float>(k2);
        }
      }

      outReal(static_cast<Eigen::Index>(k1), static_cast<Eigen::Index>(k2)) = sumReal * scale;
      outImag(static_cast<Eigen::Index>(k1), static_cast<Eigen::Index>(k2)) = sumImag * scale;
    }
  }
}

}  // anonymous namespace

// ============================================================================
// Public API implementations
// ============================================================================

namespace mrpt::math
{
/**
 * @brief Computes real FFT of input data
 *
 * @param inRealData Input real-valued signal (must be power of 2 length)
 * @param outFftRe Real part of FFT output
 * @param outFftIm Imaginary part of FFT output
 * @param outFftMag Magnitude of FFT output
 */
void fft_real(
    CVectorFloat& inRealData,
    CVectorFloat& outFftRe,
    CVectorFloat& outFftIm,
    CVectorFloat& outFftMag)
{
  MRPT_START

  const size_t n = static_cast<size_t>(inRealData.size());

  // Create auxiliary vector (1-indexed for compatibility with legacy code)
  CVectorFloat auxVect(n + 1);
  std::copy(inRealData.begin(), inRealData.end(), auxVect.begin() + 1);

  realft(&auxVect[0], n);

  const size_t n2 = 1 + (n / 2);

  outFftRe.resize(static_cast<Eigen::Index>(n2));
  outFftIm.resize(static_cast<Eigen::Index>(n2));
  outFftMag.resize(static_cast<Eigen::Index>(n2));

  for (size_t i = 0; i < n2; i++)
  {
    const auto idx = static_cast<Eigen::Index>(i);

    if (i == (n2 - 1))
      outFftRe[idx] = auxVect[2];
    else
      outFftRe[idx] = auxVect[1 + 2 * i];

    if (i == 0 || i == (n2 - 1))
      outFftIm[idx] = 0.0f;
    else
      outFftIm[idx] = auxVect[1 + 2 * i + 1];

    outFftMag[idx] = std::sqrt(mrpt::square(outFftRe[idx]) + mrpt::square(outFftIm[idx]));
  }

  MRPT_END
}

/**
 * @brief 2D real DFT
 *
 * @param inData Input 2D real data (must be power of 2 dimensions)
 * @param outReal Real part of output
 * @param outImag Imaginary part of output
 */
void dft2_real(const CMatrixFloat& inData, CMatrixFloat& outReal, CMatrixFloat& outImag)
{
  MRPT_START

  using FloatPtr = FftType*;

  const size_t dim1 = static_cast<size_t>(inData.rows());
  const size_t dim2 = static_cast<size_t>(inData.cols());

  // Allocate arrays
  std::vector<FloatPtr> a(dim1);
  for (size_t i = 0; i < dim1; i++)
  {
    a[i] = new FftType[dim2];
    for (size_t j = 0; j < dim2; j++)
    {
      a[i][j] = inData(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
    }
  }

  const size_t maxDim = std::max(dim1, dim2 / 2);
  std::vector<FftType> t(2 * dim1 + 20);
  std::vector<int> ip(
      static_cast<size_t>(std::ceil(20 + 2 + std::sqrt(static_cast<double>(maxDim)))));
  ip[0] = 0;
  std::vector<FftType> w(std::max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20);

  // Compute 2D real DFT
  rdft2d(
      static_cast<int>(dim1), static_cast<int>(dim2), 1, a.data(), t.data(), ip.data(), w.data());

  // Extract results
  outReal.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));
  outImag.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));

  // Pack results according to format
  for (size_t i = 1; i < dim1; i++)
  {
    for (size_t j = 1; j < dim2 / 2; j++)
    {
      const auto i_idx = static_cast<Eigen::Index>(i);
      const auto j_idx = static_cast<Eigen::Index>(j);
      const auto i_sym = static_cast<Eigen::Index>(dim1 - i);
      const auto j_sym = static_cast<Eigen::Index>(dim2 - j);

      outReal(i_idx, j_idx) = a[i][j * 2];
      outReal(i_sym, j_sym) = a[i][j * 2];
      outImag(i_idx, j_idx) = -a[i][j * 2 + 1];
      outImag(i_sym, j_sym) = a[i][j * 2 + 1];
    }
  }

  for (size_t j = 1; j < dim2 / 2; j++)
  {
    const auto j_idx = static_cast<Eigen::Index>(j);
    const auto j_sym = static_cast<Eigen::Index>(dim2 - j);

    outReal(0, j_idx) = a[0][j * 2];
    outReal(0, j_sym) = a[0][j * 2];
    outImag(0, j_idx) = -a[0][j * 2 + 1];
    outImag(0, j_sym) = a[0][j * 2 + 1];
  }

  for (size_t i = 1; i < dim1 / 2; i++)
  {
    const auto i_idx = static_cast<Eigen::Index>(i);
    const auto i_sym = static_cast<Eigen::Index>(dim1 - i);
    const auto half_dim2 = static_cast<Eigen::Index>(dim2 / 2);

    outReal(i_idx, 0) = a[i][0];
    outReal(i_sym, 0) = a[i][0];
    outImag(i_idx, 0) = -a[i][1];
    outImag(i_sym, 0) = a[i][1];
    outReal(i_idx, half_dim2) = a[dim1 - i][1];
    outReal(i_sym, half_dim2) = a[dim1 - i][1];
    outImag(i_idx, half_dim2) = a[dim1 - i][0];
    outImag(i_sym, half_dim2) = -a[dim1 - i][0];
  }

  outReal(0, 0) = a[0][0];
  outReal(0, static_cast<Eigen::Index>(dim2 / 2)) = a[0][1];
  outReal(static_cast<Eigen::Index>(dim1 / 2), 0) = a[dim1 / 2][0];
  outReal(static_cast<Eigen::Index>(dim1 / 2), static_cast<Eigen::Index>(dim2 / 2)) =
      a[dim1 / 2][1];

  // Free memory
  for (size_t i = 0; i < dim1; i++) delete[] a[i];

  MRPT_END
}

/**
 * @brief 2D inverse real DFT
 */
void idft2_real(const CMatrixFloat& inReal, const CMatrixFloat& inImag, CMatrixFloat& outData)
{
  MRPT_START

  using FloatPtr = FftType*;

  ASSERT_(inReal.rows() == inImag.rows());
  ASSERT_(inReal.cols() == inImag.cols());

  const auto dim1 = static_cast<size_t>(inReal.rows());
  const auto dim2 = static_cast<size_t>(inReal.cols());

  if (mrpt::round2up(dim1) != dim1 || mrpt::round2up(dim2) != dim2)
  {
    THROW_EXCEPTION("Matrix sizes are not a power of two!");
  }

  // Allocate arrays
  std::vector<FloatPtr> a(dim1);
  for (size_t i = 0; i < dim1; i++) a[i] = new FftType[dim2];

  // Pack input data
  for (size_t i = 1; i < dim1; i++)
  {
    for (size_t j = 1; j < dim2 / 2; j++)
    {
      a[i][2 * j] = inReal(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      a[i][2 * j + 1] = -inImag(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
    }
  }

  for (size_t j = 1; j < dim2 / 2; j++)
  {
    a[0][2 * j] = inReal(0, static_cast<Eigen::Index>(j));
    a[0][2 * j + 1] = -inImag(0, static_cast<Eigen::Index>(j));
  }

  for (size_t i = 1; i < dim1 / 2; i++)
  {
    const auto i_idx = static_cast<Eigen::Index>(i);
    const auto half_dim2 = static_cast<Eigen::Index>(dim2 / 2);

    a[i][0] = inReal(i_idx, 0);
    a[i][1] = -inImag(i_idx, 0);
    a[dim1 - i][1] = inReal(i_idx, half_dim2);
    a[dim1 - i][0] = inImag(i_idx, half_dim2);
  }

  a[0][0] = inReal(0, 0);
  a[0][1] = inReal(0, static_cast<Eigen::Index>(dim2 / 2));
  a[dim1 / 2][0] = inReal(static_cast<Eigen::Index>(dim1 / 2), 0);
  a[dim1 / 2][1] = inReal(static_cast<Eigen::Index>(dim1 / 2), static_cast<Eigen::Index>(dim2 / 2));

  const size_t maxDim = std::max(dim1, dim2 / 2);
  std::vector<FftType> t(2 * dim1 + 20);
  std::vector<int> ip(
      static_cast<size_t>(std::ceil(20 + 2 + std::sqrt(static_cast<double>(maxDim)))));
  ip[0] = 0;
  std::vector<FftType> w(std::max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20);

  // Compute inverse 2D real DFT
  rdft2d(
      static_cast<int>(dim1), static_cast<int>(dim2), -1, a.data(), t.data(), ip.data(), w.data());

  // Extract results with scaling
  outData.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));
  const FftType scale = 2.0f / static_cast<FftType>(dim1 * dim2);

  for (size_t i = 0; i < dim1; i++)
  {
    for (size_t j = 0; j < dim2; j++)
    {
      outData(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = a[i][j] * scale;
    }
  }

  // Free memory
  for (size_t i = 0; i < dim1; i++) delete[] a[i];

  MRPT_END
}

/**
 * @brief 2D complex DFT
 */
void dft2_complex(
    const CMatrixFloat& inReal,
    const CMatrixFloat& inImag,
    CMatrixFloat& outReal,
    CMatrixFloat& outImag)
{
  MRPT_START

  ASSERT_(inReal.rows() == inImag.rows());
  ASSERT_(inReal.cols() == inImag.cols());

  const size_t dim1 = static_cast<size_t>(inReal.rows());
  const size_t dim2 = static_cast<size_t>(inReal.cols());

  const bool dim1IsPower2 = (mrpt::round2up(dim1) == dim1);
  const bool dim2IsPower2 = (mrpt::round2up(dim2) == dim2);

  if (dim1IsPower2 && dim2IsPower2)
  {
    // Use optimized FFT
    using FloatPtr = FftType*;

    static std::vector<FloatPtr> a;
    static std::vector<FftType> t;
    static std::vector<int> ip;
    static std::vector<FftType> w;
    static size_t lastDim1 = 0;
    static size_t lastDim2 = 0;

    if (lastDim1 != dim1 || lastDim2 != dim2)
    {
      // Reallocate buffers
      for (auto& row : a) delete[] row;
      a.clear();

      a.resize(dim1);
      for (size_t i = 0; i < dim1; i++) a[i] = new FftType[2 * dim2];

      const size_t maxDim = std::max(dim1, dim2 / 2);
      t.resize(2 * dim1 + 20);
      ip.resize(static_cast<size_t>(std::ceil(20 + 2 + std::sqrt(static_cast<double>(maxDim)))));
      ip[0] = 0;
      w.resize(std::max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20);

      lastDim1 = dim1;
      lastDim2 = dim2;
    }

    // Copy data
    for (size_t i = 0; i < dim1; i++)
    {
      for (size_t j = 0; j < dim2; j++)
      {
        a[i][2 * j + 0] = inReal(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
        a[i][2 * j + 1] = inImag(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      }
    }

    // Compute 2D complex DFT
    cdft2d(
        static_cast<int>(dim1), static_cast<int>(2 * dim2), 1, a.data(), t.data(), ip.data(),
        w.data());

    // Extract results
    outReal.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));
    outImag.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));

    for (size_t i = 0; i < dim1; i++)
    {
      for (size_t j = 0; j < dim2; j++)
      {
        outReal(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = a[i][j * 2 + 0];
        outImag(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = a[i][j * 2 + 1];
      }
    }
  }
  else
  {
    // Use general DFT
    generalDFT(-1, inReal, inImag, outReal, outImag);
  }

  MRPT_END
}

/**
 * @brief 2D inverse complex DFT
 */
void idft2_complex(
    const CMatrixFloat& inReal,
    const CMatrixFloat& inImag,
    CMatrixFloat& outReal,
    CMatrixFloat& outImag)
{
  MRPT_START

  ASSERT_(inReal.rows() == inImag.rows());
  ASSERT_(inReal.cols() == inImag.cols());

  const size_t dim1 = static_cast<size_t>(inReal.rows());
  const size_t dim2 = static_cast<size_t>(inReal.cols());

  const bool dim1IsPower2 = (mrpt::round2up(dim1) == dim1);
  const bool dim2IsPower2 = (mrpt::round2up(dim2) == dim2);

  if (dim1IsPower2 && dim2IsPower2)
  {
    // Use optimized FFT
    using FloatPtr = FftType*;

    static std::vector<FloatPtr> a;
    static std::vector<FftType> t;
    static std::vector<int> ip;
    static std::vector<FftType> w;
    static size_t lastDim1 = 0;
    static size_t lastDim2 = 0;

    if (lastDim1 != dim1 || lastDim2 != dim2)
    {
      // Reallocate buffers
      for (auto& row : a) delete[] row;
      a.clear();

      a.resize(dim1);
      for (size_t i = 0; i < dim1; i++) a[i] = new FftType[2 * dim2];

      const size_t maxDim = std::max(dim1, dim2 / 2);
      t.resize(2 * dim1 + 20);
      ip.resize(static_cast<size_t>(std::ceil(20 + 2 + std::sqrt(static_cast<double>(maxDim)))));
      ip[0] = 0;
      w.resize(std::max(dim1 / 2, dim2 / 4) + dim2 / 4 + 20);

      lastDim1 = dim1;
      lastDim2 = dim2;
    }

    // Copy data
    for (size_t i = 0; i < dim1; i++)
    {
      for (size_t j = 0; j < dim2; j++)
      {
        a[i][2 * j + 0] = inReal(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
        a[i][2 * j + 1] = inImag(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
      }
    }

    // Compute inverse 2D complex DFT
    cdft2d(
        static_cast<int>(dim1), static_cast<int>(2 * dim2), -1, a.data(), t.data(), ip.data(),
        w.data());

    // Extract results with scaling
    outReal.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));
    outImag.setSize(static_cast<Eigen::Index>(dim1), static_cast<Eigen::Index>(dim2));

    const FftType scale = 1.0f / static_cast<FftType>(dim1 * dim2);

    for (size_t i = 0; i < dim1; i++)
    {
      for (size_t j = 0; j < dim2; j++)
      {
        outReal(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
            a[i][j * 2 + 0] * scale;
        outImag(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
            a[i][j * 2 + 1] * scale;
      }
    }

    // Element (0,0) is purely real
    outImag(0, 0) = 0.0f;
  }
  else
  {
    // Use general DFT
    generalDFT(1, inReal, inImag, outReal, outImag);
  }

  MRPT_END
}

/**
 * @brief Cross-correlation using FFT
 */
void cross_correlation_FFT(const CMatrixFloat& A, const CMatrixFloat& B, CMatrixFloat& outCorr)
{
  MRPT_START

  ASSERT_(A.cols() == B.cols() && A.rows() == B.rows());

  const size_t lx = static_cast<size_t>(A.cols());
  const size_t ly = static_cast<size_t>(A.rows());

  if (mrpt::round2up(ly) != ly || mrpt::round2up(lx) != lx)
    THROW_EXCEPTION("Size of input matrices must be powers of two.");

  // Compute FFTs
  CMatrixFloat I1Real, I1Imag, I2Real, I2Imag;
  const CMatrixFloat zeros(static_cast<Eigen::Index>(ly), static_cast<Eigen::Index>(lx));

  dft2_complex(A, zeros, I1Real, I1Imag);
  dft2_complex(B, zeros, I2Real, I2Imag);

  // Complex division: I2 / I1
  for (size_t y = 0; y < ly; y++)
  {
    for (size_t x = 0; x < lx; x++)
    {
      const auto y_idx = static_cast<Eigen::Index>(y);
      const auto x_idx = static_cast<Eigen::Index>(x);

      const float r1 = I1Real(y_idx, x_idx);
      const float r2 = I2Real(y_idx, x_idx);
      const float i1 = I1Imag(y_idx, x_idx);
      const float i2 = I2Imag(y_idx, x_idx);

      const float den = mrpt::square(r1) + mrpt::square(i1);
      I2Real(y_idx, x_idx) = (r1 * r2 + i1 * i2) / den;
      I2Imag(y_idx, x_idx) = (i2 * r1 - r2 * i1) / den;
    }
  }

  // Inverse FFT
  CMatrixFloat resReal, resImag;
  idft2_complex(I2Real, I2Imag, resReal, resImag);

  // Compute magnitude
  outCorr.setSize(static_cast<Eigen::Index>(ly), static_cast<Eigen::Index>(lx));
  for (size_t y = 0; y < ly; y++)
  {
    for (size_t x = 0; x < lx; x++)
    {
      const auto y_idx = static_cast<Eigen::Index>(y);
      const auto x_idx = static_cast<Eigen::Index>(x);

      outCorr(y_idx, x_idx) =
          std::sqrt(mrpt::square(resReal(y_idx, x_idx)) + mrpt::square(resImag(y_idx, x_idx)));
    }
  }

  MRPT_END
}

}  // namespace mrpt::math