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
#include <mrpt/math/fresnel.h>
#include <mrpt/system/os.h>

#include <cmath>

TEST(fresnel, fresnelc)
{
  // fresnelc()
  const double test_values[][2] = {
      {                0.0,                 .0},
      {0.79788456080286541,  0.721705924292605},
      {                1.0,  0.779893400376823},
      {                0.4,  0.397480759172359},
      {                1.5,  0.445261176039822},
      {                2.0,  0.488253406075341},
      {                2.4,  0.554961405856428},
      {               3.34,  0.407099627096608},
      {               50.0,  0.499999189430728},
      {               -0.4, -0.397480759172359},
      {               -1.5, -0.445261176039822},
      {               -2.0, -0.488253406075341},
      {               -2.4, -0.554961405856428},
      {              -3.34, -0.407099627096608},
      {              -50.0, -0.499999189430728}
  };

  for (auto test_value : test_values)
  {
    const double x = test_value[0], val_good = test_value[1];
    const double val = mrpt::math::fresnel_cos_integral(x);
    EXPECT_NEAR(val, val_good, 1e-5)
        << " x: " << x << "\n val_good: " << val_good << "\n val: " << val << "\n";
  }
}

TEST(fresnel, fresnels)
{
  // fresnelc()
  const double test_values[][2] = {
      {  0.0,                 .0},
      {  1.0,  0.438259147390355},
      {  1.5,  0.697504960082093},
      {  2.4,  0.619689964945684},
      { 50.0,  0.493633802585939},
      { -2.0, -0.343415678363698},
      { -2.4, -0.619689964945684},
      {-3.34, -0.479600423968308},
      {-50.0, -0.493633802585939}
  };

  for (auto test_value : test_values)
  {
    const double x = test_value[0], val_good = test_value[1];
    const double val = mrpt::math::fresnel_sin_integral(x);
    EXPECT_NEAR(val, val_good, 1e-5)
        << " x: " << x << "\n val_good: " << val_good << "\n val: " << val << "\n";
  }
}

namespace
{
// Reference values by direct numerical integration of the definitions
//   C(x) = int_0^x cos(pi/2 * t^2) dt,  S(x) = int_0^x sin(pi/2 * t^2) dt
// using composite Simpson's rule, so that no hard-coded table is needed.
double simpson(double x, bool sine)
{
  // Even; the Simpson error stays far below the 1e-6 tolerance used below,
  // while keeping the whole test in the low milliseconds.
  const int n = 20000;
  const double h = x / n;
  auto f = [sine](double t)
  {
    const double a = M_PI * 0.5 * t * t;
    return sine ? std::sin(a) : std::cos(a);
  };
  double s = f(0) + f(x);
  for (int i = 1; i < n; i++) s += f(i * h) * ((i & 1) ? 4.0 : 2.0);
  return s * h / 3.0;
}
}  // namespace

TEST(fresnel, matchesNumericalIntegration)
{
  // Values spanning every internal branch: the power series (|x| < ~0.4), the
  // Chebyshev expansions for each sub-range, and the asymptotic series.
  const double xs[] = {0.05, 0.2, 0.3, 0.39, 0.5, 1.0,   2.0,   3.5,
                       4.2,  5.0, 5.5, 6.5,  9.0, -0.25, -0.35, -5.2};

  for (const double x : xs)
  {
    const double refC = simpson(x, false);
    const double refS = simpson(x, true);

    EXPECT_NEAR(mrpt::math::fresnel_cos_integral(x), refC, 1e-6) << "x=" << x;
    EXPECT_NEAR(mrpt::math::fresnel_sin_integral(x), refS, 1e-6) << "x=" << x;

    // The long-double entry points must agree with the double ones:
    EXPECT_NEAR(
        static_cast<double>(mrpt::math::lfresnel_cos_integral(static_cast<long double>(x))), refC,
        1e-6)
        << "x=" << x;
    EXPECT_NEAR(
        static_cast<double>(mrpt::math::lfresnel_sin_integral(static_cast<long double>(x))), refS,
        1e-6)
        << "x=" << x;
  }
}

TEST(fresnel, oddSymmetry)
{
  for (const double x : {0.2, 1.0, 4.5, 6.0, 20.0})
  {
    EXPECT_NEAR(mrpt::math::fresnel_cos_integral(-x), -mrpt::math::fresnel_cos_integral(x), 1e-12);
    EXPECT_NEAR(mrpt::math::fresnel_sin_integral(-x), -mrpt::math::fresnel_sin_integral(x), 1e-12);
  }
}
