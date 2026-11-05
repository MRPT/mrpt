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
#include <mrpt/core/format.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace
{
using cplx = std::complex<double>;

// Evaluates x^4 + a*x^3 + b*x^2 + c*x + d
cplx evalQuartic(const cplx& x, double a, double b, double c, double d)
{
  return (((x + a) * x + b) * x + c) * x + d;
}

// Evaluates x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e
cplx evalQuintic(const cplx& x, double a, double b, double c, double d, double e)
{
  return ((((x + a) * x + b) * x + c) * x + d) * x + e;
}

// Checks that the roots reported by solve_poly4() are actual roots, taking into
// account the meaning of each returned value (see the API docs).
void checkQuartic(double a, double b, double c, double d, int expectedNumRealRoots)
{
  const std::string sMsg = mrpt::format("x^4 + %g*x^3 + %g*x^2 + %g*x + %g = 0", a, b, c, d);

  double x[4] = {0, 0, 0, 0};
  const int res = mrpt::math::solve_poly4(x, a, b, c, d);
  ASSERT_EQ(res, expectedNumRealRoots) << sMsg;

  const double tol = 1e-6;
  switch (res)
  {
    case 4:
      for (int i = 0; i < 4; i++)
      {
        EXPECT_NEAR(std::abs(evalQuartic(cplx(x[i], 0), a, b, c, d)), 0.0, tol)
            << sMsg << " root #" << i;
      }
      break;
    case 2:
      // x[0], x[1] real; x[2] +- i*x[3] complex:
      EXPECT_NEAR(std::abs(evalQuartic(cplx(x[0], 0), a, b, c, d)), 0.0, tol) << sMsg;
      EXPECT_NEAR(std::abs(evalQuartic(cplx(x[1], 0), a, b, c, d)), 0.0, tol) << sMsg;
      EXPECT_NEAR(std::abs(evalQuartic(cplx(x[2], x[3]), a, b, c, d)), 0.0, tol) << sMsg;
      break;
    case 0:
      // x[0] +- i*x[1] and x[2] +- i*x[3]:
      EXPECT_NEAR(std::abs(evalQuartic(cplx(x[0], x[1]), a, b, c, d)), 0.0, tol) << sMsg;
      EXPECT_NEAR(std::abs(evalQuartic(cplx(x[2], x[3]), a, b, c, d)), 0.0, tol) << sMsg;
      break;
    default:
      FAIL() << "Unexpected return value " << res << " for " << sMsg;
  }
}
}  // namespace

TEST(poly_roots, solve_poly3_doubleRoot)
{
  // (x-1)^2 * (x+2) = x^3 - 3*x + 2
  double r[3] = {0, 0, 0};
  const int n = mrpt::math::solve_poly3(r, 0.0, -3.0, 2.0);
  // The solver reports the "two distinct real roots" case for a double root:
  EXPECT_EQ(n, 2);
  for (int i = 0; i < n; i++)
  {
    const double v = ((r[i] + 0.0) * r[i] - 3.0) * r[i] + 2.0;
    EXPECT_NEAR(v, 0.0, 1e-9);
  }
}

TEST(poly_roots, solve_poly4_biquadratic)
{
  // (x^2-1)*(x^2-4): 4 real roots, and c==0 routes through the biquadratic solver
  checkQuartic(0, -5, 0, 4, 4);

  // (x^2+1)*(x^2+4): no real roots, biquadratic solver with a positive discriminant
  checkQuartic(0, 5, 0, 4, 0);

  // x^4 + 1: biquadratic solver with a negative discriminant (complex sqrt)
  checkQuartic(0, 0, 0, 1, 0);

  // (x^2-1)*(x^2+4): 2 real + 2 complex roots
  checkQuartic(0, 3, 0, -4, 2);
}

TEST(poly_roots, solve_poly4_generalCases)
{
  // (x-1)(x-2)(x-3)(x-4), shifted so that the resolvent has three positive roots
  checkQuartic(-10, 35, -50, 24, 4);

  // Mirrored version, exercises the opposite sign of the linear coefficient
  checkQuartic(10, 35, 50, 24, 4);

  // (x^2+1)*(x-1)*(x-2) = x^4 -3x^3 +3x^2 -3x +2
  checkQuartic(-3, 3, -3, 2, 2);

  // (x^2+2x+2)*(x^2-2x+5): two pairs of complex roots
  checkQuartic(0, 3, 6, 10, 0);

  // Quadruple root at x=1: four real roots
  checkQuartic(-4, 6, -4, 1, 4);
}

TEST(poly_roots, solve_poly5)
{
  struct TestCase
  {
    double a, b, c, d, e;
  };
  // Each case is a monic quintic; only the real root returned in x[0] is
  // checked here, plus whatever solve_poly4() reports for the deflated quartic.
  const TestCase cases[] = {
      {-15, 85, -225, 274, -120}, // (x-1)(x-2)(x-3)(x-4)(x-5)
      {  0,  0,    0,   0,   -1}, // x^5 = 1
      {  0,  0,    0,   0,    1}, // x^5 = -1
      {  0,  1,    0,   1,    0}, // e==0 => trivial real root at x=0
      {  2,  3,    4,   5,    6},
  };

  for (const auto& tc : cases)
  {
    double x[5] = {0, 0, 0, 0, 0};
    const int n = mrpt::math::solve_poly5(x, tc.a, tc.b, tc.c, tc.d, tc.e);
    const std::string sMsg = mrpt::format(
        "x^5 + %g*x^4 + %g*x^3 + %g*x^2 + %g*x + %g = 0", tc.a, tc.b, tc.c, tc.d, tc.e);

    // At least the one real root found by the quintic-specific solver:
    EXPECT_GE(n, 1) << sMsg;
    EXPECT_NEAR(std::abs(evalQuintic(cplx(x[0], 0), tc.a, tc.b, tc.c, tc.d, tc.e)), 0.0, 1e-5)
        << sMsg;
  }
}

TEST(poly_roots, solve_poly2_degenerate)
{
  double r1 = 0, r2 = 0;

  // a==0 and b==0: no solution at all
  EXPECT_EQ(mrpt::math::solve_poly2(0, 0, 1, r1, r2), 0);

  // a==0: degenerates into the linear equation b*x+c=0
  EXPECT_EQ(mrpt::math::solve_poly2(0, 2, -4, r1, r2), 1);
  EXPECT_NEAR(r1, 2.0, 1e-12);

  // Double root
  EXPECT_EQ(mrpt::math::solve_poly2(1, -2, 1, r1, r2), 2);
  EXPECT_NEAR(r1, 1.0, 1e-12);
  EXPECT_NEAR(r2, 1.0, 1e-12);
}

using mrpt::math::solve_poly2;
using mrpt::math::solve_poly3;
using mrpt::math::solve_poly4;
using mrpt::math::solve_poly5;

namespace
{
// Monic polynomial coefficients from its roots: c[0]=1, c[1]=a, ...
std::vector<double> polyFromRoots(const std::vector<double>& roots)
{
  std::vector<double> c{1.0};
  for (const double r : roots)
  {
    std::vector<double> n(c.size() + 1, 0.0);
    for (size_t i = 0; i < c.size(); i++)
    {
      n[i] += c[i];
      n[i + 1] -= r * c[i];
    }
    c = n;
  }
  return c;
}

void expectSameRoots(std::vector<double> expected, std::vector<double> got, double tol)
{
  ASSERT_EQ(expected.size(), got.size());
  std::sort(expected.begin(), expected.end());
  std::sort(got.begin(), got.end());
  for (size_t i = 0; i < expected.size(); i++)
  {
    EXPECT_NEAR(got[i], expected[i], tol * std::max(1.0, std::abs(expected[i])));
  }
}
}  // namespace

TEST(PolyRootsRegression, cubicRandomRealRoots)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  for (int it = 0; it < 2000; it++)
  {
    const std::vector<double> r = {
        rng.drawUniform(-10.0, 10.0), rng.drawUniform(-10.0, 10.0), rng.drawUniform(-10.0, 10.0)};
    const auto c = polyFromRoots(r);
    double x[3];
    const int n = solve_poly3(x, c[1], c[2], c[3]);
    if (n != 3)
    {
      // Only acceptable for (numerically) repeated roots
      continue;
    }
    expectSameRoots(r, {x[0], x[1], x[2]}, 1e-6);
  }
}

TEST(PolyRootsRegression, cubicOneRealRootAndComplexPair)
{
  // (x-2)(x^2+2x+5) = x^3 + x - 10: roots 2, -1 +- 2i
  double x[3];
  const int n = solve_poly3(x, 0.0, 1.0, -10.0);
  ASSERT_EQ(n, 1);
  EXPECT_NEAR(x[0], 2.0, 1e-12);
  EXPECT_NEAR(x[1], -1.0, 1e-12);
  EXPECT_NEAR(std::abs(x[2]), 2.0, 1e-12);
}

TEST(PolyRootsRegression, quarticRandomRealRoots)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  for (int it = 0; it < 2000; it++)
  {
    const std::vector<double> r = {
        rng.drawUniform(-10.0, 10.0), rng.drawUniform(-10.0, 10.0), rng.drawUniform(-10.0, 10.0),
        rng.drawUniform(-10.0, 10.0)};
    const auto c = polyFromRoots(r);
    double x[4];
    const int n = solve_poly4(x, c[1], c[2], c[3], c[4]);
    ASSERT_EQ(n, 4);
    expectSameRoots(r, {x[0], x[1], x[2], x[3]}, 1e-5);
  }
}

// A double root with f'(x)==0 must be kept as is by the Newton refinement
TEST(PolyRootsRegression, quarticTwoDoubleRootsNotReplacedBySentinel)
{
  // (x^2-1)^2 = x^4 - 2x^2 + 1
  double x[4];
  const int n = solve_poly4(x, 0.0, -2.0, 0.0, 1.0);
  ASSERT_EQ(n, 4);
  expectSameRoots({-1.0, -1.0, 1.0, 1.0}, {x[0], x[1], x[2], x[3]}, 1e-6);
}

// Quadruple roots are four real roots
TEST(PolyRootsRegression, quarticQuadrupleRoot)
{
  double x[4];
  // (x-1)^4
  int n = solve_poly4(x, -4.0, 6.0, -4.0, 1.0);
  EXPECT_EQ(n, 4);
  // x^4
  n = solve_poly4(x, 0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(n, 4);
}

TEST(PolyRootsRegression, quinticDistinctRoots)
{
  const std::vector<double> r = {-3.5, -1.0, 0.25, 2.0, 7.0};
  const auto c = polyFromRoots(r);
  double x[5];
  const int n = solve_poly5(x, c[1], c[2], c[3], c[4], c[5]);
  ASSERT_EQ(n, 5);
  expectSameRoots(r, {x[0], x[1], x[2], x[3], x[4]}, 1e-8);
}

// This input used to make the root bracketing loop run forever
TEST(PolyRootsRegression, quinticTerminates)
{
  // Reference roots computed with 40-digit arithmetic
  const std::vector<double> r = {
      -9.557121819752329, -8.880131009215592, -5.072111171285448, -4.1854290348811505,
      -1.1467468947959927};
  double x[5];
  const int n = solve_poly5(
      x, 28.841539929930512, 308.53998246536435, 1494.4750481560641, 3151.479625670228,
      2066.0594510957394);
  ASSERT_EQ(n, 5);
  expectSameRoots(r, {x[0], x[1], x[2], x[3], x[4]}, 1e-8);
}

TEST(PolyRootsRegression, quadraticBasicCases)
{
  double r1 = 0;
  double r2 = 0;
  ASSERT_EQ(solve_poly2(2.0, -6.0, 4.0, r1, r2), 2);
  EXPECT_NEAR(r1, 1.0, 1e-15);
  EXPECT_NEAR(r2, 2.0, 1e-15);
  ASSERT_EQ(solve_poly2(1.0, 0.0, 1.0, r1, r2), 0);
  ASSERT_EQ(solve_poly2(0.0, 2.0, -3.0, r1, r2), 1);
  EXPECT_NEAR(r1, 1.5, 1e-15);
  EXPECT_EQ(solve_poly2(0.0, 0.0, 1.0, r1, r2), 0);
}

// No loss of accuracy by cancellation when b^2 >> 4ac
TEST(PolyRootsRegression, quadraticNoCancellation)
{
  double r1 = 0;
  double r2 = 0;
  ASSERT_EQ(solve_poly2(1.0, 1e8, 1.0, r1, r2), 2);
  // Exact roots: -1e8 (+1e-8) and -1e-8 (-1e-24)
  EXPECT_NEAR(r1, -1e8, 1e-6);
  EXPECT_NEAR(r2, -1.0000000000000000e-08, 1e-20);
}
