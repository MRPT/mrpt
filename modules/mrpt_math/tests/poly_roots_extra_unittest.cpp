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

#include <cmath>
#include <complex>

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

  // Quadruple root at x=1: the solver cannot tell the four coincident roots
  // apart and reports them as two complex conjugate pairs sitting on the root.
  checkQuartic(-4, 6, -4, 1, 0);
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
