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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>

#include <sstream>
#include <stdexcept>
#include <vector>

using namespace mrpt::math;

template <typename T>
class TPoint2DTests : public ::testing::Test
{
};

using PointTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(TPoint2DTests, PointTypes);

TYPED_TEST(TPoint2DTests, constructorsAndAccessors)
{
  using P = TPoint2D_<TypeParam>;

  const P zero;
  EXPECT_EQ(zero.x, TypeParam(0));
  EXPECT_EQ(zero.y, TypeParam(0));
  EXPECT_EQ(zero.rows(), 2U);
  EXPECT_EQ(zero.cols(), 1U);
  EXPECT_EQ(zero.size(), 2U);

  P p(1, 2);
  EXPECT_EQ(p[0], TypeParam(1));
  EXPECT_EQ(p[1], TypeParam(2));
  p[0] = 10;
  p[1] = 20;
  EXPECT_EQ(p.x, TypeParam(10));
  EXPECT_EQ(p.y, TypeParam(20));

  const P cp = p;
  EXPECT_EQ(cp[0], TypeParam(10));
  EXPECT_EQ(cp[1], TypeParam(20));

  EXPECT_THROW(p[2], std::out_of_range);
  EXPECT_THROW(cp[2], std::out_of_range);

  // data() gives contiguous (x,y) access:
  EXPECT_EQ(p.data()[0], p.x);
  EXPECT_EQ(cp.data()[1], cp.y);

  // resize() only accepts the static size:
  P q;
  q.resize(2);
  EXPECT_THROW(q.resize(3), std::exception);
}

TYPED_TEST(TPoint2DTests, conversionConstructors)
{
  using P = TPoint2D_<TypeParam>;

  const P fromPose2D(TPose2D(1.0, 2.0, 0.5));
  EXPECT_NEAR(double(fromPose2D.x), 1.0, 1e-6);
  EXPECT_NEAR(double(fromPose2D.y), 2.0, 1e-6);

  const P fromPoint3D(TPoint3D_<TypeParam>(1, 2, 3));
  EXPECT_NEAR(double(fromPoint3D.x), 1.0, 1e-6);
  EXPECT_NEAR(double(fromPoint3D.y), 2.0, 1e-6);

  const P fromPose3D(TPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));
  EXPECT_NEAR(double(fromPose3D.x), 1.0, 1e-6);
  EXPECT_NEAR(double(fromPose3D.y), 2.0, 1e-6);

  // cast<> between float and double variants:
  const auto asFloat = P(3, 4).template cast<float>();
  EXPECT_NEAR(asFloat.x, 3.0f, 1e-6f);
  const auto asDouble = P(3, 4).template cast<double>();
  EXPECT_NEAR(asDouble.y, 4.0, 1e-6);
}

TYPED_TEST(TPoint2DTests, vectorInterop)
{
  using P = TPoint2D_<TypeParam>;

  const std::vector<double> v{7.0, 8.0};
  const P p = P::FromVector(v);
  EXPECT_NEAR(double(p.x), 7.0, 1e-6);
  EXPECT_NEAR(double(p.y), 8.0, 1e-6);

  std::vector<double> out;
  p.asVector(out);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_NEAR(out[1], 8.0, 1e-6);

  const auto out2 = p.template asVector<std::vector<double>>();
  EXPECT_NEAR(out2[0], 7.0, 1e-6);
}

TYPED_TEST(TPoint2DTests, arithmetic)
{
  using P = TPoint2D_<TypeParam>;

  P a(1, 2);
  const P b(10, 20);

  a += b;
  EXPECT_EQ(a, P(11, 22));
  a -= b;
  EXPECT_EQ(a, P(1, 2));
  a *= 3;
  EXPECT_EQ(a, P(3, 6));
  a /= 3;
  EXPECT_EQ(a, P(1, 2));
  EXPECT_THROW(a /= 0, std::exception);

  EXPECT_EQ(a + b, P(11, 22));
  EXPECT_EQ(b - a, P(9, 18));
  EXPECT_EQ(a * 2, P(2, 4));
  EXPECT_EQ(b / 2, P(5, 10));
  EXPECT_THROW((void)(b / 0), std::exception);
  EXPECT_EQ(-a, P(-1, -2));
  EXPECT_EQ(TypeParam(2) * a, P(2, 4));

  EXPECT_TRUE(a != b);
  EXPECT_FALSE(a == b);

  // Strict weak ordering: x first, then y
  EXPECT_TRUE(P(1, 2) < P(2, 0));
  EXPECT_FALSE(P(2, 0) < P(1, 2));
  EXPECT_TRUE(P(1, 2) < P(1, 3));
  EXPECT_FALSE(P(1, 3) < P(1, 2));
}

TYPED_TEST(TPoint2DTests, norms)
{
  using P = TPoint2D_<TypeParam>;

  const P p(3, 4);
  EXPECT_NEAR(double(p.sqrNorm()), 25.0, 1e-4);
  EXPECT_NEAR(double(p.norm()), 5.0, 1e-4);

  const P u = p.unitarize();
  EXPECT_NEAR(double(u.norm()), 1.0, 1e-5);
  EXPECT_NEAR(double(u.x), 0.6, 1e-5);

  EXPECT_THROW(P(0, 0).unitarize(), std::exception);
}

TYPED_TEST(TPoint2DTests, stringConversion)
{
  using P = TPoint2D_<TypeParam>;

  const P p(1.5, -2.25);
  const std::string s = p.asString();
  EXPECT_FALSE(s.empty());

  const P q = P::FromString(s);
  EXPECT_NEAR(double(q.x), 1.5, 1e-4);
  EXPECT_NEAR(double(q.y), -2.25, 1e-4);

  P r;
  r.fromString("[3.0 4.0]");
  EXPECT_NEAR(double(r.x), 3.0, 1e-4);

  EXPECT_THROW(r.fromString("not a matrix"), std::exception);
  EXPECT_THROW(r.fromString("[1 2 3]"), std::exception);

  std::stringstream ss;
  ss << p;
  EXPECT_FALSE(ss.str().empty());
}

TYPED_TEST(TPoint2DTests, tupleInterface)
{
  using P = TPoint2D_<TypeParam>;

  const P p(5, 6);
  EXPECT_EQ(p.template get<0>(), TypeParam(5));
  EXPECT_EQ(p.template get<1>(), TypeParam(6));

  const auto t = p.as_tuple();
  EXPECT_EQ(std::get<0>(t), TypeParam(5));

  TypeParam x = 0, y = 0;
  std::tie(x, y) = p.as_tuple();
  EXPECT_EQ(x, TypeParam(5));
  EXPECT_EQ(y, TypeParam(6));
}
