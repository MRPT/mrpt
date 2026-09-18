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
#include <mrpt/topography.h>

#include <sstream>

using namespace mrpt::topography;

TEST(TCoords, DefaultConstructor)
{
  TCoords c;
  EXPECT_DOUBLE_EQ(c.decimal_value, 0.0);
  EXPECT_DOUBLE_EQ(c.getDecimalValue(), 0.0);
}

TEST(TCoords, DecimalConstructor)
{
  TCoords c(12.5);
  EXPECT_DOUBLE_EQ(c.getDecimalValue(), 12.5);
}

TEST(TCoords, DegMinSecPositive)
{
  TCoords c(36, 1, 30.0);
  // 36 + 1/60 + 30/3600 = 36.025
  EXPECT_NEAR(c.getDecimalValue(), 36.025, 1e-9);

  int deg;
  int min;
  double sec;
  c.getDegMinSec(deg, min, sec);
  EXPECT_EQ(deg, 36);
  EXPECT_EQ(min, 1);
  EXPECT_NEAR(sec, 30.0, 1e-6);
}

TEST(TCoords, DegMinSecNegative)
{
  TCoords c(-3, 2, 40.0);
  EXPECT_NEAR(c.getDecimalValue(), -(3 + 2 / 60.0 + 40 / 3600.0), 1e-9);

  int deg;
  int min;
  double sec;
  c.getDegMinSec(deg, min, sec);
  EXPECT_EQ(deg, -3);
  EXPECT_EQ(min, 2);
  EXPECT_NEAR(sec, 40.0, 1e-6);
}

TEST(TCoords, SetFromDecimalAndReadBack)
{
  TCoords c;
  c.setFromDecimal(-45.5);
  EXPECT_NEAR(c.getDecimalValue(), -45.5, 1e-12);
}

TEST(TCoords, SetDegMinSecRoundTrip)
{
  TCoords c;
  c.setDegMinSec(10, 15, 36.0);
  int deg;
  int min;
  double sec;
  c.getDegMinSec(deg, min, sec);
  EXPECT_EQ(deg, 10);
  EXPECT_EQ(min, 15);
  EXPECT_NEAR(sec, 36.0, 1e-6);
}

TEST(TCoords, GetAsString)
{
  TCoords c(36, 1, 30.0);
  const std::string s = c.getAsString();
  // Expected form: "36deg 1' 30.0000''"
  EXPECT_NE(s.find("36deg"), std::string::npos);
  EXPECT_NE(s.find("1'"), std::string::npos);
}

TEST(TCoords, EqualityOperators)
{
  TCoords a(10.0);
  TCoords b(10.0);
  TCoords c(20.0);
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(TCoords, StreamOperator)
{
  TCoords c(1, 2, 3.0);
  std::ostringstream ss;
  ss << c;
  EXPECT_FALSE(ss.str().empty());
  EXPECT_EQ(ss.str(), c.getAsString());
}

TEST(TCoords, MutableDoubleReference)
{
  TCoords c(1.0);
  double& ref = c;
  ref = 99.0;
  EXPECT_DOUBLE_EQ(c.getDecimalValue(), 99.0);
}

TEST(TGeodeticCoords, DefaultIsClear)
{
  TGeodeticCoords g;
  EXPECT_TRUE(g.isClear());
  EXPECT_DOUBLE_EQ(g.height, 0.0);
}

TEST(TGeodeticCoords, NonZeroIsNotClear)
{
  TGeodeticCoords g(1.0, 0.0, 0.0);
  EXPECT_FALSE(g.isClear());

  TGeodeticCoords g2(0.0, 1.0, 0.0);
  EXPECT_FALSE(g2.isClear());

  TGeodeticCoords g3(0.0, 0.0, 1.0);
  EXPECT_FALSE(g3.isClear());
}

TEST(TGeodeticCoords, EqualityOperators)
{
  TGeodeticCoords a(36.0, -4.0, 10.0);
  TGeodeticCoords b(36.0, -4.0, 10.0);
  TGeodeticCoords c(36.0, -4.0, 20.0);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(TEllipsoid, DefaultIsWGS84)
{
  TEllipsoid e;
  EXPECT_EQ(e.name, "WGS84");
  EXPECT_DOUBLE_EQ(e.sa, 6378137.0);
  EXPECT_DOUBLE_EQ(e.sb, 6356752.314245);
}

TEST(TEllipsoid, CustomConstructor)
{
  TEllipsoid e(1000.0, 900.0, "test_ellipsoid");
  EXPECT_DOUBLE_EQ(e.sa, 1000.0);
  EXPECT_DOUBLE_EQ(e.sb, 900.0);
  EXPECT_EQ(e.name, "test_ellipsoid");
}

TEST(TEllipsoid, NamedFactoriesAreConsistent)
{
  // Semi-major axis must always be >= semi-minor axis for a valid
  // reference ellipsoid.
  for (const auto& e :
       {TEllipsoid::Ellipsoid_WGS84(), TEllipsoid::Ellipsoid_GRS80(),
        TEllipsoid::Ellipsoid_Clarke_1866(), TEllipsoid::Ellipsoid_Airy_1830(),
        TEllipsoid::Ellipsoid_Krasovsky_1940()})
  {
    EXPECT_GT(e.sa, e.sb);
    EXPECT_FALSE(e.name.empty());
  }
}

TEST(TDatum7Params, ArcsecAndPpmConversion)
{
  // Rotations given in arc-seconds, scale in ppm:
  TDatum7Params d(1.0, 2.0, 3.0, 3600.0, 0.0, 0.0, 1e6);
  // 3600 arc-seconds = 1 degree
  EXPECT_NEAR(d.Rx, mrpt::DEG2RAD(1.0), 1e-12);
  EXPECT_NEAR(d.Ry, 0.0, 1e-12);
  EXPECT_NEAR(d.Rz, 0.0, 1e-12);
  // 1e6 ppm = 1.0 scale delta
  EXPECT_NEAR(d.dS, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(d.dX, 1.0);
  EXPECT_DOUBLE_EQ(d.dY, 2.0);
  EXPECT_DOUBLE_EQ(d.dZ, 3.0);
}

TEST(TDatum7Params_TOPCON, ConstructorStoresAllFields)
{
  TDatum7Params_TOPCON d(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 1e6);
  EXPECT_DOUBLE_EQ(d.dX, 1);
  EXPECT_DOUBLE_EQ(d.dY, 2);
  EXPECT_DOUBLE_EQ(d.dZ, 3);
  EXPECT_DOUBLE_EQ(d.m11, 4);
  EXPECT_DOUBLE_EQ(d.m33, 12);
  EXPECT_NEAR(d.dS, 1.0, 1e-12);
}

TEST(TDatum10Params, ArcsecAndPpmConversion)
{
  TDatum10Params d(1, 2, 3, 10, 20, 30, 3600.0, 0.0, 0.0, 2e6);
  EXPECT_NEAR(d.Rx, mrpt::DEG2RAD(1.0), 1e-12);
  EXPECT_NEAR(d.dS, 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(d.Xp, 10);
  EXPECT_DOUBLE_EQ(d.Yp, 20);
  EXPECT_DOUBLE_EQ(d.Zp, 30);
}

TEST(TDatumHelmert2D, AngleAndPpmConversion)
{
  TDatumHelmert2D d(1, 2, 90.0, 1e6, 5, 6);
  EXPECT_NEAR(d.alpha, mrpt::DEG2RAD(90.0), 1e-12);
  EXPECT_NEAR(d.dS, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(d.Xp, 5);
  EXPECT_DOUBLE_EQ(d.Yp, 6);
}

TEST(TDatumHelmert2D_TOPCON, ConstructorStoresAllFields)
{
  TDatumHelmert2D_TOPCON d(1, 2, 3, 4);
  EXPECT_DOUBLE_EQ(d.a, 1);
  EXPECT_DOUBLE_EQ(d.b, 2);
  EXPECT_DOUBLE_EQ(d.c, 3);
  EXPECT_DOUBLE_EQ(d.d, 4);
}

TEST(TDatumHelmert3D, ArcsecAndPpmConversion)
{
  TDatumHelmert3D d(1, 2, 3, 3600.0, 0.0, 0.0, 1e6);
  EXPECT_NEAR(d.Rx, mrpt::DEG2RAD(1.0), 1e-12);
  EXPECT_NEAR(d.dS, 1.0, 1e-12);
}

TEST(TDatumHelmert3D_TOPCON, ConstructorStoresAllFields)
{
  TDatumHelmert3D_TOPCON d(1, 2, 3, 4, 5, 6, 7);
  EXPECT_DOUBLE_EQ(d.a, 1);
  EXPECT_DOUBLE_EQ(d.g, 7);
}

TEST(TDatum1DTransf, PpmConversion)
{
  TDatum1DTransf d(1, 2, 3, 1e6);
  EXPECT_DOUBLE_EQ(d.dX, 1);
  EXPECT_DOUBLE_EQ(d.dY, 2);
  EXPECT_DOUBLE_EQ(d.DZ, 3);
  EXPECT_NEAR(d.dS, 1.0, 1e-12);
}

TEST(TDatumTransfInterpolation, PpmAndArcsecConversion)
{
  TDatumTransfInterpolation d(1, 2, 1e6, 2e6, 3600.0);
  EXPECT_NEAR(d.dSx, 1.0, 1e-12);
  EXPECT_NEAR(d.dSy, 2.0, 1e-12);
  EXPECT_NEAR(d.beta, mrpt::DEG2RAD(1.0), 1e-12);
}
