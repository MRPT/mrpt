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
#include <mrpt/topography.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::topography;

TEST(Transform7Params, IdentityWithNoDeltasOrRotation)
{
  TDatum7Params d(0, 0, 0, 0, 0, 0, 0);
  TPoint3D p(1.0, 2.0, 3.0);
  TPoint3D o;
  transform7params(p, d, o);
  EXPECT_NEAR(o.x, p.x, 1e-9);
  EXPECT_NEAR(o.y, p.y, 1e-9);
  EXPECT_NEAR(o.z, p.z, 1e-9);
}

TEST(Transform7Params, PureTranslation)
{
  TDatum7Params d(10, 20, 30, 0, 0, 0, 0);
  TPoint3D p(1.0, 2.0, 3.0);
  TPoint3D o;
  transform7params(p, d, o);
  EXPECT_NEAR(o.x, 11.0, 1e-9);
  EXPECT_NEAR(o.y, 22.0, 1e-9);
  EXPECT_NEAR(o.z, 33.0, 1e-9);
}

TEST(Transform7Params, MatchesDocumentedMatrixFormula)
{
  // [ X Y Z ]_WGS84 = [dX dY dZ] + (1+dS)[1 RZ -RY; -RZ 1 RX; RY -RX 1][X Y
  // Z]_local
  TDatum7Params d(1, 2, 3, 36.0, -18.0, 7.2, 5e5);  // arbitrary small angles
  TPoint3D p(100.0, -50.0, 25.0);
  TPoint3D o;
  transform7params(p, d, o);

  const double scale = 1 + d.dS;
  const double ex = d.dX + scale * (p.x + p.y * d.Rz - p.z * d.Ry);
  const double ey = d.dY + scale * (-p.x * d.Rz + p.y + p.z * d.Rx);
  const double ez = d.dZ + scale * (p.x * d.Ry - p.y * d.Rx + p.z);

  EXPECT_NEAR(o.x, ex, 1e-9);
  EXPECT_NEAR(o.y, ey, 1e-9);
  EXPECT_NEAR(o.z, ez, 1e-9);
}

TEST(Transform7ParamsTOPCON, IdentityMatrix)
{
  TDatum7Params_TOPCON d(5, 6, 7, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0);
  TPoint3D p(1.0, 2.0, 3.0);
  TPoint3D o;
  transform7params_TOPCON(p, d, o);
  EXPECT_NEAR(o.x, 6.0, 1e-9);
  EXPECT_NEAR(o.y, 8.0, 1e-9);
  EXPECT_NEAR(o.z, 10.0, 1e-9);
}

TEST(Transform10Params, PivotPointMapsToTranslationOnly)
{
  TDatum10Params d(1, 2, 3, 10, 20, 30, 0, 0, 0, 0);
  TPoint3D p(10.0, 20.0, 30.0);  // == pivot point (Xp,Yp,Zp)
  TPoint3D o;
  transform10params(p, d, o);
  // (p - pivot) == 0, so output should be dX+Xp, dY+Yp, dZ+Zp
  EXPECT_NEAR(o.x, 1 + 10, 1e-9);
  EXPECT_NEAR(o.y, 2 + 20, 1e-9);
  EXPECT_NEAR(o.z, 3 + 30, 1e-9);
}

TEST(Transform10Params, MatchesDocumentedMatrixFormula)
{
  TDatum10Params d(1, 2, 3, 5, -5, 2, 36.0, -18.0, 7.2, 5e5);
  TPoint3D p(100.0, -50.0, 25.0);
  TPoint3D o;
  transform10params(p, d, o);

  const double scale = 1 + d.dS;
  const double px = p.x - d.Xp;
  const double py = p.y - d.Yp;
  const double pz = p.z - d.Zp;
  const double ex = d.dX + scale * (px + py * d.Rz - pz * d.Ry) + d.Xp;
  const double ey = d.dY + scale * (-px * d.Rz + py + pz * d.Rx) + d.Yp;
  const double ez = d.dZ + scale * (px * d.Ry - py * d.Rx + pz) + d.Zp;

  EXPECT_NEAR(o.x, ex, 1e-9);
  EXPECT_NEAR(o.y, ey, 1e-9);
  EXPECT_NEAR(o.z, ez, 1e-9);
}

TEST(TransformHelmert2D, Rotation90Degrees)
{
  TDatumHelmert2D d(0, 0, 90.0, 0, 0, 0);
  TPoint2D p(1.0, 0.0);
  TPoint2D o;
  transformHelmert2D(p, d, o);
  EXPECT_NEAR(o.x, 0.0, 1e-9);
  EXPECT_NEAR(o.y, 1.0, 1e-9);
}

TEST(TransformHelmert2D, TranslationOnly)
{
  TDatumHelmert2D d(5, -3, 0.0, 0, 0, 0);
  TPoint2D p(1.0, 2.0);
  TPoint2D o;
  transformHelmert2D(p, d, o);
  EXPECT_NEAR(o.x, 6.0, 1e-9);
  EXPECT_NEAR(o.y, -1.0, 1e-9);
}

TEST(TransformHelmert2DTOPCON, MatchesFormula)
{
  TDatumHelmert2D_TOPCON d(2.0, 0.5, 10.0, -5.0);
  TPoint2D p(3.0, 4.0);
  TPoint2D o;
  transformHelmert2D_TOPCON(p, d, o);
  EXPECT_NEAR(o.x, d.a * p.x + d.b * p.y + d.c, 1e-9);
  EXPECT_NEAR(o.y, -d.b * p.x + d.a * p.y + d.d, 1e-9);
}

TEST(TransformHelmert3D, MatchesTransform7ParamsFormulaWithNegatedAngles)
{
  // transformHelmert3D() applies the same matrix formula as
  // transform7params(), but with negated rotation values. d.Rx/Ry/Rz and
  // d.dS are already converted to radians/fraction by the TDatumHelmert3D
  // constructor, so they are used here directly (without going through
  // TDatum7Params's constructor, which expects raw arc-seconds/ppm).
  TDatumHelmert3D d(1, 2, 3, 36.0, -18.0, 7.2, 5e5);
  TPoint3D p(10.0, -20.0, 5.0);
  TPoint3D o;
  transformHelmert3D(p, d, o);

  const double scale = 1 + d.dS;
  const double Rx = -d.Rx;
  const double Ry = -d.Ry;
  const double Rz = -d.Rz;

  TPoint3D o_direct;
  o_direct.x = d.dX + scale * (p.x + p.y * Rz - p.z * Ry);
  o_direct.y = d.dY + scale * (-p.x * Rz + p.y + p.z * Rx);
  o_direct.z = d.dZ + scale * (p.x * Ry - p.y * Rx + p.z);

  EXPECT_NEAR(o.x, o_direct.x, 1e-9);
  EXPECT_NEAR(o.y, o_direct.y, 1e-9);
  EXPECT_NEAR(o.z, o_direct.z, 1e-9);
}

TEST(TransformHelmert3D, SmallRotationIsMeaningful)
{
  // Regression test: a small, realistic datum-transformation rotation (a
  // few arc-seconds) must produce a small but clearly non-zero effect,
  // i.e. the rotation angles must not be silently collapsed to
  // (near-)zero by an accidental double unit conversion.
  TDatumHelmert3D d(0, 0, 0, 0.0, 0.0, 3600.0, 0);  // 1 degree about Z
  TPoint3D p(1000.0, 0.0, 0.0);
  TPoint3D o;
  transformHelmert3D(p, d, o);

  const double expected_y = mrpt::DEG2RAD(1.0) * p.x;
  EXPECT_NEAR(o.y, expected_y, 1e-6);
  EXPECT_GT(std::abs(o.y), 1.0);  // must be a meaningful (meter-scale) effect
}

TEST(TransformHelmert3DTOPCON, MatchesFormula)
{
  TDatumHelmert3D_TOPCON d(1, 2, 3, 4, 5, 6, 7);
  TPoint3D p(1.0, 2.0, 3.0);
  TPoint3D o;
  transformHelmert3D_TOPCON(p, d, o);
  EXPECT_NEAR(o.x, d.a * p.x + d.b * p.y + d.c, 1e-9);
  EXPECT_NEAR(o.y, d.d * p.x + d.e * p.y + d.f, 1e-9);
  EXPECT_NEAR(o.z, p.z + d.g, 1e-9);
}

TEST(Transform1D, MatchesFormula)
{
  TDatum1DTransf d(2.0, 3.0, 5.0, 1e6);  // dS == 1.0 after ppm conversion
  TPoint3D p(10.0, 20.0, 30.0);
  TPoint3D o;
  transform1D(p, d, o);

  EXPECT_NEAR(o.x, p.x, 1e-9);
  EXPECT_NEAR(o.y, p.y, 1e-9);
  const double expected_z = (d.dY * p.x - d.dX * p.y + p.z) * (1 + d.dS) + d.DZ;
  EXPECT_NEAR(o.z, expected_z, 1e-9);
}

TEST(TransfInterpolation, NoDistortionIsPureScale)
{
  TDatumTransfInterpolation d(0, 0, 1e6, 2e6, 0.0);  // dSx=1.0, dSy=2.0, beta=0
  TPoint3D p(10.0, 5.0, 1.0);
  TPoint3D o;
  transfInterpolation(p, d, o);
  EXPECT_NEAR(o.x, p.x * 1.0, 1e-9);
  EXPECT_NEAR(o.y, p.y * 2.0, 1e-9);
  EXPECT_NEAR(o.z, p.z, 1e-9);
}

TEST(TransfInterpolation, MatchesFormula)
{
  TDatumTransfInterpolation d(1.0, -2.0, 5e5, 2e6, 3600.0);  // beta == 1 degree
  TPoint3D p(7.0, -3.0, 4.0);
  TPoint3D o;
  transfInterpolation(p, d, o);

  const double ex = d.dX + p.x * d.dSx - p.y * d.dSy * sin(d.beta);
  const double ey = d.dY + p.y * d.dSy * cos(d.beta);
  EXPECT_NEAR(o.x, ex, 1e-9);
  EXPECT_NEAR(o.y, ey, 1e-9);
  EXPECT_NEAR(o.z, p.z, 1e-9);
}
