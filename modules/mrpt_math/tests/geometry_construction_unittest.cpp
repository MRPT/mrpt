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
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace mrpt::math;

namespace
{
mrpt::random::CRandomGenerator& rng() { return mrpt::random::getRandomGenerator(); }

TVector3D randomUnitVector()
{
  for (;;)
  {
    const TVector3D v(rng().drawUniform(-1, 1), rng().drawUniform(-1, 1), rng().drawUniform(-1, 1));
    const double n = v.norm();
    if (n > 0.1 && n <= 1.0)
    {
      return v * (1.0 / n);
    }
  }
}

TPoint3D randomPoint(double r)
{
  return {rng().drawUniform(-r, r), rng().drawUniform(-r, r), rng().drawUniform(-r, r)};
}

// Point-line distance via the cross product (independent of TLine3D).
double lineDist(const TLine3D& l, const TPoint3D& p)
{
  return (p - l.pBase).cross(l.director).norm() / l.director.norm();
}

TPose3D randomPose()
{
  return {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5),     rng().drawUniform(-5, 5),
          rng().drawUniform(-3, 3), rng().drawUniform(-1.5, 1.5), rng().drawUniform(-3, 3)};
}

template <typename MAT>
void expectOrthonormalRightHanded(const MAT& M, double tol)
{
  Eigen::Matrix3d R;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      R(r, c) = M(r, c);
    }
  }
  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0, tol) << R;
  EXPECT_NEAR(R.determinant(), 1.0, tol) << R;
}

// Reference PCA of a point cloud: centroid, and eigen-decomposition (ascending)
// of the scatter matrix.
void referencePCA(
    const std::vector<TPoint3D>& pts,
    Eigen::Vector3d& mean,
    Eigen::Vector3d& eVals,
    Eigen::Matrix3d& eVecs)
{
  mean.setZero();
  for (const auto& p : pts)
  {
    mean += Eigen::Vector3d(p.x, p.y, p.z);
  }
  mean /= static_cast<double>(pts.size());
  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  for (const auto& p : pts)
  {
    const Eigen::Vector3d d = Eigen::Vector3d(p.x, p.y, p.z) - mean;
    S += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
  eVals = es.eigenvalues();
  eVecs = es.eigenvectors();
}
}  // namespace

TEST(GeometryConstruction, SegmentBisector3D)
{
  // Segments not centered at the origin.
  rng().randomize(1234);
  for (int k = 0; k < 50; k++)
  {
    const TSegment3D s(randomPoint(5), randomPoint(5));
    TPlane bis;
    getSegmentBisector(s, bis);
    TPoint3D c;
    s.getCenter(c);
    ASSERT_NEAR(bis.distance(c), 0.0, 1e-9) << s;
    // Any point on the plane is equidistant to both endpoints:
    const TVector3D n = bis.getUnitaryNormalVector();
    const TVector3D t = n.cross(randomUnitVector()).unitarize();
    const TPoint3D q = c + t * rng().drawUniform(-3, 3);
    EXPECT_NEAR((q - s.point1).norm(), (q - s.point2).norm(), 1e-9);
  }
}

TEST(GeometryConstruction, SegmentBisector2D)
{
  rng().randomize(1234);
  for (int k = 0; k < 200; k++)
  {
    const TPoint2D a(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPoint2D b(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    TLine2D bis;
    getSegmentBisector(TSegment2D(a, b), bis);
    double dir[2];
    bis.getDirectorVector(dir);
    const TPoint2D mid = (a + b) * 0.5;
    const TPoint2D q = mid + TPoint2D(dir[0], dir[1]) * rng().drawUniform(-3, 3);
    EXPECT_NEAR(bis.distance(mid), 0.0, 1e-9);
    EXPECT_NEAR((q - a).norm(), (q - b).norm(), 1e-9);
  }
}

TEST(GeometryConstruction, AngleBisectorsAreEquidistant)
{
  rng().randomize(1234);
  for (int k = 0; k < 200; k++)
  {
    // 2D: random crossing lines.
    const TPoint2D P(rng().drawUniform(-3, 3), rng().drawUniform(-3, 3));
    const double a1 = rng().drawUniform(-M_PI, M_PI);
    const double a2 = a1 + rng().drawUniform(0.1, M_PI - 0.1);
    const TLine2D l1(P, P + TPoint2D(std::cos(a1), std::sin(a1)) * rng().drawUniform(0.5, 3));
    const TLine2D l2(P, P + TPoint2D(std::cos(a2), std::sin(a2)) * rng().drawUniform(0.5, 3));
    TLine2D bis;
    getAngleBisector(l1, l2, bis);
    double dir[2];
    bis.getDirectorVector(dir);
    EXPECT_NEAR(bis.distance(P), 0.0, 1e-9);
    const TPoint2D q = P + TPoint2D(dir[0], dir[1]) * (2.0 / std::hypot(dir[0], dir[1]));
    EXPECT_NEAR(l1.distance(q), l2.distance(q), 1e-9);

    // 3D: same construction on a random plane.
    const TPose3D pose = randomPose();
    const TPoint3D P3 = pose.composePoint(TPoint3D(P.x, P.y, 0));
    const TPoint3D A3 = pose.composePoint(TPoint3D(P.x + std::cos(a1), P.y + std::sin(a1), 0));
    const TPoint3D B3 = pose.composePoint(TPoint3D(P.x + std::cos(a2), P.y + std::sin(a2), 0));
    const TLine3D m1(P3, A3);
    const TLine3D m2(P3, B3);
    TLine3D bis3;
    getAngleBisector(m1, m2, bis3);
    EXPECT_NEAR(lineDist(bis3, P3), 0.0, 1e-9);
    const TPoint3D q3 = P3 + bis3.director * (2.0 / bis3.director.norm());
    EXPECT_NEAR(lineDist(m1, q3), lineDist(m2, q3), 1e-9);
  }
}

TEST(GeometryConstruction, Line2DFromPoseRotated)
{
  // The local vector must be rotated by +phi.
  for (const double phi : {M_PI / 6, M_PI / 4, -2.0})
  {
    const TPose2D pose(1.0, 2.0, phi);
    const TPoint2D origin(pose.x, pose.y);
    const TPoint2D ux(std::cos(phi), std::sin(phi));
    const TPoint2D uy(-std::sin(phi), std::cos(phi));

    // FromPoseX: normal is the local +X axis, so the line runs along local Y.
    const TLine2D lx = TLine2D::FromPoseX(pose);
    EXPECT_TRUE(lx.contains(origin));
    EXPECT_TRUE(lx.contains(origin + uy * 3.0)) << "phi=" << phi;
    EXPECT_NEAR(lx.distance(origin + ux * 3.0), 3.0, 1e-9) << "phi=" << phi;

    // FromPoseY: normal is the local +Y axis, so the line runs along local X.
    const TLine2D ly = TLine2D::FromPoseY(pose);
    EXPECT_TRUE(ly.contains(origin));
    EXPECT_TRUE(ly.contains(origin + ux * 3.0)) << "phi=" << phi;
    EXPECT_NEAR(ly.distance(origin + uy * 3.0), 3.0, 1e-9) << "phi=" << phi;

    // FromPoseDirection must be consistent with FromPoseX for (1,0):
    const double v[2] = {1.0, 0.0};
    const TLine2D ld = TLine2D::FromPoseDirection(pose, v);
    EXPECT_TRUE(ld.contains(origin + uy * 3.0)) << "phi=" << phi;
  }
}

TEST(GeometryConstruction, Line3DFromPoseRotated)
{
  rng().randomize(1234);
  for (int k = 0; k < 50; k++)
  {
    const TPose3D pose = randomPose();
    CMatrixDouble33 R;
    pose.getRotationMatrix(R);
    const TPoint3D o(pose.x, pose.y, pose.z);
    const TLine3D lines[3] = {
        TLine3D::FromPoseX(pose), TLine3D::FromPoseY(pose), TLine3D::FromPoseZ(pose)};
    for (int axis = 0; axis < 3; axis++)
    {
      const TPoint3D q = o + TPoint3D(R(0, axis), R(1, axis), R(2, axis)) * 2.5;
      EXPECT_NEAR(lineDist(lines[axis], o), 0.0, 1e-9);
      EXPECT_NEAR(lineDist(lines[axis], q), 0.0, 1e-9);
    }
    const double lv[3] = {0.3, -0.4, 0.5};
    const TLine3D ld = TLine3D::FromPoseDirection(pose, lv);
    EXPECT_NEAR(lineDist(ld, pose.composePoint(TPoint3D(0.6, -0.8, 1.0))), 0.0, 1e-9);
  }
}

TEST(GeometryConstruction, AxisBaseFromDirectionRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 500; k++)
  {
    const TVector3D d = randomUnitVector() * rng().drawUniform(0.01, 100);
    const auto M = generateAxisBaseFromDirection(d.x, d.y, d.z);
    expectOrthonormalRightHanded(M, 1e-12);
    const TVector3D u = d.unitarize();
    EXPECT_NEAR(M(0, 0), u.x, 1e-12);
    EXPECT_NEAR(M(1, 0), u.y, 1e-12);
    EXPECT_NEAR(M(2, 0), u.z, 1e-12);
  }
}

TEST(GeometryConstruction, AxisBaseFromDirectionSmallOrNearVertical)
{
  // Small or nearly vertical inputs must still give an orthonormal base.
  for (const auto& d : std::vector<TVector3D>{
           {5e-5, 5e-5,    0},
           {9e-5,    0,    1},
           {1e-4, 1e-4, 1e-4}
  })
  {
    const auto M = generateAxisBaseFromDirection(d.x, d.y, d.z);
    expectOrthonormalRightHanded(M, 1e-12);
  }
}

TEST(GeometryConstruction, AxisBaseFromDirectionAndAxisRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 300; k++)
  {
    const TVector3D u = randomUnitVector();
    for (uint8_t coord = 0; coord < 3; coord++)
    {
      const auto M = generateAxisBaseFromDirectionAndAxis(u, coord);
      expectOrthonormalRightHanded(M, 1e-12);
      EXPECT_NEAR(M(0, coord), u.x, 1e-15);
      EXPECT_NEAR(M(1, coord), u.y, 1e-15);
      EXPECT_NEAR(M(2, coord), u.z, 1e-15);
      EXPECT_EQ(M(3, 3), 1.0);
    }
  }
}

TEST(GeometryConstruction, AxisBaseFromDirectionAndAxisNearlyX)
{
  // A direction very close to the X axis.
  const TVector3D u = TVector3D(1.0, 9e-6, 0).unitarize();
  const auto M = generateAxisBaseFromDirectionAndAxis(u, 2);
  expectOrthonormalRightHanded(M, 1e-12);
}

TEST(GeometryConstruction, PlaneFrom3PointsSmallTriangle)
{
  // The collinearity test must not depend on the triangle size.
  TPlane p;
  ASSERT_NO_THROW(p = TPlane::From3Points({0, 0, 0}, {1e-3, 0, 0}, {0, 1e-3, 0}));
  EXPECT_NEAR(p.distance({0.3, 0.7, 0}), 0.0, 1e-12);
  EXPECT_NEAR(p.distance({0, 0, 2}), 2.0, 1e-12);
}

TEST(GeometryConstruction, PlaneAsPose3DRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 300; k++)
  {
    const TVector3D n = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TPlane pl(n.x, n.y, n.z, rng().drawUniform(-5, 5));
    TPose3D pose;
    pl.getAsPose3D(pose);
    CMatrixDouble33 R;
    pose.getRotationMatrix(R);
    expectOrthonormalRightHanded(R, 1e-12);
    const TVector3D u = n.unitarize();
    EXPECT_NEAR(R(0, 2) * u.x + R(1, 2) * u.y + R(2, 2) * u.z, 1.0, 1e-9);
    EXPECT_NEAR(pl.distance(TPoint3D(pose.x, pose.y, pose.z)), 0.0, 1e-9);
    // Local XY points map onto the plane:
    EXPECT_NEAR(pl.distance(pose.composePoint(TPoint3D(1.5, -2.0, 0))), 0.0, 1e-9);
  }
}

TEST(GeometryConstruction, Project3DWithPlaneAndOrigin)
{
  // The plane frame must be centered at "newOrigin".
  const TPlane pl(0, 0, 1, -2);  // z = 2
  const TPoint3D newOrigin(5, -3, 2);
  TPoint3D out;
  project3D(newOrigin, pl, newOrigin, out);
  EXPECT_NEAR(out.norm(), 0.0, 1e-12);
}

TEST(GeometryConstruction, Project3DIncidenceAndRoundTrip)
{
  rng().randomize(1234);
  for (int k = 0; k < 200; k++)
  {
    const TPose3D pose = randomPose();
    const TPose3D inv = -pose;

    // Plane: a point on the plane stays on the projected plane.
    const TVector3D n = randomUnitVector() * rng().drawUniform(0.1, 10);
    const TPoint3D onPlane = randomPoint(5);
    TPlane pl(n.x, n.y, n.z, -n.dot(onPlane));
    TPlane plT;
    project3D(pl, pose, plT);
    EXPECT_NEAR(plT.distance(pose.composePoint(onPlane)), 0.0, 1e-9);
    EXPECT_NEAR(plT.distance(pose.composePoint(onPlane + n)), n.norm(), 1e-9);

    // Line: points on the line stay on the projected line.
    const TLine3D ln = TLine3D::FromPointAndDirector(randomPoint(5), randomUnitVector() * 3.0);
    TLine3D lnT;
    project3D(ln, pose, lnT);
    EXPECT_NEAR(lineDist(lnT, pose.composePoint(ln.pBase + ln.director * 1.7)), 0.0, 1e-9);

    // Point and polygon round trip:
    const TPolygon3D poly{randomPoint(5), randomPoint(5), randomPoint(5)};
    TPolygon3D polyT;
    TPolygon3D polyBack;
    project3D(poly, pose, polyT);
    project3D(polyT, inv, polyBack);
    for (size_t i = 0; i < poly.size(); i++)
    {
      EXPECT_NEAR((polyBack[i] - poly[i]).norm(), 0.0, 1e-9);
    }
  }
}

TEST(GeometryConstruction, Project2DLineIncidence)
{
  rng().randomize(1234);
  for (int k = 0; k < 300; k++)
  {
    const TPose2D pose(
        rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-M_PI, M_PI));
    const TPoint2D a(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    const TPoint2D b(rng().drawUniform(-5, 5), rng().drawUniform(-5, 5));
    if ((a - b).norm() < 0.1)
    {
      continue;
    }
    const TLine2D ln(a, b);
    TLine2D lnT;
    project2D(ln, pose, lnT);
    EXPECT_NEAR(lnT.distance(pose.composePoint(a)), 0.0, 1e-9);
    EXPECT_NEAR(lnT.distance(pose.composePoint(b)), 0.0, 1e-9);
  }
}

TEST(GeometryConstruction, CreatePlaneFromPoseAndNormalRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 200; k++)
  {
    const TPose3D pose = randomPose();
    const TVector3D nl = randomUnitVector();
    const double normal[3] = {nl.x, nl.y, nl.z};
    TPlane pl;
    createPlaneFromPoseAndNormal(pose, normal, pl);
    // Pose origin lies on the plane, and so does any local point orthogonal to n:
    const TVector3D t = nl.cross(randomUnitVector()).unitarize();
    EXPECT_NEAR(pl.distance(TPoint3D(pose.x, pose.y, pose.z)), 0.0, 1e-9);
    EXPECT_NEAR(pl.distance(pose.composePoint(t * 2.0)), 0.0, 1e-9);
    EXPECT_NEAR(pl.distance(pose.composePoint(nl * 2.0)), 2.0, 1e-9);
  }
}

TEST(GeometryConstruction, RegressionPlaneAndLineVsPCA)
{
  rng().randomize(1234);
  for (int k = 0; k < 50; k++)
  {
    const TPose3D pose = randomPose();
    std::vector<TPoint3D> pts;
    for (int i = 0; i < 40; i++)
    {
      pts.push_back(pose.composePoint(TPoint3D(
          rng().drawUniform(-5, 5), rng().drawUniform(-2, 2), rng().drawGaussian1D(0, 0.05))));
    }
    Eigen::Vector3d mean;
    Eigen::Vector3d ev;
    Eigen::Matrix3d evec;
    referencePCA(pts, mean, ev, evec);

    TPlane pl;
    const double ePlane = getRegressionPlane(pts, pl);
    const TVector3D n = pl.getUnitaryNormalVector();
    EXPECT_NEAR(std::abs(Eigen::Vector3d(n.x, n.y, n.z).dot(evec.col(0))), 1.0, 1e-9);
    EXPECT_NEAR(pl.distance(TPoint3D(mean[0], mean[1], mean[2])), 0.0, 1e-9);
    EXPECT_NEAR(ePlane, ev[0] / (ev[1] + ev[2]), 1e-9);

    TLine3D ln;
    const double eLine = getRegressionLine(pts, ln);
    const TVector3D u = ln.director.unitarize();
    EXPECT_NEAR(std::abs(Eigen::Vector3d(u.x, u.y, u.z).dot(evec.col(2))), 1.0, 1e-9);
    EXPECT_NEAR(lineDist(ln, TPoint3D(mean[0], mean[1], mean[2])), 0.0, 1e-9);
    EXPECT_NEAR(eLine, std::sqrt((ev[0] + ev[1]) / ev[2]), 1e-9);

    // 2D line on the local XY coordinates:
    std::vector<TPoint2D> pts2;
    for (const auto& p : pts)
    {
      pts2.emplace_back(p.x, p.y);
    }
    Eigen::Vector2d m2 = Eigen::Vector2d::Zero();
    for (const auto& p : pts2)
    {
      m2 += Eigen::Vector2d(p.x, p.y);
    }
    m2 /= static_cast<double>(pts2.size());
    Eigen::Matrix2d S2 = Eigen::Matrix2d::Zero();
    for (const auto& p : pts2)
    {
      const Eigen::Vector2d d = Eigen::Vector2d(p.x, p.y) - m2;
      S2 += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(S2);
    TLine2D l2;
    const double eLine2 = getRegressionLine(pts2, l2);
    double dir[2];
    l2.getUnitaryDirectorVector(dir);
    EXPECT_NEAR(
        std::abs(Eigen::Vector2d(dir[0], dir[1]).dot(es2.eigenvectors().col(1))), 1.0, 1e-9);
    EXPECT_NEAR(l2.distance(TPoint2D(m2[0], m2[1])), 0.0, 1e-9);
    EXPECT_NEAR(eLine2, std::sqrt(es2.eigenvalues()[0] / es2.eigenvalues()[1]), 1e-9);
  }
}

TEST(GeometryConstruction, RegressionPlaneOfSmallSquare)
{
  // The collinearity test must not depend on the size of the point set.
  const std::vector<TPoint3D> pts{
      {   0,    0, 0},
      {5e-3,    0, 0},
      {5e-3, 5e-3, 0},
      {   0, 5e-3, 0}
  };
  TPlane pl;
  ASSERT_NO_THROW(getRegressionPlane(pts, pl));
  EXPECT_NEAR(std::abs(pl.getUnitaryNormalVector().z), 1.0, 1e-12);
}
