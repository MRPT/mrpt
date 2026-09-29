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
#include <mrpt/core/bits_math.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CVectorField3D.h>

using mrpt::poses::CPose3D;
using namespace mrpt::viz;

namespace
{
// A ray is the +X axis of a pose:
CPose3D rayAlongX(double x, double y, double z) { return CPose3D(x, y, z, 0, 0, 0); }
CPose3D rayDown(double x, double y, double z)
{
  return CPose3D(x, y, z, 0, mrpt::DEG2RAD(90.0), 0);
}
CPose3D rayUp(double x, double y, double z) { return CPose3D(x, y, z, 0, mrpt::DEG2RAD(-90.0), 0); }
}  // namespace

// ---------------------------------------------------------------------------
// CCylinder::traceRay()
// ---------------------------------------------------------------------------
TEST(CCylinder, TraceRayHitsSideAndBases)
{
  auto cyl = CCylinder::Create(1.0f, 1.0f, 1.0f, 20);
  double dist = 0;

  // Horizontal ray towards the side wall:
  EXPECT_TRUE(cyl->traceRay(rayAlongX(-5, 0, 0.5), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);

  // Horizontal ray above the cylinder, or off to the side:
  EXPECT_FALSE(cyl->traceRay(rayAlongX(-5, 0, 2.0), dist));
  EXPECT_FALSE(cyl->traceRay(rayAlongX(-5, 3.0, 0.5), dist));

  // Vertical rays hit the caps:
  EXPECT_TRUE(cyl->traceRay(rayDown(0.2, 0.1, 5.0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);
  EXPECT_TRUE(cyl->traceRay(rayUp(0.2, 0.1, -3.0), dist));
  EXPECT_NEAR(dist, 3.0, 1e-6);

  // ...unless they pass beside it:
  EXPECT_FALSE(cyl->traceRay(rayDown(3.0, 0.0, 5.0), dist));
}

TEST(CCylinder, TraceRayTangentAndInsideAndBehind)
{
  auto cyl = CCylinder::Create(1.0f, 1.0f, 1.0f, 20);
  double dist = 0;

  // Nearly grazing the wall, just inside and just outside of it (not exactly
  // on the tangent, which depends on rounding):
  EXPECT_TRUE(cyl->traceRay(rayAlongX(-5, 0.99, 0.5), dist));
  EXPECT_NEAR(dist, 5.0 - std::sqrt(1.0 - 0.99 * 0.99), 1e-6);
  EXPECT_FALSE(cyl->traceRay(rayAlongX(-5, 1.01, 0.5), dist));

  // The wall is behind the ray origin:
  EXPECT_FALSE(cyl->traceRay(rayAlongX(5, 1.0, 0.5), dist));
  EXPECT_FALSE(cyl->traceRay(rayAlongX(5, 0.0, 0.5), dist));

  // Origin inside the cylinder: the first positive crossing is the far wall:
  EXPECT_TRUE(cyl->traceRay(rayAlongX(0, 0, 0.5), dist));
  EXPECT_NEAR(dist, 1.0, 1e-6);
}

TEST(CCylinder, TraceRayWithoutBases)
{
  auto cyl = CCylinder::Create(1.0f, 1.0f, 1.0f, 20);
  cyl->setHasBases(false, false);
  double dist = 0;

  // A vertical ray through the axis of an open tube hits nothing:
  EXPECT_FALSE(cyl->traceRay(rayDown(0.0, 0.0, 5.0), dist));

  // Only the top cap:
  cyl->setHasBases(true, false);
  EXPECT_TRUE(cyl->traceRay(rayDown(0.0, 0.0, 5.0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);
  // The cap is a surface: it is also hit from the open side below.
  EXPECT_TRUE(cyl->traceRay(rayUp(0.0, 0.0, -3.0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);

  // Only the bottom cap:
  cyl->setHasBases(false, true);
  EXPECT_TRUE(cyl->traceRay(rayUp(0.0, 0.0, -3.0), dist));
  EXPECT_NEAR(dist, 3.0, 1e-6);
  EXPECT_TRUE(cyl->traceRay(rayDown(0.0, 0.0, 5.0), dist));
  EXPECT_NEAR(dist, 5.0, 1e-6);
}

TEST(CCylinder, TraceRayCone)
{
  // A truncated cone, wider at the top:
  auto cone = CCylinder::Create(1.0f, 3.0f, 2.0f, 20);
  double dist = 0;

  // At z=1 the radius is 2:
  EXPECT_TRUE(cone->traceRay(rayAlongX(-10, 0, 1.0), dist));
  EXPECT_NEAR(dist, 8.0, 1e-6);

  // A ray that only reaches the widening part of the cone at a higher z:
  EXPECT_FALSE(cone->traceRay(rayAlongX(-10, 2.5, 0.2), dist));
  EXPECT_TRUE(cone->traceRay(rayAlongX(-10, 2.5, 1.8), dist));

  // A steep cone makes the quadratic's leading coefficient negative:
  auto flare = CCylinder::Create(0.5f, 10.0f, 1.0f, 20);
  EXPECT_TRUE(flare->traceRay(rayDown(0.0, 0.0, 5.0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);
  flare->setHasBases(false, false);
  EXPECT_TRUE(flare->traceRay(rayAlongX(-20, 0, 0.5), dist));
}

TEST(CCylinder, BoundingBoxAndAccessors)
{
  auto cyl = CCylinder::Create(1.0f, 2.0f, 3.0f, 12);
  const auto bb = cyl->getBoundingBoxLocalf();
  EXPECT_NEAR(bb.min.x, -2.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 2.0f, 1e-6f);
  EXPECT_NEAR(bb.max.z, 3.0f, 1e-6f);
  EXPECT_EQ(cyl->getSlicesCount(), 12u);
  cyl->setSlicesCount(30);
  EXPECT_EQ(cyl->getSlicesCount(), 30u);
  EXPECT_TRUE(cyl->hasTopBase());
  EXPECT_TRUE(cyl->hasBottomBase());
}

// ---------------------------------------------------------------------------
// CDisk / CTexturedPlane
// ---------------------------------------------------------------------------
TEST(CDisk, TraceRayAndBoundingBox)
{
  auto disk = CDisk::Create(2.0f, 1.0f, 30);
  double dist = 0;

  // Through the ring:
  EXPECT_TRUE(disk->traceRay(rayDown(1.5, 0.0, 4.0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-6);
  // Through the hole, and outside:
  EXPECT_FALSE(disk->traceRay(rayDown(0.2, 0.0, 4.0), dist));
  EXPECT_FALSE(disk->traceRay(rayDown(2.5, 0.0, 4.0), dist));
  // Parallel to the disk:
  EXPECT_FALSE(disk->traceRay(rayAlongX(-5, 0, 0), dist));
  // Pointing away from it:
  EXPECT_FALSE(disk->traceRay(rayUp(1.5, 0.0, 4.0), dist));

  const auto bb = disk->getBoundingBoxLocalf();
  EXPECT_NEAR(bb.min.x, -2.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 2.0f, 1e-6f);
  EXPECT_NEAR(bb.max.z, 0.0f, 1e-6f);
}

TEST(CTexturedPlane, TraceRayAndBoundingBox)
{
  auto plane = CTexturedPlane::Create(-1.0f, 2.0f, -3.0f, 4.0f);
  double dist = 0;

  EXPECT_TRUE(plane->traceRay(rayDown(0.5, 0.5, 3.0), dist));
  EXPECT_NEAR(dist, 3.0, 1e-6);
  EXPECT_FALSE(plane->traceRay(rayDown(5.0, 0.5, 3.0), dist));

  // The cached polygon must follow the plane's corners:
  plane->setPlaneCorners(10.0f, 12.0f, 10.0f, 12.0f);
  EXPECT_FALSE(plane->traceRay(rayDown(0.5, 0.5, 3.0), dist));
  EXPECT_TRUE(plane->traceRay(rayDown(11.0, 11.0, 3.0), dist));

  const auto bb = plane->getBoundingBoxLocalf();
  EXPECT_NEAR(bb.min.x, 10.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 12.0f, 1e-6f);
  EXPECT_NEAR(bb.max.z, 0.0f, 1e-6f);
}

// ---------------------------------------------------------------------------
// Bounding boxes of other objects
// ---------------------------------------------------------------------------
TEST(CSetOfLines, SegmentsAndBoundingBox)
{
  const std::vector<mrpt::math::TSegment3D> segs = {
      {   {0, 0, 0}, {1, 2, 3}},
      {{-1, -2, -3}, {0, 0, 0}},
  };
  CSetOfLines lines(segs, false /*no antialiasing*/);
  ASSERT_EQ(lines.size(), 2u);

  const auto bb = lines.getBoundingBoxLocalf();
  EXPECT_NEAR(bb.min.x, -1.0f, 1e-6f);
  EXPECT_NEAR(bb.min.z, -3.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 2.0f, 1e-6f);

  lines.setLineByIndex(1, mrpt::math::TSegment3D({0, 0, 0}, {10, 0, 0}));
  EXPECT_NEAR(lines.getBoundingBoxLocalf().max.x, 10.0f, 1e-6f);
  EXPECT_THROW(
      lines.setLineByIndex(2, mrpt::math::TSegment3D({0, 0, 0}, {1, 1, 1})), std::exception);
}

TEST(CVectorField3D, ConstructorFromMatricesAndBoundingBox)
{
  mrpt::math::CMatrixFloat vx(2, 2);
  mrpt::math::CMatrixFloat vy(2, 2);
  mrpt::math::CMatrixFloat vz(2, 2);
  mrpt::math::CMatrixFloat px(2, 2);
  mrpt::math::CMatrixFloat py(2, 2);
  mrpt::math::CMatrixFloat pz(2, 2);
  vx.setConstant(1.0f);
  vy.setConstant(0.0f);
  vz.setConstant(0.5f);
  for (int r = 0; r < 2; r++)
  {
    for (int c = 0; c < 2; c++)
    {
      px(r, c) = static_cast<float>(c);
      py(r, c) = static_cast<float>(r);
      pz(r, c) = 0.0f;
    }
  }
  CVectorField3D vf(vx, vy, vz, px, py, pz);
  EXPECT_EQ(vf.getVectorField_x().rows(), 2);
  mrpt::math::CMatrixFloat cx;
  mrpt::math::CMatrixFloat cy;
  mrpt::math::CMatrixFloat cz;
  vf.getPointCoordinates(cx, cy, cz);
  EXPECT_EQ(cx.cols(), 2);

  const auto bb = vf.getBoundingBoxLocalf();
  // Sample points span x in [0,1], and the arrows extend it by the vectors:
  EXPECT_NEAR(bb.min.x, 0.0f, 1e-6f);
  EXPECT_NEAR(bb.max.x, 2.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 1.0f, 1e-6f);
  EXPECT_NEAR(bb.max.z, 0.5f, 1e-6f);

  // Also right after having generated the render buffers:
  vf.updateBuffers();
  EXPECT_NEAR(vf.getBoundingBoxLocalf().max.x, 2.0f, 1e-6f);
  EXPECT_NO_THROW(vf.enableColorFromModule(true));
  vf.updateBuffers();

  // No data:
  CVectorField3D empty;
  EXPECT_NO_THROW(empty.getBoundingBoxLocalf());
}

TEST(CPointCloudColoured, BoundingBoxAndRecolorizeByCoordinate)
{
  CPointCloudColoured pc;
  EXPECT_NO_THROW(pc.getBoundingBoxLocalf());  // empty cloud

  for (int i = 0; i <= 10; i++)
  {
    pc.push_back(
        static_cast<float>(i), 2.0f * static_cast<float>(i), 5.0f - static_cast<float>(i), 0, 0, 0);
  }
  const auto bb = pc.getBoundingBoxLocalf();
  EXPECT_NEAR(bb.min.x, 0.0f, 1e-6f);
  EXPECT_NEAR(bb.max.x, 10.0f, 1e-6f);
  EXPECT_NEAR(bb.max.y, 20.0f, 1e-6f);
  EXPECT_NEAR(bb.min.z, -5.0f, 1e-6f);

  // Recolorizing along each axis must give the extremes of the color map to
  // the points at the ends of the range:
  for (int axis = 0; axis < 3; axis++)
  {
    pc.recolorizeByCoordinate(0.0f, 10.0f, axis, mrpt::img::cmGRAYSCALE);
    const auto cLow = pc.getPointColor(0);
    const auto cHigh = pc.getPointColor(10);
    if (axis == 2)
    {  // z decreases with the index
      EXPECT_GT(cLow.R, cHigh.R);
    }
    else
    {
      EXPECT_LT(cLow.R, cHigh.R) << "axis " << axis;
    }
  }
  EXPECT_THROW(pc.recolorizeByCoordinate(0.0f, 1.0f, 3), std::exception);
  // Degenerate range must not divide by zero:
  EXPECT_NO_THROW(pc.recolorizeByCoordinate(1.0f, 1.0f, 0));
}

TEST(CPointCloudColoured, SetVertexWithoutColorDefaultsToWhite)
{
  CPointCloudColoured pc;
  pc.push_back(1, 2, 3, 0, 0, 0);
  // Using the generic import interface with no color:
  // (goes through PLY_import_set_vertex(idx, pt, nullptr))
  const std::string file = mrpt::system::getTempFileName() + "_nocolor.ply";
  {
    CPointCloud plain;
    plain.insertPoint(4, 5, 6);
    ASSERT_TRUE(plain.saveToPlyFile(file));
  }
  CPointCloudColoured pc2;
  ASSERT_TRUE(pc2.loadFromPlyFile(file));
  ASSERT_EQ(pc2.size(), 1u);
  const auto c = pc2.getPointColor(0);
  EXPECT_EQ(c.R, 255);
  EXPECT_EQ(c.G, 255);
  EXPECT_EQ(c.B, 255);
  mrpt::system::deleteFile(file);
}

// ---------------------------------------------------------------------------
// CMesh
// ---------------------------------------------------------------------------
TEST(CMesh, ColorFromZBothTriangulations)
{
  CMesh mesh;
  EXPECT_NO_THROW(mesh.getBoundingBoxLocalf());  // empty mesh

  mrpt::math::CMatrixFloat Z(6, 6);
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      Z(r, c) = static_cast<float>(r + c) * 0.1f;
    }
  }
  mesh.setGridLimits(0, 5, 0, 5);
  mesh.setZ(Z);
  mesh.enableColorFromZ(true, mrpt::img::cmJET);

  // Both diagonal orientations are chosen from the sign of the local
  // slope, so use a saddle too:
  const auto bb1 = mesh.getBoundingBoxLocalf();
  EXPECT_NEAR(bb1.min.z, 0.0f, 1e-6f);
  EXPECT_NEAR(bb1.max.z, 1.0f, 1e-5f);
  mesh.updateBuffers();  // also exercises the color-from-Z triangles
  EXPECT_NEAR(mesh.getBoundingBoxLocalf().max.z, 1.0f, 1e-5f);

  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      Z(r, c) = ((r + c) % 2 == 0) ? 1.0f : -1.0f;
    }
  }
  mesh.setZ(Z);
  EXPECT_NEAR(mesh.getBoundingBoxLocalf().min.z, -1.0f, 1e-6f);
  mesh.enableWireFrame(true);
  EXPECT_NO_THROW(mesh.updateBuffers());
  mesh.enableWireFrame(false);
  mesh.enableTransparency(true);
  EXPECT_NO_THROW(mesh.updateBuffers());
}
