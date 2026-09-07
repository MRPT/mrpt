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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTextMessageCapable.h>
#include <mrpt/viz/CVectorField3D.h>

using namespace mrpt::viz;

namespace
{
/** Round-trips an object through a memory stream and returns the copy. */
template <class T>
typename T::Ptr serializeRoundTrip(const T& obj)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << obj;
  buf.Seek(0);
  auto copy = T::Create();
  arch >> *copy;
  return copy;
}
}  // namespace

// ----------------------------------------------------------------- CCylinder

TEST(CCylinder, SettersAndGetters)
{
  auto o = CCylinder::Create();
  EXPECT_FLOAT_EQ(o->getBottomRadius(), 1.0f);
  EXPECT_FLOAT_EQ(o->getTopRadius(), 1.0f);
  EXPECT_FLOAT_EQ(o->getHeight(), 1.0f);
  EXPECT_TRUE(o->hasTopBase());
  EXPECT_TRUE(o->hasBottomBase());

  o->setRadius(2.5f);
  EXPECT_FLOAT_EQ(o->getBottomRadius(), 2.5f);
  EXPECT_FLOAT_EQ(o->getTopRadius(), 2.5f);

  o->setRadii(1.0f, 0.5f);
  EXPECT_FLOAT_EQ(o->getBottomRadius(), 1.0f);
  EXPECT_FLOAT_EQ(o->getTopRadius(), 0.5f);

  o->setHeight(3.0f);
  EXPECT_FLOAT_EQ(o->getHeight(), 3.0f);

  o->setSlicesCount(16);
  EXPECT_EQ(o->getSlicesCount(), 16U);

  o->setHasBases(false, true);
  EXPECT_FALSE(o->hasTopBase());
  EXPECT_TRUE(o->hasBottomBase());

  const auto bb = o->getBoundingBoxLocal();
  EXPECT_NEAR(bb.max.z, 3.0, 1e-4);
  EXPECT_NEAR(bb.min.z, 0.0, 1e-4);
}

TEST(CCylinder, TraceRay)
{
  auto o = CCylinder::Create(1.0f, 1.0f, 2.0f, 20);

  // A ray along +X aimed at the cylinder's side at mid height:
  double dist = 0;
  EXPECT_TRUE(o->traceRay(mrpt::poses::CPose3D(-5.0, 0, 1.0, 0, 0, 0), dist));
  EXPECT_NEAR(dist, 4.0, 1e-3);

  // Well above the cylinder: no hit.
  dist = 0;
  EXPECT_FALSE(o->traceRay(mrpt::poses::CPose3D(-5.0, 0, 10.0, 0, 0, 0), dist));

  // A truncated cone uses the other branch of the intersection solver:
  auto cone = CCylinder::Create(1.0f, 0.2f, 2.0f, 20);
  dist = 0;
  EXPECT_TRUE(cone->traceRay(mrpt::poses::CPose3D(-5.0, 0, 0.1, 0, 0, 0), dist));
  EXPECT_GT(dist, 0.0);
}

TEST(CCylinder, SerializationRoundTrip)
{
  CCylinder o(1.5f, 0.5f, 3.0f, 24);
  o.setHasBases(false, true);
  const auto copy = serializeRoundTrip(o);

  EXPECT_FLOAT_EQ(copy->getBottomRadius(), 1.5f);
  EXPECT_FLOAT_EQ(copy->getTopRadius(), 0.5f);
  EXPECT_FLOAT_EQ(copy->getHeight(), 3.0f);
  EXPECT_EQ(copy->getSlicesCount(), 24U);
  EXPECT_FALSE(copy->hasTopBase());
  EXPECT_TRUE(copy->hasBottomBase());
}

// --------------------------------------------------------------- CPointCloud

TEST(CPointCloud, InsertResizeAndClear)
{
  auto o = CPointCloud::Create();
  EXPECT_TRUE(o->empty());

  o->reserve(10);
  for (int i = 0; i < 10; i++)
  {
    o->insertPoint(mrpt::math::TPoint3Df(static_cast<float>(i), 0, 0));
  }
  EXPECT_EQ(o->size(), 10U);
  EXPECT_FALSE(o->empty());
  EXPECT_FLOAT_EQ(o->getPoint3Df(3).x, 3.0f);

  o->setPoint(3, -1.0f, -2.0f, -3.0f);
  EXPECT_FLOAT_EQ(o->getPoint3Df(3).y, -2.0f);

  o->resize(5);
  EXPECT_EQ(o->size(), 5U);
  EXPECT_EQ(o->getArrayPoints().size(), 5U);

  o->clear();
  EXPECT_TRUE(o->empty());
}

TEST(CPointCloud, SetAllPoints)
{
  auto o = CPointCloud::Create();

  const std::vector<double> xs{0.0, 1.0, 2.0};
  const std::vector<double> ys{0.0, -1.0, -2.0};
  const std::vector<double> zs{1.0, 1.0, 1.0};
  o->setAllPoints(xs, ys, zs);
  ASSERT_EQ(o->size(), 3U);
  EXPECT_FLOAT_EQ(o->getPoint3Df(2).x, 2.0f);
  EXPECT_FLOAT_EQ(o->getPoint3Df(2).y, -2.0f);

  std::vector<mrpt::math::TPoint3D> pts{
      {9.0, 8.0, 7.0}
  };
  o->setAllPoints(pts);
  ASSERT_EQ(o->size(), 1U);
  EXPECT_FLOAT_EQ(o->getPoint3Df(0).z, 7.0f);

  std::vector<mrpt::math::TPoint3Df> ptsf{
      {1.f, 2.f, 3.f},
      {4.f, 5.f, 6.f}
  };
  o->setAllPointsFast(ptsf);
  EXPECT_EQ(o->size(), 2U);
}

TEST(CPointCloud, ColorFromCoordinate)
{
  auto o = CPointCloud::Create();
  for (int i = 0; i < 5; i++)
  {
    o->insertPoint(static_cast<float>(i), static_cast<float>(2 * i), static_cast<float>(-i));
  }
  o->setGradientColors(mrpt::img::TColorf(0, 0, 1), mrpt::img::TColorf(1, 0, 0));

  // Only one axis at a time drives the gradient:
  o->enableColorFromX();
  o->updateBuffers();
  {
    const auto& c = o->shaderPointsVertexColorBuffer();
    ASSERT_EQ(c.size(), 5U);
    // The two extremes of the coordinate range must not share a color:
    EXPECT_NE(c.front().R, c.back().R);
    EXPECT_EQ(c.front().R, 0);
    EXPECT_EQ(c.back().R, 255);
  }

  o->enableColorFromY();
  o->updateBuffers();
  EXPECT_NE(
      o->shaderPointsVertexColorBuffer().front().R, o->shaderPointsVertexColorBuffer().back().R);

  o->enableColorFromZ();
  o->updateBuffers();
  EXPECT_EQ(o->shaderPointsVertexColorBuffer().size(), 5U);

  // Turning it off again gives every point the object's uniform color:
  o->enableColorFromZ(false);
  o->setColor_u8(mrpt::img::TColor(7, 8, 9, 255));
  o->updateBuffers();
  EXPECT_EQ(o->shaderPointsVertexColorBuffer().front().R, 7);
  EXPECT_EQ(o->shaderPointsVertexColorBuffer().back().G, 8);
}

TEST(CPointCloud, SerializationRoundTrip)
{
  CPointCloud o;
  o.insertPoint(1.0f, 2.0f, 3.0f);
  o.insertPoint(4.0f, 5.0f, 6.0f);
  o.setPointSize(7.0f);
  o.enableColorFromZ(true);

  const auto copy = serializeRoundTrip(o);
  ASSERT_EQ(copy->size(), 2U);
  EXPECT_FLOAT_EQ(copy->getPoint3Df(1).y, 5.0f);
  EXPECT_FLOAT_EQ(copy->getPointSize(), 7.0f);
}

// ------------------------------------------------------------- CSetOfObjects

TEST(CSetOfObjects, ContainerOperations)
{
  auto set = CSetOfObjects::Create();
  EXPECT_TRUE(set->empty());

  auto sph = CSphere::Create(1.0f);
  sph->setName("ball");
  auto cyl = CCylinder::Create();
  cyl->setName("tube");

  set->insert(sph);
  set->insert(cyl);
  EXPECT_EQ(set->size(), 2U);
  EXPECT_TRUE(set->contains(sph));

  EXPECT_EQ(set->getByName("ball"), sph);
  EXPECT_FALSE(set->getByName("nope"));
  EXPECT_EQ(set->getByClass<CCylinder>(), cyl);
  EXPECT_FALSE(set->getByClass<CText>());

  std::vector<std::string> names;
  set->dumpListOfObjects(names);
  EXPECT_EQ(names.size(), 2U);

  const auto y = set->asYAML();
  EXPECT_TRUE(y.isSequence());

  // Colors propagate down to the children:
  set->setColor_u8(mrpt::img::TColor(10, 20, 30, 200));
  EXPECT_EQ(sph->getColor_u8().R, 10);
  set->setColorA_u8(100);
  EXPECT_EQ(cyl->getColor_u8().A, 100);

  set->removeObject(sph);
  EXPECT_EQ(set->size(), 1U);
  EXPECT_FALSE(set->contains(sph));

  set->clear();
  EXPECT_TRUE(set->empty());
}

TEST(CSetOfObjects, InsertingItselfThrows)
{
  auto set = CSetOfObjects::Create();
  EXPECT_THROW(set->insert(set), std::exception);
}

TEST(CSetOfObjects, BoundingBoxSpansAllChildren)
{
  auto set = CSetOfObjects::Create();

  auto a = CSphere::Create(1.0f);
  a->setLocation(-5, 0, 0);
  auto b = CSphere::Create(1.0f);
  b->setLocation(5, 0, 0);
  set->insert(a);
  set->insert(b);

  const auto bb = set->getBoundingBoxLocal();
  EXPECT_LE(bb.min.x, -5.0);
  EXPECT_GE(bb.max.x, 5.0);
}

// ------------------------------------------------------------ CSetOfTriangles

TEST(CSetOfTriangles, InsertAndQuery)
{
  auto o = CSetOfTriangles::Create();
  EXPECT_EQ(o->getTrianglesCount(), 0U);

  const TTriangle t1(
      mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(1, 0, 0),
      mrpt::math::TPoint3Df(0, 1, 0));
  const TTriangle t2(
      mrpt::math::TPoint3Df(0, 0, 1), mrpt::math::TPoint3Df(1, 0, 1),
      mrpt::math::TPoint3Df(0, 1, 1));
  o->insertTriangle(t1);
  o->insertTriangle(t2);
  EXPECT_EQ(o->getTrianglesCount(), 2U);

  TTriangle got;
  o->getTriangle(1, got);
  EXPECT_FLOAT_EQ(got.z(0), 1.0f);

  // getPolygons() must size its output itself:
  std::vector<mrpt::math::TPolygon3D> polys;
  o->getPolygons(polys);
  ASSERT_EQ(polys.size(), 2U);
  EXPECT_EQ(polys[0].size(), 3U);

  // A ray straight down onto the upper triangle:
  double dist = 0;
  EXPECT_TRUE(o->traceRay(mrpt::poses::CPose3D(0.1, 0.1, 5.0, 0, mrpt::DEG2RAD(90.0), 0), dist));
  EXPECT_GT(dist, 0.0);

  o->setColor_u8(mrpt::img::TColor(1, 2, 3, 255));
  TTriangle colored;
  o->getTriangle(0, colored);
  EXPECT_EQ(colored.vertices[0].xyzrgba.r, 1);

  o->clearTriangles();
  EXPECT_EQ(o->getTrianglesCount(), 0U);
}

TEST(CSetOfTriangles, InsertFromAnotherSet)
{
  auto src = CSetOfTriangles::Create();
  src->insertTriangle(TTriangle(
      mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(1, 0, 0),
      mrpt::math::TPoint3Df(0, 1, 0)));

  auto dst = CSetOfTriangles::Create();
  dst->insertTriangles(src);
  EXPECT_EQ(dst->getTrianglesCount(), 1U);

  const std::vector<TTriangle> more{TTriangle(
      mrpt::math::TPoint3Df(0, 0, 2), mrpt::math::TPoint3Df(1, 0, 2),
      mrpt::math::TPoint3Df(0, 1, 2))};
  dst->insertTriangles(more.begin(), more.end());
  EXPECT_EQ(dst->getTrianglesCount(), 2U);
}

// ---------------------------------------------------- CSetOfTexturedTriangles

TEST(CSetOfTexturedTriangles, InsertQueryAndTraceRay)
{
  auto o = CSetOfTexturedTriangles::Create();
  EXPECT_EQ(o->getTrianglesCount(), 0U);

  o->insertTriangle(TTriangle(
      mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(1, 0, 0),
      mrpt::math::TPoint3Df(0, 1, 0)));
  ASSERT_EQ(o->getTrianglesCount(), 1U);

  const auto t = o->getTriangle(0);
  EXPECT_FLOAT_EQ(t.x(1), 1.0f);
  EXPECT_THROW(o->getTriangle(99), std::exception);

  double dist = 0;
  EXPECT_TRUE(o->traceRay(mrpt::poses::CPose3D(0.1, 0.1, 5.0, 0, mrpt::DEG2RAD(90.0), 0), dist));
  EXPECT_GT(dist, 0.0);

  mrpt::img::CImage img(4, 4, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {3, 3}, mrpt::img::TColor(255, 0, 0));
  o->assignImage(img);
  EXPECT_EQ(o->getTextureImage().getWidth(), 4U);

  o->clearTriangles();
  EXPECT_EQ(o->getTrianglesCount(), 0U);
}

// -------------------------------------------------------------------- CMesh

TEST(CMesh, GridLimitsAndMask)
{
  auto o = CMesh::Create(true, -2, 2, -3, 3);

  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  o->getGridLimits(xMin, xMax, yMin, yMax);
  EXPECT_FLOAT_EQ(xMin, -2.0f);
  EXPECT_FLOAT_EQ(yMax, 3.0f);

  o->setGridLimits(-1, 1, -1, 1);
  EXPECT_FLOAT_EQ(o->getxMin(), -1.0f);
  EXPECT_FLOAT_EQ(o->getyMax(), 1.0f);

  o->setxMin(-4);
  o->setxMax(4);
  o->setyMin(-5);
  o->setyMax(5);
  EXPECT_FLOAT_EQ(o->getxMax(), 4.0f);
  EXPECT_FLOAT_EQ(o->getyMin(), -5.0f);
  o->setXBounds(-1, 1);
  o->setYBounds(-1, 1);
  float lo = 0;
  float hi = 0;
  o->getXBounds(lo, hi);
  EXPECT_FLOAT_EQ(hi, 1.0f);

  o->setMeshTextureExtension(0.5f, 0.25f);
  float tx = 0;
  float ty = 0;
  o->getMeshTextureExtension(tx, ty);
  EXPECT_FLOAT_EQ(tx, 0.5f);
  EXPECT_FLOAT_EQ(ty, 0.25f);

  mrpt::math::CMatrixFloat Z(3, 3);
  Z.fill(1.0f);
  o->setZ(Z);
  mrpt::math::CMatrixFloat Zback;
  o->getZ(Zback);
  EXPECT_EQ(Zback.rows(), 3);

  // A mask removes cells from the generated triangle mesh:
  mrpt::math::CMatrixFloat mask(3, 3);
  mask.fill(1.0f);
  o->setZ(Z);
  o->updateBuffers();
  const size_t nFull = o->shaderTexturedTrianglesBuffer().size();

  mask(0, 0) = 0.0f;
  o->setMask(mask);
  mrpt::math::CMatrixFloat maskBack;
  o->getMask(maskBack);
  EXPECT_EQ(maskBack.rows(), 3);
  o->updateBuffers();
  EXPECT_LT(o->shaderTexturedTrianglesBuffer().size(), nFull);

  o->enableWireFrame(true);
  o->enableTransparency(true);
  o->enableColorFromZ(true, mrpt::img::cmJET);
  EXPECT_NO_THROW(o->updateBuffers());
}

TEST(CMesh, AssignImageAndTraceRay)
{
  auto o = CMesh::Create(false, -1, 1, -1, 1);

  mrpt::img::CImage img(8, 4, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {7, 3}, mrpt::img::TColor(0, 255, 0));
  o->assignImage(img);
  EXPECT_NO_THROW(o->updateBuffers());

  o->adjustGridToImageAR();
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  o->getGridLimits(xMin, xMax, yMin, yMax);
  // The image is 2:1, so the Y span must be half the X span:
  EXPECT_NEAR((yMax - yMin) * 2.0f, xMax - xMin, 1e-4f);

  mrpt::math::CMatrixFloat Z(4, 4);
  Z.fill(0.0f);
  o->assignImageAndZ(img, Z);
  EXPECT_NO_THROW(o->updateBuffers());

  // A flat mesh at z=0, hit by a ray pointing straight down:
  o->setZ(Z);
  o->setGridLimits(-1, 1, -1, 1);
  double dist = 0;
  EXPECT_TRUE(o->traceRay(mrpt::poses::CPose3D(0, 0, 5.0, 0, mrpt::DEG2RAD(90.0), 0), dist));
  EXPECT_NEAR(dist, 5.0, 1e-2);
}

TEST(CMesh, SerializationRoundTrip)
{
  CMesh o(true, -2, 2, -3, 3);
  mrpt::math::CMatrixFloat Z(3, 4);
  Z.fill(0.5f);
  o.setZ(Z);
  o.enableWireFrame(true);
  o.enableColorFromZ(true, mrpt::img::cmJET);

  const auto copy = serializeRoundTrip(o);
  EXPECT_FLOAT_EQ(copy->getxMin(), -2.0f);
  EXPECT_FLOAT_EQ(copy->getyMax(), 3.0f);
  mrpt::math::CMatrixFloat Zback;
  copy->getZ(Zback);
  EXPECT_EQ(Zback.rows(), 3);
  EXPECT_EQ(Zback.cols(), 4);
}

// ---------------------------------------------------------------- CMeshFast

TEST(CMeshFast, SettersAndRoundTrip)
{
  auto o = CMeshFast::Create(false, -2, 2, -2, 2);

  o->setXMin(-1);
  o->setXMax(1);
  o->setYMin(-3);
  o->setYMax(3);
  EXPECT_FLOAT_EQ(o->getXMin(), -1.0f);
  EXPECT_FLOAT_EQ(o->getYMax(), 3.0f);
  o->setXBounds(-2, 2);
  o->setYBounds(-2, 2);

  float lo = 0;
  float hi = 0;
  o->getYBounds(lo, hi);
  EXPECT_FLOAT_EQ(hi, 2.0f);

  mrpt::math::CMatrixFloat Z(4, 4);
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      Z(r, c) = static_cast<float>(r + c);
    }
  }
  o->setZ(Z);
  o->enableColorFromZ(true, mrpt::img::cmHOT);
  o->updateBuffers();
  EXPECT_EQ(o->shaderPointsVertexPointBuffer().size(), 16U);

  const auto bb = o->getBoundingBoxLocal();
  EXPECT_NEAR(bb.max.z, 6.0, 1e-4);

  const auto copy = serializeRoundTrip(*o);
  mrpt::math::CMatrixFloat Zback;
  copy->getZ(Zback);
  EXPECT_EQ(Zback.rows(), 4);
}

TEST(CMeshFast, AssignImage)
{
  auto o = CMeshFast::Create(false, -1, 1, -1, 1);

  mrpt::img::CImage img(6, 3, mrpt::img::CH_RGB);
  img.filledRectangle({0, 0}, {5, 2}, mrpt::img::TColor(0, 0, 255));
  o->assignImage(img);
  EXPECT_NO_THROW(o->updateBuffers());

  o->adjustGridToImageAR();
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  o->getGridLimits(xMin, xMax, yMin, yMax);
  EXPECT_NEAR((yMax - yMin) * 2.0f, xMax - xMin, 1e-4f);
}

// ------------------------------------------------------------------ CMesh3D

TEST(CMesh3D, LoadFromMatrices)
{
  auto o = CMesh3D::Create();

  // 2 faces: one triangle and one quad, sharing an edge.
  mrpt::math::CMatrixDynamic<bool> isQuad(2, 1);
  isQuad(0, 0) = false;
  isQuad(1, 0) = true;

  mrpt::math::CMatrixDynamic<int> faceVerts(4, 2);
  faceVerts(0, 0) = 0;
  faceVerts(1, 0) = 1;
  faceVerts(2, 0) = 2;
  faceVerts(3, 0) = 0;
  faceVerts(0, 1) = 1;
  faceVerts(1, 1) = 3;
  faceVerts(2, 1) = 4;
  faceVerts(3, 1) = 2;

  mrpt::math::CMatrixDynamic<float> coords(3, 5);
  const float pts[5][3] = {
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {2, 0, 0},
      {2, 1, 0}
  };
  for (int i = 0; i < 5; i++)
  {
    for (int r = 0; r < 3; r++)
    {
      coords(r, i) = pts[i][r];
    }
  }

  o->loadMesh(5, 2, isQuad, faceVerts, coords);
  o->updateBuffers();
  // 1 triangle + 1 quad (2 triangles):
  EXPECT_EQ(o->shaderTrianglesBuffer().size(), 3U);
  // 3 + 4 edges, two vertices each:
  EXPECT_EQ(o->shaderLinesVertexPointBuffer().size(), 14U);

  o->setEdgeColor(1, 0, 0);
  o->setFaceColor(0, 1, 0);
  o->setVertColor(0, 0, 1);
  o->enableShowVertices(true);
  o->enableFaceNormals(false);
  EXPECT_NO_THROW(o->updateBuffers());

  const auto copy = serializeRoundTrip(*o);
  copy->updateBuffers();
  EXPECT_EQ(copy->shaderTrianglesBuffer().size(), 3U);
}

TEST(CMesh3D, RejectsFacesThatAreNeitherTrianglesNorQuads)
{
  auto o = CMesh3D::Create();
  std::vector<int> vertsPerFace = {5};
  std::vector<int> faceVerts = {0, 1, 2, 3, 4};
  std::vector<float> coords(15, 0.0f);
  EXPECT_THROW(
      o->loadMesh(5, 1, vertsPerFace.data(), faceVerts.data(), coords.data()), std::exception);
}

// ------------------------------------------------------------ CVectorField3D

TEST(CVectorField3D, SettersAndGetters)
{
  auto o = CVectorField3D::Create();

  mrpt::math::CMatrixFloat m(2, 3);
  m.fill(0.25f);
  o->setPointCoordinates(m, m, m);
  o->setVectorField(m, m, m);

  EXPECT_EQ(o->getVectorField_x().rows(), 2);
  EXPECT_EQ(o->getVectorField_z().cols(), 3);

  mrpt::math::CMatrixFloat vx;
  mrpt::math::CMatrixFloat vy;
  mrpt::math::CMatrixFloat vz;
  o->getVectorField(vx, vy, vz);
  EXPECT_EQ(vx.cols(), 3);
  o->getPointCoordinates(vx, vy, vz);
  EXPECT_EQ(vy.rows(), 2);

  o->setPointColor(1, 0, 0);
  EXPECT_FLOAT_EQ(o->getPointColor().R, 1.0f);
  o->setVectorFieldColor(0, 1, 0);
  EXPECT_FLOAT_EQ(o->getVectorFieldColor().G, 1.0f);
  o->setMaxSpeedForColor(3.5f);
  EXPECT_FLOAT_EQ(o->getMaxSpeedForColor(), 3.5f);

  o->enableColorFromModule(true);
  o->setMotionFieldColormap(0, 0, 1, 1, 0, 0);
  EXPECT_NO_THROW(o->updateBuffers());

  o->clear();
  EXPECT_EQ(o->getVectorField_x().rows(), 0);
}

TEST(CVectorField3D, SerializationRoundTrip)
{
  CVectorField3D o;
  mrpt::math::CMatrixFloat m(2, 2);
  m.fill(1.0f);
  o.setPointCoordinates(m, m, m);
  o.setVectorField(m, m, m);
  o.setMaxSpeedForColor(2.0f);
  o.enableColorFromModule(true);

  const auto copy = serializeRoundTrip(o);
  EXPECT_EQ(copy->getVectorField_y().rows(), 2);
  EXPECT_FLOAT_EQ(copy->getMaxSpeedForColor(), 2.0f);
  EXPECT_TRUE(copy->isColorFromModuleEnabled());
}

// ---------------------------------------------------------------- CColorBar

TEST(CColorBar, SettersAndBoundingBox)
{
  auto o = CColorBar::Create(mrpt::img::cmHOT, 0.4, 2.0, 0.0f, 1.0f, -10.0f, 10.0f, "%5.1f", 0.1f);
  o->updateBuffers();
  const size_t nTris = o->shaderTrianglesBuffer().size();
  EXPECT_GT(nTris, 0U);

  o->setColormap(mrpt::img::cmJET);
  o->setColorAndValueLimits(0.2f, 0.8f, -1.0f, 1.0f);
  o->updateBuffers();
  EXPECT_EQ(o->shaderTrianglesBuffer().size(), nTris);

  const auto bb = o->getBoundingBoxLocal();
  EXPECT_NEAR(bb.max.y, 2.0, 1e-4);

  const auto copy = serializeRoundTrip(*o);
  EXPECT_NO_THROW(copy->updateBuffers());
}

// --------------------------------------------------------- CTextMessageCapable

TEST(CTextMessageCapable, AddUpdateAndClear)
{
  CTextMessageCapable c;
  EXPECT_TRUE(c.getTextMessages().messages.empty());

  TFontParams fp;
  fp.vfont_scale = 12;
  c.addTextMessage(0.05, 0.05, "hello", 0 /*unique_index*/, fp);
  c.addTextMessage(0.05, 0.15, "world", 1 /*unique_index*/, fp);
  EXPECT_EQ(c.getTextMessages().messages.size(), 2U);

  EXPECT_TRUE(c.updateTextMessage(1, "there"));
  EXPECT_FALSE(c.updateTextMessage(99, "nobody"));

  // Labels start out flagged as outdated, so this must create their objects:
  c.getTextMessages().regenerateGLobjects();
  for (const auto& kv : c.getTextMessages().messages)
  {
    EXPECT_TRUE(kv.second.gl_text);
    EXPECT_FALSE(kv.second.gl_text_outdated);
  }
  // ...and a second pass must leave them alone:
  EXPECT_NO_THROW(c.getTextMessages().regenerateGLobjects());

  c.clearTextMessages();
  EXPECT_TRUE(c.getTextMessages().messages.empty());
}

// -------------------------------------------------------------- CText/CText3D

TEST(CText, SettersAndRoundTrip)
{
  auto o = CText::Create("hello");
  EXPECT_EQ(o->getString(), "hello");

  o->setString("bye");
  EXPECT_EQ(o->getString(), "bye");
  // Setting the same string again is a no-op:
  o->setString("bye");
  EXPECT_EQ(o->getString(), "bye");

  o->setFont("serif", 14);
  EXPECT_EQ(o->getFont(), "serif");
  o->setFont("serif", 14);

  mrpt::containers::yaml props = mrpt::containers::yaml::Map();
  o->toYAMLMap(props);
  EXPECT_TRUE(props.has("text"));

  const auto copy = serializeRoundTrip(*o);
  EXPECT_EQ(copy->getString(), "bye");
}

TEST(CText3D, SettersAndRoundTrip)
{
  auto o = CText3D::Create("abc", "sans", 0.5);
  EXPECT_EQ(o->getString(), "abc");
  EXPECT_EQ(o->getFont(), "sans");

  o->setString("xyz");
  o->setFont("mono");
  o->setTextStyle(mrpt::viz::FILL);
  o->setTextSpacing(1.25);
  o->setTextKerning(0.5);
  EXPECT_EQ(o->getTextStyle(), mrpt::viz::FILL);
  EXPECT_DOUBLE_EQ(o->setTextSpacing(), 1.25);
  EXPECT_DOUBLE_EQ(o->setTextKerning(), 0.5);

  mrpt::containers::yaml props = mrpt::containers::yaml::Map();
  o->toYAMLMap(props);
  EXPECT_TRUE(props.has("text"));

  const auto copy = serializeRoundTrip(*o);
  EXPECT_EQ(copy->getString(), "xyz");
  EXPECT_EQ(copy->getFont(), "mono");
}

// ----------------------------------------------------------- COctoMapVoxels

namespace
{
/** COctoMapVoxels' mutators are protected, for use by the map classes that
 *  build the visualization; this exposes them to the test. */
class TestableOctoMapVoxels : public COctoMapVoxels
{
 public:
  using COctoMapVoxels::push_back_GridCube;
  using COctoMapVoxels::push_back_Voxel;
  using COctoMapVoxels::resizeVoxelSets;
  using COctoMapVoxels::setBoundingBox;
  using COctoMapVoxels::sort_voxels_by_z;
};
}  // namespace

TEST(COctoMapVoxels, GridCubesVoxelsAndDisplayModes)
{
  TestableOctoMapVoxels o;
  o.resizeVoxelSets(2);
  EXPECT_EQ(o.getVoxelSetCount(), 2U);

  o.push_back_Voxel(0, COctoMapVoxels::TVoxel({0, 0, 0}, 0.5, mrpt::img::TColor(255, 0, 0, 255)));
  o.push_back_Voxel(0, COctoMapVoxels::TVoxel({1, 1, 2}, 0.5, mrpt::img::TColor(0, 255, 0, 255)));
  o.push_back_Voxel(1, COctoMapVoxels::TVoxel({2, 2, 1}, 0.5, mrpt::img::TColor(0, 0, 255, 255)));
  EXPECT_EQ(o.getVoxelCount(0), 2U);
  EXPECT_EQ(o.getVoxelCount(1), 1U);

  o.push_back_GridCube(COctoMapVoxels::TGridCube({-1, -1, -1}, {1, 1, 1}));
  EXPECT_EQ(o.getGridCubeCount(), 1U);
  EXPECT_NEAR(o.getGridCube(0).max.z, 1.0f, 1e-5f);

  o.setBoundingBox({-2, -2, -2}, {3, 3, 3});
  const auto bb = o.getBoundingBoxLocal();
  EXPECT_NEAR(bb.min.x, -2.0, 1e-4);
  EXPECT_NEAR(bb.max.z, 3.0, 1e-4);

  // Solid cubes + grid lines:
  o.showGridLines(true);
  o.showVoxels(0, true);
  o.showVoxels(1, true);
  EXPECT_TRUE(o.areGridLinesVisible());
  EXPECT_TRUE(o.areVoxelsVisible(0));
  o.setGridLinesWidth(2.5f);
  o.setGridLinesColor(mrpt::img::TColor(1, 2, 3, 255));
  EXPECT_FLOAT_EQ(o.getGridLinesWidth(), 2.5f);
  EXPECT_EQ(o.getGridLinesColor().G, 2);

  o.updateBuffers();
  EXPECT_GT(o.shaderTrianglesBuffer().size(), 0U);
  // One cube = 12 line pairs = 24 vertices:
  EXPECT_EQ(o.shaderLinesVertexPointBuffer().size(), 24U);
  EXPECT_TRUE(o.shaderPointsVertexPointBuffer().empty());

  // Points mode instead of solid cubes:
  o.showVoxelsAsPoints(true);
  o.setVoxelAsPointsSize(5.0f);
  EXPECT_TRUE(o.areVoxelsShownAsPoints());
  EXPECT_FLOAT_EQ(o.getVoxelAsPointsSize(), 5.0f);
  o.updateBuffers();
  EXPECT_EQ(o.shaderPointsVertexPointBuffer().size(), 3U);

  // A hidden set contributes nothing:
  o.showVoxels(1, false);
  EXPECT_FALSE(o.areVoxelsVisible(1));
  o.updateBuffers();
  EXPECT_EQ(o.shaderPointsVertexPointBuffer().size(), 2U);

  // Grid lines off:
  o.showGridLines(false);
  o.updateBuffers();
  EXPECT_TRUE(o.shaderLinesVertexPointBuffer().empty());

  o.sort_voxels_by_z();
  EXPECT_LE(o.getVoxel(0, 0).coords.z, o.getVoxel(0, 1).coords.z);

  o.enableCubeTransparency(true);
  EXPECT_TRUE(o.isCubeTransparencyEnabled());
  o.enableLights(false);
  EXPECT_FALSE(o.areLightsEnabled());
  o.colorMap(mrpt::img::cmJET);
  EXPECT_EQ(o.colorMap(), mrpt::img::cmJET);

  o.clear();
  EXPECT_EQ(o.getVoxelSetCount(), 0U);
  EXPECT_EQ(o.getGridCubeCount(), 0U);
}
