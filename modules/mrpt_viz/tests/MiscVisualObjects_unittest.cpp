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
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/TTriangle.h>

using namespace mrpt::viz;

TEST(CAxis, SettersAndGetters)
{
  auto axis = CAxis::Create();
  axis->setAxisLimits(-1, -2, -3, 1, 2, 3);
  axis->setFrequency(2.0f);
  EXPECT_FLOAT_EQ(axis->getFrequency(), 2.0f);

  axis->setTextScale(0.5f);
  EXPECT_FLOAT_EQ(axis->getTextScale(), 0.5f);

  axis->setTextLabelOrientation(0, 10.f, 20.f, 30.f);
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  axis->getTextLabelOrientation(0, yaw, pitch, roll);
  EXPECT_FLOAT_EQ(yaw, 10.f);

  axis->enableTickMarks(true);
  axis->enableTickMarks(true, false, true);
  axis->setTickMarksLength(0.2f);
  EXPECT_FLOAT_EQ(axis->getTickMarksLength(), 0.2f);

  EXPECT_NO_THROW(axis->updateBuffers());
}

TEST(CVectorField2D, GridAndField)
{
  auto vf = CVectorField2D::Create();
  vf->setPointColor(0.1f, 0.2f, 0.3f);
  vf->setVectorFieldColor(0.4f, 0.5f, 0.6f);
  vf->setGridCenterAndCellSize(0, 0, 1.0f, 1.0f);
  vf->setGridLimits(-5, 5, -5, 5);

  float xmin = 0;
  float xmax = 0;
  float ymin = 0;
  float ymax = 0;
  vf->getGridLimits(xmin, xmax, ymin, ymax);
  EXPECT_FLOAT_EQ(xmin, -5);

  mrpt::math::CMatrixFloat mx(2, 2);
  mrpt::math::CMatrixFloat my(2, 2);
  mx.setConstant(1.0f);
  my.setConstant(-1.0f);
  vf->resize(2, 2);
  vf->setVectorField(mx, my);

  mrpt::math::CMatrixFloat outX;
  mrpt::math::CMatrixFloat outY;
  vf->getVectorField(outX, outY);
  EXPECT_EQ(outX.rows(), 2);

  vf->adjustVectorFieldToGrid();
  vf->clear();
}

TEST(CVectorField3D, FieldAndCoordinates)
{
  auto vf = CVectorField3D::Create();
  vf->setPointColor(0.1f, 0.2f, 0.3f);
  vf->setVectorFieldColor(0.4f, 0.5f, 0.6f);
  vf->setMaxSpeedForColor(5.0f);
  EXPECT_FLOAT_EQ(vf->getMaxSpeedForColor(), 5.0f);

  mrpt::math::CMatrixFloat mx(2, 2);
  mrpt::math::CMatrixFloat my(2, 2);
  mrpt::math::CMatrixFloat mz(2, 2);
  mx.setConstant(1.0f);
  my.setConstant(0.5f);
  mz.setConstant(0.0f);
  vf->resize(2, 2);
  vf->setPointCoordinates(mx, my, mz);
  vf->setVectorField(mx, my, mz);

  mrpt::math::CMatrixFloat outX;
  mrpt::math::CMatrixFloat outY;
  mrpt::math::CMatrixFloat outZ;
  vf->getVectorField(outX, outY, outZ);
  EXPECT_EQ(outX.rows(), 2);

  vf->enableColorFromModule(true);
  vf->enableShowPoints(true);
  vf->clear();
}

TEST(CFrustum, FovAndPlanes)
{
  auto f = CFrustum::Create(0.1f, 5.0f, 60.0f, 45.0f, 1.0f, true, true);
  f->setPlaneColor(mrpt::img::TColor(10, 20, 30));
  f->setNearFarPlanes(0.5f, 10.0f);
  EXPECT_FLOAT_EQ(f->getNearPlaneDistance(), 0.5f);
  EXPECT_FLOAT_EQ(f->getFarPlaneDistance(), 10.0f);

  f->setHorzFOV(70.0f);
  EXPECT_NEAR(f->getHorzFOV(), 70.0f, 1e-2);

  f->setVertFOV(50.0f);
  EXPECT_NEAR(f->getVertFOV(), 50.0f, 1e-2);

  f->setHorzFOVAsymmetric(10.f, 20.f);
  EXPECT_NEAR(f->getHorzFOVLeft(), 10.f, 1e-2);
  EXPECT_NEAR(f->getHorzFOVRight(), 20.f, 1e-2);

  f->setVertFOVAsymmetric(5.f, 15.f);
  EXPECT_NEAR(f->getVertFOVDown(), 5.f, 1e-2);
  EXPECT_NEAR(f->getVertFOVUp(), 15.f, 1e-2);
}

TEST(CSetOfObjects, InsertRemoveContains)
{
  auto group = CSetOfObjects::Create();
  auto sphere = CSphere::Create();
  group->insert(sphere);
  EXPECT_TRUE(group->contains(sphere));
  EXPECT_FALSE(group->empty());

  std::vector<std::string> lst;
  group->dumpListOfObjects(lst);
  EXPECT_FALSE(lst.empty());

  group->removeObject(sphere);
  EXPECT_FALSE(group->contains(sphere));

  std::vector<CVisualObject::Ptr> objs{CSphere::Create(), CBox::Create()};
  group->insertCollection(objs);
  EXPECT_EQ(group->size(), 2u);

  group->clear();
  EXPECT_TRUE(group->empty());
}

TEST(CSetOfObjects, StreamInsertOperator)
{
  auto group = CSetOfObjects::Create();
  group << CSphere::Create();
  EXPECT_EQ(group->size(), 1u);
}

TEST(CCamera, ProjectionModes)
{
  CCamera cam;
  cam.setProjectiveModel(true);
  EXPECT_TRUE(cam.isProjective());
  cam.setOrthogonal(true);
  EXPECT_FALSE(cam.isProjective());

  mrpt::img::TCamera intrinsics;
  intrinsics.ncols = 640;
  intrinsics.nrows = 480;
  intrinsics.fx(500);
  intrinsics.fy(500);
  intrinsics.cx(320);
  intrinsics.cy(240);
  cam.setProjectiveFromPinhole(intrinsics);
  EXPECT_TRUE(cam.isProjective());

  cam.setNoProjection();
  cam.setProjectiveFOVdeg(60.f);
  EXPECT_FLOAT_EQ(cam.getProjectiveFOVdeg(), 60.f);

  cam.setPointingAt(1.f, 2.f, 3.f);
  EXPECT_FLOAT_EQ(cam.getPointingAtX(), 1.f);

  cam.setPointingAt(mrpt::math::TPoint3D(4, 5, 6));
  EXPECT_FLOAT_EQ(cam.getPointingAtY(), 5.f);

  cam.setZoomDistance(10.f);
  EXPECT_FLOAT_EQ(cam.getZoomDistance(), 10.f);

  cam.setAzimuthDegrees(30.f);
  EXPECT_FLOAT_EQ(cam.getAzimuthDegrees(), 30.f);

  cam.setElevationDegrees(15.f);
  EXPECT_FLOAT_EQ(cam.getElevationDegrees(), 15.f);

  cam.setRollDegrees(5.f);
  EXPECT_FLOAT_EQ(cam.getRollDegrees(), 5.f);

  cam.set6DOFMode(true);
}

TEST(TTriangle, ColorAndNormals)
{
  TTriangle t;
  t.x(0) = 0.f;
  t.y(0) = 0.f;
  t.z(0) = 0.f;
  t.x(1) = 1.f;
  t.y(1) = 0.f;
  t.z(1) = 0.f;
  t.x(2) = 0.f;
  t.y(2) = 1.f;
  t.z(2) = 0.f;
  t.u(0) = 0.f;
  t.v(0) = 0.f;

  t.setColor(mrpt::img::TColor(255, 0, 0));
  EXPECT_EQ(t.vertices[0].xyzrgba.r, 255);

  t.setColor(mrpt::img::TColorf(0.f, 1.f, 0.f));

  t.computeNormals();
  t.computeTangents();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  t.writeTo(arch);
  buf.Seek(0);
  TTriangle t2;
  t2.readFrom(arch);
  EXPECT_FLOAT_EQ(t2.x(1), 1.f);
}

TEST(COctoMapVoxels, GridAndVoxelSets)
{
  auto obj = COctoMapVoxels::Create();
  obj->resizeVoxelSets(1);
  obj->reserveVoxels(0, 4);
  for (int i = 0; i < 4; i++)
  {
    COctoMapVoxels::TVoxel v(
        mrpt::math::TPoint3Df(static_cast<float>(i), 0, 0), 0.2f, mrpt::img::TColor(10, 20, 30));
    obj->push_back_Voxel(0, v);
  }
  obj->showVoxels(0, true);
  EXPECT_TRUE(obj->areVoxelsVisible(0));

  obj->showVoxelsAsPoints(true);
  EXPECT_TRUE(obj->areVoxelsShownAsPoints());
  obj->setVoxelAsPointsSize(3.0f);
  EXPECT_FLOAT_EQ(obj->getVoxelAsPointsSize(), 3.0f);

  obj->enableLights(false);
  EXPECT_FALSE(obj->areLightsEnabled());

  obj->enableCubeTransparency(true);
  EXPECT_TRUE(obj->isCubeTransparencyEnabled());

  obj->showGridLines(true);
  EXPECT_TRUE(obj->areGridLinesVisible());

  obj->setGridLinesWidth(2.0f);
  EXPECT_FLOAT_EQ(obj->getGridLinesWidth(), 2.0f);
  obj->setGridLinesColor(mrpt::img::TColor(1, 2, 3));

  obj->setBoundingBox(mrpt::math::TPoint3D(-1, -1, -1), mrpt::math::TPoint3D(1, 1, 1));
  obj->resizeGridCubes(2);

  obj->clear();
}

TEST(CSkyBox, AssignFaceImages)
{
  auto sb = CSkyBox::Create();
  mrpt::img::CImage img(4, 4, mrpt::img::CH_RGB);
  sb->assignImage(CUBE_TEXTURE_FACE::FRONT, img);
  sb->assignImage(CUBE_TEXTURE_FACE::BACK, mrpt::img::CImage(img));
}

TEST(CDisk, RadiusAndSlices)
{
  auto disk = CDisk::Create();
  disk->setDiskRadius(2.0f, 0.5f);
  disk->setSlicesCount(32);
  EXPECT_NO_THROW(disk->updateBuffers());
}

TEST(CBox, CornersAndBorder)
{
  auto box = CBox::Create();
  box->setBoxCorners(mrpt::math::TPoint3D(-1, -1, -1), mrpt::math::TPoint3D(1, 1, 1));
  mrpt::math::TPoint3D c1;
  mrpt::math::TPoint3D c2;
  box->getBoxCorners(c1, c2);
  EXPECT_DOUBLE_EQ(c1.x, -1.0);

  box->setWireframe(true);
  box->enableBoxBorder(true);
  box->setBoxBorderColor(mrpt::img::TColor(1, 2, 3));
  EXPECT_NO_THROW(box->updateBuffers());
}

TEST(CGridPlaneXY, LimitsAndFrequency)
{
  auto g = CGridPlaneXY::Create();
  g->setPlaneLimits(-2, 2, -3, 3);
  float xmin = 0;
  float xmax = 0;
  float ymin = 0;
  float ymax = 0;
  g->getPlaneLimits(xmin, xmax, ymin, ymax);
  EXPECT_FLOAT_EQ(xmin, -2.f);

  g->setPlaneZcoord(1.5f);
  EXPECT_FLOAT_EQ(g->getPlaneZcoord(), 1.5f);

  g->setGridFrequency(2.0f);
  EXPECT_FLOAT_EQ(g->getGridFrequency(), 2.0f);
}

TEST(CGridPlaneXZ, LimitsAndFrequency)
{
  auto g = CGridPlaneXZ::Create();
  g->setPlaneLimits(-2, 2, -3, 3);
  float xmin = 0;
  float xmax = 0;
  float zmin = 0;
  float zmax = 0;
  g->getPlaneLimits(xmin, xmax, zmin, zmax);
  EXPECT_FLOAT_EQ(xmin, -2.f);

  g->setPlaneYcoord(1.5f);
  EXPECT_FLOAT_EQ(g->getPlaneYcoord(), 1.5f);

  g->setGridFrequency(2.0f);
  EXPECT_FLOAT_EQ(g->getGridFrequency(), 2.0f);
}

TEST(CTexturedPlane, CornersRepeatAndLighting)
{
  auto plane = CTexturedPlane::Create();
  plane->setPlaneCorners(-1, 1, -2, 2);
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  plane->getPlaneCorners(xMin, xMax, yMin, yMax);
  EXPECT_FLOAT_EQ(xMin, -1.f);

  plane->setTextureRepeat(2.0f, 3.0f);
  plane->enableLighting(false);
  plane->cullFaces(TCullFace::BACK);
  EXPECT_NO_THROW(plane->updateBuffers());
}
