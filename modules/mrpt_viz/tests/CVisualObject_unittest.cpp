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
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CTexturedPlane.h>

using namespace mrpt::viz;

TEST(CVisualObject, NameVisibilityShadowsShowName)
{
  auto box = CBox::Create();
  box->setName("mybox");
  EXPECT_EQ(box->getName(), "mybox");

  EXPECT_TRUE(box->isVisible());
  box->setVisibility(false);
  EXPECT_FALSE(box->isVisible());

  const auto& cbox = *box;
  EXPECT_TRUE(cbox.castShadows());  // default=true
  box->castShadows(false);
  EXPECT_FALSE(cbox.castShadows());

  EXPECT_FALSE(box->isShowNameEnabled());
  box->enableShowName(true);
  EXPECT_TRUE(box->isShowNameEnabled());
}

TEST(CVisualObject, PoseOverloads)
{
  auto box = CBox::Create();

  box->setPose(mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_NEAR(box->getCPose().x(), 1.0, 1e-9);

  box->setPose(mrpt::poses::CPose2D(4, 5, 0.1));
  EXPECT_NEAR(box->getCPose().x(), 4.0, 1e-9);

  box->setPose(mrpt::math::TPose3D(6, 7, 8, 0, 0, 0));
  EXPECT_NEAR(box->getCPose().y(), 7.0, 1e-9);

  box->setPose(mrpt::math::TPose2D(9, 10, 0));
  EXPECT_NEAR(box->getCPose().x(), 9.0, 1e-9);

  box->setPose(mrpt::poses::CPoint3D(11, 12, 13));
  EXPECT_NEAR(box->getCPose().z(), 13.0, 1e-9);

  box->setPose(mrpt::poses::CPoint2D(14, 15));
  EXPECT_NEAR(box->getCPose().x(), 14.0, 1e-9);

  box->setLocation(1, 2, 3);
  EXPECT_NEAR(box->getCPose().x(), 1.0, 1e-9);
  box->setLocation(mrpt::math::TPoint3D(4, 5, 6));
  EXPECT_NEAR(box->getCPose().y(), 5.0, 1e-9);
}

TEST(CVisualObject, MaterialAndScale)
{
  auto box = CBox::Create();

  box->materialShininess(0.5f);
  EXPECT_FLOAT_EQ(box->materialShininess(), 0.5f);

  box->materialSpecularExponent(10.f);
  EXPECT_FLOAT_EQ(box->materialSpecularExponent(), 10.f);

  box->materialEmissive(mrpt::img::TColorf(0.1f, 0.2f, 0.3f));
  EXPECT_FLOAT_EQ(box->materialEmissive().R, 0.1f);

  box->setScale(2.0f);
  EXPECT_FLOAT_EQ(box->getScaleX(), 2.0f);
  EXPECT_FLOAT_EQ(box->getScaleY(), 2.0f);
  EXPECT_FLOAT_EQ(box->getScaleZ(), 2.0f);

  box->setScale(1.f, 2.f, 3.f);
  EXPECT_FLOAT_EQ(box->getScaleX(), 1.f);
  EXPECT_FLOAT_EQ(box->getScaleY(), 2.f);
  EXPECT_FLOAT_EQ(box->getScaleZ(), 3.f);
}

TEST(CVisualObject, ColorVariants)
{
  auto box = CBox::Create();
  // setColor()/getColor() round-trip through an 8-bit representation
  // internally, so exact float equality is not guaranteed.
  box->setColor(mrpt::img::TColorf(0.1f, 0.2f, 0.3f, 0.4f));
  EXPECT_NEAR(box->getColor().R, 0.1f, 0.01f);

  box->setColor(0.5f, 0.6f, 0.7f, 0.8f);
  EXPECT_NEAR(box->getColor().G, 0.6f, 0.01f);

  box->setColor_u8(mrpt::img::TColor(10, 20, 30, 255));
  EXPECT_EQ(box->getColor_u8().R, 10);

  box->setColor_u8(1, 2, 3, 4);
  EXPECT_EQ(box->getColor_u8().B, 3);

  box->setColorA(0.5f);
  box->setColorA_u8(128);
}

TEST(CVisualObject, LightAndCullFace)
{
  auto box = CBox::Create();
  EXPECT_TRUE(box->isLightEnabled());
  box->enableLight(false);
  EXPECT_FALSE(box->isLightEnabled());

  EXPECT_EQ(box->cullFaces(), TCullFace::NONE);
  box->cullFaces(TCullFace::BACK);
  EXPECT_EQ(box->cullFaces(), TCullFace::BACK);
}

TEST(CVisualObject, ChangeNotificationAndBoundingBox)
{
  auto box = CBox::Create();
  box->clearChangedFlag();
  box->notifyChange();
  box->notifyBBoxChange();
  EXPECT_TRUE(box->hasToUpdateBuffers());

  const auto bb = box->getBoundingBox();
  EXPECT_LE(bb.min.x, bb.max.x);
}

TEST(VisualObjectParams_Points, PointCloudMixin)
{
  auto pc = CPointCloud::Create();
  pc->setPointSize(5.0f);
  EXPECT_FLOAT_EQ(pc->getPointSize(), 5.0f);

  EXPECT_TRUE(pc->isEnabledVariablePointSize());  // default=true
  pc->enableVariablePointSize(false);
  EXPECT_FALSE(pc->isEnabledVariablePointSize());

  pc->setVariablePointSize_k(0.1f);
  EXPECT_FLOAT_EQ(pc->getVariablePointSize_k(), 0.1f);

  pc->setVariablePointSize_DepthScale(2.0f);
  EXPECT_FLOAT_EQ(pc->getVariablePointSize_DepthScale(), 2.0f);
}

TEST(VisualObjectParams_Lines, SetOfLinesMixin)
{
  auto lines = CSetOfLines::Create();
  lines->setLineWidth(3.0f);
  EXPECT_FLOAT_EQ(lines->getLineWidth(), 3.0f);

  EXPECT_FALSE(lines->isAntiAliasingEnabled());
  lines->enableAntiAliasing(true);
  EXPECT_TRUE(lines->isAntiAliasingEnabled());
}

TEST(VisualObjectParams_Triangles, SetOfTrianglesMixin)
{
  auto tris = CSetOfTriangles::Create();
  EXPECT_TRUE(tris->isLightEnabled());
  tris->enableLight(false);
  EXPECT_FALSE(tris->isLightEnabled());

  tris->cullFaces(TCullFace::FRONT);
  EXPECT_EQ(tris->cullFaces(), TCullFace::FRONT);
}

TEST(VisualObjectParams_TexturedTriangles, TexturedPlaneMixin)
{
  auto plane = CTexturedPlane::Create();
  mrpt::img::CImage img(4, 4, mrpt::img::CH_RGB);
  plane->assignImage(img);
  EXPECT_TRUE(plane->textureImageHasBeenAssigned());
  EXPECT_EQ(plane->getTextureImage().getWidth(), 4u);

  EXPECT_FALSE(plane->textureLinearInterpolation());
  plane->enableTextureLinearInterpolation(true);
  EXPECT_TRUE(plane->textureLinearInterpolation());

  EXPECT_TRUE(plane->textureMipMap());  // default=true
  plane->enableTextureMipMap(false);
  EXPECT_FALSE(plane->textureMipMap());

  mrpt::img::CImage normalMap(4, 4, mrpt::img::CH_RGB);
  plane->assignNormalMap(normalMap);
  EXPECT_TRUE(plane->normalMapHasBeenAssigned());
}
