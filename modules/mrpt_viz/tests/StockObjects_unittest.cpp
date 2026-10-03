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
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/stock_objects.h>

#include <cmath>

using namespace mrpt::viz;

TEST(StockObjects, RobotRhodon)
{
  auto obj = stock_objects::RobotRhodon();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, RobotGiraff)
{
  auto obj = stock_objects::RobotGiraff();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, RobotPioneer)
{
  auto obj = stock_objects::RobotPioneer();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, CornerXYZ)
{
  auto obj = stock_objects::CornerXYZ(2.0f);
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, CornerXYZArrowProportions)
{
  // The arrow head takes the same fraction of the length at any scale:
  const float scale = 10.0f;
  auto obj = stock_objects::CornerXYZ(scale);
  ASSERT_TRUE(obj);
  auto arrowX = obj->getByClass<CArrow>(0);
  ASSERT_TRUE(arrowX);
  EXPECT_NO_THROW(arrowX->updateBuffers());

  const float headBaseRadius = 0.05f * scale;
  size_t numHeadBaseVertices = 0;
  for (const auto& t : arrowX->shaderTrianglesBuffer())
  {
    for (const auto& v : t.vertices)
    {
      const auto& p = v.xyzrgba.pt;
      if (std::abs(std::hypot(p.y, p.z) - headBaseRadius) < 1e-4f)
      {
        EXPECT_NEAR(p.x, 0.75f * scale, 1e-4f);
        numHeadBaseVertices++;
      }
    }
  }
  EXPECT_GT(numHeadBaseVertices, 0u);
}
TEST(StockObjects, CornerXYZEye)
{
  auto obj = stock_objects::CornerXYZEye();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, CornerXYZSimple)
{
  auto obj = stock_objects::CornerXYZSimple(1.5f, 2.0f);
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, CornerXYSimple)
{
  auto obj = stock_objects::CornerXYSimple(1.5f, 2.0f);
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, BumblebeeCamera)
{
  auto obj = stock_objects::BumblebeeCamera();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, Hokuyo_URG)
{
  auto obj = stock_objects::Hokuyo_URG();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, Hokuyo_UTM)
{
  auto obj = stock_objects::Hokuyo_UTM();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
TEST(StockObjects, Househam_Sprayer)
{
  auto obj = stock_objects::Househam_Sprayer();
  ASSERT_TRUE(obj);
  EXPECT_GT(obj->size(), 0u);
}
