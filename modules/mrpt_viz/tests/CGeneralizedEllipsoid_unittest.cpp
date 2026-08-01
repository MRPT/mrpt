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
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>

using namespace mrpt::viz;

namespace
{
template <typename EllipsoidPtr, int DIM>
void checkCommonEllipsoidApi(const EllipsoidPtr& obj)
{
  ASSERT_TRUE(obj);
  mrpt::math::CMatrixFixed<double, DIM, DIM> cov;
  cov.setIdentity();
  for (int i = 0; i < DIM; i++)
  {
    cov(i, i) = 0.1 * (i + 1);
  }
  obj->setCovMatrix(cov);
  const auto covOut = obj->getCovMatrix();
  EXPECT_NEAR(covOut(0, 0), 0.1, 1e-9);

  obj->setQuantiles(2.5f);
  EXPECT_FLOAT_EQ(obj->getQuantiles(), 2.5f);

  obj->setNumberOfSegments(30);
  EXPECT_EQ(obj->getNumberOfSegments(), 30u);

  obj->enableDrawSolid3D(true);
  obj->setColor(0.3f, 0.5f, 0.7f);
  obj->setLocation(1, 2, 3);

  EXPECT_NO_THROW(obj->updateBuffers());
}
}  // namespace

TEST(CEllipsoid2D, BasicApi)
{
  auto obj = CEllipsoid2D::Create();
  checkCommonEllipsoidApi<CEllipsoid2D::Ptr, 2>(obj);
}

TEST(CEllipsoid3D, BasicApi)
{
  auto obj = CEllipsoid3D::Create();
  checkCommonEllipsoidApi<CEllipsoid3D::Ptr, 3>(obj);
}

TEST(CEllipsoidInverseDepth2D, BasicApi)
{
  auto obj = CEllipsoidInverseDepth2D::Create();
  checkCommonEllipsoidApi<CEllipsoidInverseDepth2D::Ptr, 2>(obj);
}

TEST(CEllipsoidInverseDepth3D, BasicApi)
{
  auto obj = CEllipsoidInverseDepth3D::Create();
  checkCommonEllipsoidApi<CEllipsoidInverseDepth3D::Ptr, 3>(obj);
}

TEST(CEllipsoidRangeBearing2D, BasicApi)
{
  auto obj = CEllipsoidRangeBearing2D::Create();
  checkCommonEllipsoidApi<CEllipsoidRangeBearing2D::Ptr, 2>(obj);
}

TEST(CEllipsoid3D, SetCovMatrixAndMean)
{
  auto obj = CEllipsoid3D::Create();
  mrpt::math::CMatrixFixed<double, 3, 3> cov;
  cov.setIdentity();
  mrpt::math::CMatrixFixed<double, 3, 1> mean;
  mean(0, 0) = 1.0;
  mean(1, 0) = 2.0;
  mean(2, 0) = 3.0;
  obj->setCovMatrixAndMean(cov, mean);
  EXPECT_NEAR(obj->getCovMatrix()(0, 0), 1.0, 1e-9);
}
