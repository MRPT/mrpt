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
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include <sstream>

// Check if we have jsoncpp to enable those tests:
#include <mrpt/serialization/config.h>

using namespace mrpt::serialization;

#if MRPT_HAS_JSONCPP
TEST(SchemaSerialization, JSON_archive)
{
  auto arch = archiveJSON();
  mrpt::poses::CPose2D pt1{1.0, 2.0, 3.0}, pt2;
  arch = pt1;
  std::stringstream ss;
  ss << arch;
  auto pos = ss.str().find("\"datatype\" : \"mrpt::poses::CPose2D\"");
  EXPECT_TRUE(pos != std::string::npos);

  // test deserializing:
  ss.seekg(0);  // rewind for reading
  auto arch2 = archiveJSON();
  // Load the plain text representation into the archive:
  ss >> arch2;
  // Parse the JSON data into an MRPT object:
  arch2.readTo(pt2);

  EXPECT_NEAR(pt1.x(), pt2.x(), 1e-6);
  EXPECT_NEAR(pt1.y(), pt2.y(), 1e-6);
  EXPECT_NEAR(pt1.phi(), pt2.phi(), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPoint2D)
{
  auto arch = archiveJSON();
  mrpt::poses::CPoint2D pt1(1.0, 2.0);
  mrpt::poses::CPoint2D pt2;
  arch = pt1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(pt2);

  EXPECT_NEAR(pt1.x(), pt2.x(), 1e-6);
  EXPECT_NEAR(pt1.y(), pt2.y(), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPoint3D)
{
  auto arch = archiveJSON();
  mrpt::poses::CPoint3D pt1(1.0, 2.0, 3.0);
  mrpt::poses::CPoint3D pt2;
  arch = pt1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(pt2);

  EXPECT_NEAR(pt1.x(), pt2.x(), 1e-6);
  EXPECT_NEAR(pt1.y(), pt2.y(), 1e-6);
  EXPECT_NEAR(pt1.z(), pt2.z(), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPose3D)
{
  auto arch = archiveJSON();
  mrpt::poses::CPose3D p1(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  mrpt::poses::CPose3D p2;
  arch = p1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(p2);

  EXPECT_NEAR(p1.x(), p2.x(), 1e-6);
  EXPECT_NEAR(p1.yaw(), p2.yaw(), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPosePDFGaussian)
{
  auto arch = archiveJSON();
  mrpt::math::CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  mrpt::poses::CPosePDFGaussian p1(mrpt::poses::CPose2D(1, 2, 0.3), cov);
  mrpt::poses::CPosePDFGaussian p2;
  arch = p1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(p2);

  EXPECT_NEAR(p1.mean.x(), p2.mean.x(), 1e-6);
  EXPECT_NEAR(p1.cov(0, 0), p2.cov(0, 0), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPosePDFGaussianInf)
{
  auto arch = archiveJSON();
  mrpt::math::CMatrixDouble33 covInv;
  covInv.setIdentity();
  covInv *= 10.0;
  mrpt::poses::CPosePDFGaussianInf p1(mrpt::poses::CPose2D(1, 2, 0.3), covInv);
  mrpt::poses::CPosePDFGaussianInf p2;
  arch = p1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(p2);

  EXPECT_NEAR(p1.mean.x(), p2.mean.x(), 1e-6);
  EXPECT_NEAR(p1.cov_inv(0, 0), p2.cov_inv(0, 0), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPointPDFGaussian)
{
  auto arch = archiveJSON();
  mrpt::math::CMatrixDouble33 cov;
  cov.setIdentity();
  cov *= 0.1;
  mrpt::poses::CPointPDFGaussian p1(mrpt::poses::CPoint3D(1, 2, 3), cov);
  mrpt::poses::CPointPDFGaussian p2;
  arch = p1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(p2);

  EXPECT_NEAR(p1.mean.x(), p2.mean.x(), 1e-6);
}

TEST(SchemaSerialization, JSON_archive_CPoint2DPDFGaussian)
{
  auto arch = archiveJSON();
  mrpt::math::CMatrixDouble22 cov;
  cov.setIdentity();
  cov *= 0.1;
  mrpt::poses::CPoint2DPDFGaussian p1(mrpt::poses::CPoint2D(1, 2), cov);
  mrpt::poses::CPoint2DPDFGaussian p2;
  arch = p1;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = archiveJSON();
  ss >> arch2;
  arch2.readTo(p2);

  EXPECT_NEAR(p1.mean.x(), p2.mean.x(), 1e-6);
}

#endif
