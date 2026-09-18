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

/** The mrpt::viz classes that also implement the "scheme" (JSON) archive
 *  interface, which is a separate code path from the binary CArchive one.
 */

#include <gtest/gtest.h>
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/config.h>  // MRPT_HAS_JSONCPP
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CPointCloud.h>

#include <sstream>

#if MRPT_HAS_JSONCPP

using namespace mrpt::viz;

namespace
{
/** Serializes `in` to JSON and reads it back into `out`. */
template <class T>
void jsonRoundTrip(const T& in, T& out)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch = in;

  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  arch2.readTo(out);
}
}  // namespace

TEST(SchemeSerialization, CArrow)
{
  CArrow a(mrpt::math::TPoint3Df(1, 2, 3), mrpt::math::TPoint3Df(4, 5, 6), 0.3f, 0.07f, 0.25f);
  a.setSlicesCount(11);

  CArrow b;
  jsonRoundTrip(a, b);

  EXPECT_NEAR(b.getBoundingBoxLocal().min.x, a.getBoundingBoxLocal().min.x, 1e-4);
  EXPECT_NEAR(b.getBoundingBoxLocal().max.z, a.getBoundingBoxLocal().max.z, 1e-4);
}

TEST(SchemeSerialization, CCylinder)
{
  CCylinder a(1.5f, 0.5f, 3.0f, 24);
  a.setHasBases(false, true);

  CCylinder b;
  jsonRoundTrip(a, b);

  EXPECT_FLOAT_EQ(b.getBottomRadius(), 1.5f);
  EXPECT_FLOAT_EQ(b.getTopRadius(), 0.5f);
  EXPECT_FLOAT_EQ(b.getHeight(), 3.0f);
  EXPECT_EQ(b.getSlicesCount(), 24U);
  EXPECT_FALSE(b.hasTopBase());
  EXPECT_TRUE(b.hasBottomBase());
}

TEST(SchemeSerialization, CPointCloud)
{
  CPointCloud a;
  a.insertPoint(1.0f, 2.0f, 3.0f);
  a.insertPoint(-1.0f, -2.0f, -3.0f);
  a.setPointSize(6.0f);
  a.setGradientColors(mrpt::img::TColorf(0, 0, 1), mrpt::img::TColorf(1, 0, 0));

  CPointCloud b;
  jsonRoundTrip(a, b);

  ASSERT_EQ(b.size(), 2U);
  EXPECT_FLOAT_EQ(b.getPoint3Df(0).x, 1.0f);
  EXPECT_FLOAT_EQ(b.getPoint3Df(1).z, -3.0f);
  EXPECT_FLOAT_EQ(b.getPointSize(), 6.0f);
}

TEST(SchemeSerialization, UnknownDatatypeVersionThrows)
{
  auto arch = mrpt::serialization::archiveJSON();
  CCylinder a;
  arch = a;

  std::stringstream ss;
  ss << arch;
  std::string txt = ss.str();

  // Bump the stored "version" past anything serializeFrom() knows:
  const auto pos = txt.find("\"version\"");
  ASSERT_NE(pos, std::string::npos);
  const auto colon = txt.find(':', pos);
  ASSERT_NE(colon, std::string::npos);
  const auto digit = txt.find_first_of("0123456789", colon);
  ASSERT_NE(digit, std::string::npos);
  txt[digit] = '9';

  std::stringstream ss2(txt);
  auto arch2 = mrpt::serialization::archiveJSON();
  ss2 >> arch2;

  CCylinder b;
  EXPECT_THROW(arch2.readTo(b), std::exception);
}

#endif  // MRPT_HAS_JSONCPP
