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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/tfest/registerAllClasses.h>

#include <fstream>
#include <sstream>

using namespace mrpt::tfest;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
template <typename T>
TMatchingPairListTempl<T> buildIdentityList()
{
  TMatchingPairListTempl<T> list;
  list.push_back(TMatchingPairTempl<T>(0, 0, 1, 2, 3, 1, 2, 3));
  list.push_back(TMatchingPairTempl<T>(1, 1, 4, 5, 6, 4, 5, 6));
  list.push_back(TMatchingPairTempl<T>(2, 2, -1, 0, 2, -1, 0, 2));
  return list;
}
}  // namespace

TEST(TMatchingPair, RegisterAllClasses) { mrpt::tfest::registerAllClasses_mrpt_tfest(); }

TEST(TMatchingPair, ConstructionAndAccessors)
{
  TMatchingPair p(0, 1, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  EXPECT_EQ(p.globalIdx, 0u);
  EXPECT_EQ(p.localIdx, 1u);
  EXPECT_FLOAT_EQ(p.global.x, 1.0f);
  EXPECT_FLOAT_EQ(p.local.z, 6.0f);

  TMatchingPair p2(0, 1, TPoint3Df(1, 2, 3), TPoint3Df(4, 5, 6));
  EXPECT_TRUE(p == p2);
}

TEST(TMatchingPair, GetClassName)
{
  const std::string name = TMatchingPair::getClassName();
  EXPECT_NE(name.find("TMatchingPairTempl"), std::string::npos);
}

TEST(TMatchingPair, PrintAndStreamOperator)
{
  TMatchingPair p(3, 7, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  std::ostringstream ss;
  p.print(ss);
  EXPECT_NE(ss.str().find("3->7"), std::string::npos);

  std::ostringstream ss2;
  ss2 << p;
  EXPECT_EQ(ss.str(), ss2.str());
}

TEST(TMatchingPair, ComparisonOperators)
{
  TMatchingPair a(0, 0, 0, 0, 0, 0, 0, 0);
  TMatchingPair b(0, 1, 0, 0, 0, 0, 0, 0);
  TMatchingPair c(1, 0, 0, 0, 0, 0, 0, 0);

  EXPECT_TRUE(a < b);
  EXPECT_TRUE(a < c);
  EXPECT_FALSE(b < a);
  EXPECT_TRUE(a == a);
  EXPECT_FALSE(a == b);
}

TEST(TMatchingPair, ListEqualityOperator)
{
  auto listA = buildIdentityList<float>();
  auto listB = buildIdentityList<float>();
  EXPECT_TRUE(listA == listB);

  listB.pop_back();
  EXPECT_FALSE(listA == listB);

  auto listC = buildIdentityList<float>();
  listC[0].localIdx = 99;
  EXPECT_FALSE(listA == listC);
}

TEST(TMatchingPair, IndexOtherMapHasCorrespondence)
{
  auto list = buildIdentityList<float>();
  EXPECT_TRUE(list.indexOtherMapHasCorrespondence(1));
  EXPECT_FALSE(list.indexOtherMapHasCorrespondence(99));
}

TEST(TMatchingPair, Contains)
{
  auto list = buildIdentityList<float>();
  EXPECT_TRUE(list.contains(list[0]));

  TMatchingPair notInList(50, 50, 0, 0, 0, 0, 0, 0);
  EXPECT_FALSE(list.contains(notInList));
}

TEST(TMatchingPair, DumpToFile)
{
  const std::string fname = mrpt::system::getTempFileName() + "_TMatchingPair_dump.txt";
  auto list = buildIdentityList<float>();
  list.dumpToFile(fname);

  std::ifstream f(fname);
  ASSERT_TRUE(f.is_open());
  std::string line;
  size_t nLines = 0;
  while (std::getline(f, line))
  {
    nLines++;
  }
  EXPECT_EQ(nLines, list.size());
  f.close();
  std::remove(fname.c_str());
}

TEST(TMatchingPair, SaveAsMATLABScript)
{
  const std::string fname = mrpt::system::getTempFileName() + "_TMatchingPair.m";
  auto list = buildIdentityList<float>();
  list.saveAsMATLABScript(fname);

  std::ifstream f(fname);
  ASSERT_TRUE(f.is_open());
  std::stringstream buf;
  buf << f.rdbuf();
  const std::string contents = buf.str();
  EXPECT_NE(contents.find("axis equal"), std::string::npos);
  EXPECT_NE(contents.find("view(3)"), std::string::npos);
  f.close();
  std::remove(fname.c_str());
}

TEST(TMatchingPair, SaveAsMATLABScriptBadPath)
{
  auto list = buildIdentityList<float>();
  // Should not throw even if the path cannot be created:
  list.saveAsMATLABScript("/this/path/does/not/exist/file.m");
}

TEST(TMatchingPair, OverallSquareError2D)
{
  TMatchingPairListTempl<float> list;
  // Identity transform: global == local, so error must be zero.
  list.push_back(TMatchingPairTempl<float>(0, 0, 1, 2, 0, 1, 2, 0));
  list.push_back(TMatchingPairTempl<float>(1, 1, 3, 4, 0, 3, 4, 0));

  const CPose2D identity(0, 0, 0);
  const float err = list.overallSquareError(identity);
  EXPECT_NEAR(err, 0.0f, 1e-5f);

  std::vector<float> errs;
  list.squareErrorVector(identity, errs);
  ASSERT_EQ(errs.size(), 2u);
  EXPECT_NEAR(errs[0], 0.0f, 1e-5f);
  EXPECT_NEAR(errs[1], 0.0f, 1e-5f);

  // Non-identity transform must give a non-zero error:
  const CPose2D shifted(1.0, 0, 0);
  const float err2 = list.overallSquareError(shifted);
  EXPECT_GT(err2, 0.0f);
}

TEST(TMatchingPair, OverallSquareErrorAndPoints)
{
  TMatchingPairListTempl<float> list;
  list.push_back(TMatchingPairTempl<float>(0, 0, 1, 2, 0, 1, 2, 0));
  list.push_back(TMatchingPairTempl<float>(1, 1, 3, 4, 0, 3, 4, 0));

  const CPose2D identity(0, 0, 0);
  std::vector<float> xs, ys;
  const float err = list.overallSquareErrorAndPoints(identity, xs, ys);
  EXPECT_NEAR(err, 0.0f, 1e-5f);
  ASSERT_EQ(xs.size(), 2u);
  EXPECT_NEAR(xs[0], 1.0f, 1e-5f);
  EXPECT_NEAR(ys[0], 2.0f, 1e-5f);
  EXPECT_NEAR(xs[1], 3.0f, 1e-5f);
  EXPECT_NEAR(ys[1], 4.0f, 1e-5f);
}

TEST(TMatchingPair, OverallSquareError3D)
{
  TMatchingPairListTempl<float> list;
  list.push_back(TMatchingPairTempl<float>(0, 0, 1, 2, 3, 1, 2, 3));
  list.push_back(TMatchingPairTempl<float>(1, 1, -1, 4, 2, -1, 4, 2));

  const CPose3D identity;
  const float err = list.overallSquareError(identity);
  EXPECT_NEAR(err, 0.0f, 1e-4f);

  std::vector<float> errs;
  list.squareErrorVector(identity, errs);
  ASSERT_EQ(errs.size(), 2u);
  EXPECT_NEAR(errs[0], 0.0f, 1e-4f);

  const CPose3D shifted(1.0, 0, 0, 0, 0, 0);
  const float err2 = list.overallSquareError(shifted);
  EXPECT_GT(err2, 0.0f);
}

TEST(TMatchingPair, FilterUniqueRobustPairs)
{
  TMatchingPairListTempl<float> list;

  // Two candidate matches for globalIdx=0, keep the one with lower error:
  TMatchingPairTempl<float> p0(0, 0, 0, 0, 0, 0, 0, 0);
  p0.errorSquareAfterTransformation = 1.0f;
  TMatchingPairTempl<float> p1(0, 1, 0, 0, 0, 0, 0, 0);
  p1.errorSquareAfterTransformation = 0.1f;
  TMatchingPairTempl<float> p2(1, 2, 0, 0, 0, 0, 0, 0);
  p2.errorSquareAfterTransformation = 0.5f;

  list.push_back(p0);
  list.push_back(p1);
  list.push_back(p2);

  TMatchingPairListTempl<float> filtered;
  list.filterUniqueRobustPairs(2, filtered);

  ASSERT_EQ(filtered.size(), 2u);
  bool foundBestForZero = false;
  for (const auto& c : filtered)
  {
    if (c.globalIdx == 0)
    {
      EXPECT_EQ(c.localIdx, 1u);
      foundBestForZero = true;
    }
  }
  EXPECT_TRUE(foundBestForZero);
}

TEST(TMatchingPair, DoublePrecisionInstantiation)
{
  auto list = buildIdentityList<double>();
  EXPECT_EQ(list.size(), 3u);
  const CPose2D identity(0, 0, 0);
  const double err = list.overallSquareError(identity);
  EXPECT_GE(err, 0.0);

  TMatchingPairListTempl<double> filtered;
  list.filterUniqueRobustPairs(3, filtered);
  EXPECT_EQ(filtered.size(), list.size());
}
