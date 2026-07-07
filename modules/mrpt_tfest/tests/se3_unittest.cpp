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
#include <mrpt/math/ops_vectors.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <mrpt/tfest.h>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

using TPoints = std::vector<std::vector<double>>;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
CPose3DQuat generate_points(TPoints& pA, TPoints& pB)
{
  const double Dx = 0.5;
  const double Dy = 1.5;
  const double Dz = 0.75;

  const double yaw = 10.0_deg;
  const double pitch = 20.0_deg;
  const double roll = 5.0_deg;

  pA.resize(5);  // A set of points at "A" reference system
  pB.resize(5);  // A set of points at "B" reference system

  pA[0].resize(3);
  pA[0][0] = 0.0;
  pA[0][1] = 0.5;
  pA[0][2] = 0.4;
  pA[1].resize(3);
  pA[1][0] = 1.0;
  pA[1][1] = 1.5;
  pA[1][2] = -0.1;
  pA[2].resize(3);
  pA[2][0] = 1.2;
  pA[2][1] = 1.1;
  pA[2][2] = 0.9;
  pA[3].resize(3);
  pA[3][0] = 0.7;
  pA[3][1] = 0.3;
  pA[3][2] = 3.4;
  pA[4].resize(3);
  pA[4][0] = 1.9;
  pA[4][1] = 2.5;
  pA[4][2] = -1.7;

  CPose3DQuat qPose = CPose3DQuat(CPose3D(Dx, Dy, Dz, yaw, pitch, roll));
  for (unsigned int i = 0; i < 5; ++i)
  {
    pB[i].resize(3);
    qPose.inverseComposePoint(pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
  }

  return qPose;

}  // end generate_points

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
template <typename T>
void generate_list_of_points(const TPoints& pA, const TPoints& pB, TMatchingPairListTempl<T>& list)
{
  TMatchingPairTempl<T> pair;
  for (unsigned int i = 0; i < 5; ++i)
  {
    pair.globalIdx = pair.localIdx = i;
    pair.global.x = static_cast<T>(pA[i][0]);
    pair.global.y = static_cast<T>(pA[i][1]);
    pair.global.z = static_cast<T>(pA[i][2]);

    pair.local.x = static_cast<T>(pB[i][0]);
    pair.local.y = static_cast<T>(pB[i][1]);
    pair.local.z = static_cast<T>(pB[i][2]);

    list.push_back(pair);
  }
}  // end generate_list_of_points

// ------------------------------------------------------
//				Genreate a vector of matched points
// ------------------------------------------------------
void generate_vector_of_points(
    const TPoints& pA,
    const TPoints& pB,
    vector<mrpt::math::TPoint3D>& ptsA,
    vector<mrpt::math::TPoint3D>& ptsB)
{
  // The input vector: inV = [pA1x, pA1y, pA1z, pB1x, pB1y, pB1z, ... ]
  ptsA.resize(pA.size());
  ptsB.resize(pA.size());
  for (unsigned int i = 0; i < pA.size(); ++i)
  {
    ptsA[i] = mrpt::math::TPoint3D(pA[i][0], pA[i][1], pA[i][2]);
    ptsB[i] = mrpt::math::TPoint3D(pB[i][0], pB[i][1], pB[i][2]);
  }
}  // end generate_vector_of_points

template <typename T>
void se3_l2_MatchList_test()
{
  TPoints pA, pB;  // The input points
  CPose3DQuat qPose = generate_points(pA, pB);

  TMatchingPairListTempl<T> list;
  generate_list_of_points(pA, pB, list);  // Generate a list of matched points

  CPose3DQuat outQuat;  // Output CPose3DQuat for the LSRigidTransformation
  double scale;         // Output scale value

  bool res = mrpt::tfest::se3_l2(list, outQuat, scale);
  EXPECT_TRUE(res);

  double err = 0.0;
  if ((qPose[3] * outQuat[3] > 0 && qPose[4] * outQuat[4] > 0 && qPose[5] * outQuat[5] > 0 &&
       qPose[6] * outQuat[6] > 0) ||
      (qPose[3] * outQuat[3] < 0 && qPose[4] * outQuat[4] < 0 && qPose[5] * outQuat[5] < 0 &&
       qPose[6] * outQuat[6] < 0))
  {
    for (unsigned int i = 0; i < 7; ++i)
    {
      err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
    }
    err = sqrt(err);
    EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
                            << qPose << endl
                            << "Out CPose3DQuat: " << endl
                            << outQuat << " [Err: " << err << "]"
                            << "\n";
  }
  else
  {
    GTEST_FAIL() << "Applied quaternion: " << endl
                 << qPose << endl
                 << "Out CPose3DQuat: " << endl
                 << outQuat << "\n";
  }
}

TEST(tfest, se3_l2_MatchList_float) { se3_l2_MatchList_test<float>(); }
TEST(tfest, se3_l2_MatchList_double) { se3_l2_MatchList_test<double>(); }

TEST(tfest, se3_l2_PtsLists)
{
  TPoints pA, pB;  // The input points
  CPose3DQuat qPose = generate_points(pA, pB);

  vector<mrpt::math::TPoint3D> ptsA, ptsB;
  generate_vector_of_points(pA, pB, ptsA,
                            ptsB);  // Generate a vector of matched points

  mrpt::poses::CPose3DQuat qu;
  double scale;
  mrpt::tfest::se3_l2(ptsA, ptsB, qu,
                      scale);  // Output quaternion for the Horn Method

  double err = 0.0;
  if ((qPose[3] * qu[3] > 0 && qPose[4] * qu[4] > 0 && qPose[5] * qu[5] > 0 &&
       qPose[6] * qu[6] > 0) ||
      (qPose[3] * qu[3] < 0 && qPose[4] * qu[4] < 0 && qPose[5] * qu[5] < 0 &&
       qPose[6] * qu[6] < 0))
  {
    for (unsigned int i = 0; i < 7; ++i)
    {
      err += square(std::fabs(qPose[i]) - std::fabs(qu[i]));
    }
    err = sqrt(err);
    EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
                            << qPose << endl
                            << "Out CPose3DQuat: " << endl
                            << qu << " [Err: " << err << "]"
                            << "\n";
  }
  else
  {
    GTEST_FAIL() << "Applied quaternion: " << endl
                 << qPose << endl
                 << "Out CPose3DQuat: " << endl
                 << qu << "\n";
  }
}  // end

TEST(tfest, se3_l2_robust)
{
  TPoints pA, pB;  // The input points
  CPose3DQuat qPose = generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);  // Generate a list of matched points

  mrpt::tfest::TSE3RobustResult estimateResult;
  mrpt::tfest::TSE3RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSizePct = 3.0 / static_cast<double>(list.size());
  mrpt::tfest::se3_l2_robust(list, params, estimateResult);

  EXPECT_GT(estimateResult.inliers_idx.size(), 0u);
  const CPose3DQuat& outQuat = estimateResult.transformation;
  double err = 0.0;
  if ((qPose[3] * outQuat[3] > 0 && qPose[4] * outQuat[4] > 0 && qPose[5] * outQuat[5] > 0 &&
       qPose[6] * outQuat[6] > 0) ||
      (qPose[3] * outQuat[3] < 0 && qPose[4] * outQuat[4] < 0 && qPose[5] * outQuat[5] < 0 &&
       qPose[6] * outQuat[6] < 0))
  {
    for (unsigned int i = 0; i < 7; ++i)
    {
      err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
    }
    err = sqrt(err);
    EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
                            << qPose << endl
                            << "Out CPose3DQuat: " << endl
                            << outQuat << " [Err: " << err << "]"
                            << "\n";
  }
  else
  {
    GTEST_FAIL() << "Applied quaternion: " << endl
                 << qPose << endl
                 << "Out CPose3DQuat: " << endl
                 << outQuat << "\n";
  }
}

TEST(tfest, se3_l2_TooFewPointsFails)
{
  TPoints pA, pB;
  generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);
  // Keep just 2 correspondences, below the minimum of 3 required:
  while (list.size() > 2)
  {
    list.erase(list.begin());
  }

  CPose3DQuat outQuat;
  double scale;
  EXPECT_FALSE(mrpt::tfest::se3_l2(list, outQuat, scale));
}

TEST(tfest, se3_l2_ForceScaleToUnity)
{
  TPoints pA, pB;
  CPose3DQuat qPose = generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);

  CPose3DQuat outQuat;
  double scale;
  ASSERT_TRUE(mrpt::tfest::se3_l2(list, outQuat, scale, /*forceScaleToUnity=*/true));
  EXPECT_NEAR(scale, 1.0, 1e-9);

  double err = 0.0;
  for (unsigned int i = 0; i < 3; ++i)
  {
    err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
  }
  EXPECT_TRUE(std::sqrt(err) < 0.5) << "Scale-forced translation differs too much from ground "
                                       "truth (rigid input, so this should hold).";
}

TEST(tfest, se3_l2_DoublePrecisionList)
{
  TPoints pA, pB;
  CPose3DQuat qPose = generate_points(pA, pB);

  TMatchingPairList_d list;
  generate_list_of_points(pA, pB, list);

  CPose3DQuat outQuat;
  double scale;
  ASSERT_TRUE(mrpt::tfest::se3_l2(list, outQuat, scale));

  double err = 0.0;
  for (unsigned int i = 0; i < 7; ++i)
  {
    err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
  }
  EXPECT_TRUE(std::sqrt(err) < 1e-6);
}

TEST(tfest, se3_l2_robust_UserCallbackFiltersSubset)
{
  TPoints pA, pB;
  generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);

  mrpt::tfest::TSE3RobustResult estimateResult;
  mrpt::tfest::TSE3RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSizePct = 3.0 / static_cast<double>(list.size());
  // Reject the correspondence with globalIdx==4, forcing the RANSAC loop to
  // skip over it whenever picked:
  params.user_individual_compat_callback = [](const TPotentialMatch& pm) -> bool
  { return pm.idx_this != 4; };

  ASSERT_TRUE(mrpt::tfest::se3_l2_robust(list, params, estimateResult));
  for (const auto idx : estimateResult.inliers_idx)
  {
    EXPECT_NE(idx, 4u);
  }
}

TEST(tfest, se3_l2_robust_VerboseNoSolutionFound)
{
  TPoints pA, pB;
  generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);  // 5 correspondences

  mrpt::tfest::TSE3RobustResult estimateResult;
  mrpt::tfest::TSE3RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSizePct = 1.0;
  params.ransac_nmaxSimulations = 5;
  params.verbose = true;
  // Reject every single correspondence => the min-set-size can never be
  // reached, forcing "No solution found" (max_size==0) return path:
  params.user_individual_compat_callback = [](const TPotentialMatch&) -> bool { return false; };

  EXPECT_FALSE(mrpt::tfest::se3_l2_robust(list, params, estimateResult));
}

TEST(tfest, se3_l2_robust_DegenerateMinSetSizeBelowThree)
{
  // With ransac_minSetSize < 3, the internal se3_l2() call on the "maybe
  // inliers" subset always fails (Horn's method needs >= 3 points),
  // exercising that failure path inside the RANSAC loop:
  TPoints pA, pB;
  generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);

  mrpt::tfest::TSE3RobustResult estimateResult;
  mrpt::tfest::TSE3RobustParams params;
  params.ransac_minSetSize = 2;
  params.ransac_maxSetSizePct = 1.0;
  params.ransac_nmaxSimulations = 5;

  EXPECT_FALSE(mrpt::tfest::se3_l2_robust(list, params, estimateResult));
}

TEST(tfest, se3_l2_LargeRotationQuaternionSignFlip)
{
  // A rotation close to 180 degrees maximizes the chance of the internal
  // Horn's method quaternion having a negative "w" (q_r) component before
  // the sign-normalization step.
  TPoints pA(5), pB(5);
  pA[0] = {0.0, 0.5, 0.4};
  pA[1] = {1.0, 1.5, -0.1};
  pA[2] = {1.2, 1.1, 0.9};
  pA[3] = {0.7, 0.3, 3.4};
  pA[4] = {1.9, 2.5, -1.7};

  const CPose3D truePose(0.5, 1.5, 0.75, 170.0_deg, 20.0_deg, 5.0_deg);
  const CPose3DQuat qPose(truePose);
  for (auto& p : pB)
  {
    p.resize(3);
  }
  for (unsigned int i = 0; i < 5; ++i)
  {
    qPose.inverseComposePoint(pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
  }

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);

  CPose3DQuat outQuat;
  double scale;
  ASSERT_TRUE(mrpt::tfest::se3_l2(list, outQuat, scale));
  // Just check we get a finite, sane result; the actual quaternion sign
  // internal branch is not user-observable.
  EXPECT_TRUE(std::isfinite(outQuat[3]));
}
