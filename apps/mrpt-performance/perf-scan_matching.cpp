/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#include <mrpt/core/cpu.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <mrpt/tfest.h>

#include "common.h"

using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::tfest;
using namespace std;

using TPoints = std::vector<std::vector<double>>;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
void generate_points(TPoints& pA, TPoints& pB)
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

  CPose3DQuat qPose(CPose3D(Dx, Dy, Dz, yaw, pitch, roll));
  for (unsigned int i = 0; i < 5; ++i)
  {
    pB[i].resize(3);
    qPose.inverseComposePoint(pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
  }

}  // end generate_points

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
void generate_list_of_points(const TPoints& pA, const TPoints& pB, TMatchingPairList& list)
{
  TMatchingPair pair;
  for (unsigned int i = 0; i < 5; ++i)
  {
    pair.globalIdx = pair.localIdx = i;
    pair.global.x = pA[i][0];
    pair.global.y = pA[i][1];
    pair.global.z = pA[i][2];

    pair.local.x = pB[i][0];
    pair.local.y = pB[i][1];
    pair.local.z = pB[i][2];

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

// ------------------------------------------------------
//				Benchmark: using CPose3DQuat
// ------------------------------------------------------
double tfest_test_1(int a1, int a2)
{
  TPoints pA, pB;
  generate_points(pA, pB);

  TMatchingPairList list;
  generate_list_of_points(pA, pB, list);

  CPose3DQuat out;
  double scale;

  const size_t N = a1;
  CTicTac tictac;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) mrpt::tfest::se3_l2(list, out, scale);

  const double T = tictac.Tac() / N;
  return T;
}

// ------------------------------------------------------
//				Benchmark: using vectors
// ------------------------------------------------------
double tfest_test_3(int a1, int a2)
{
  TPoints pA, pB;
  generate_points(pA, pB);

  vector<mrpt::math::TPoint3D> ptsA, ptsB;
  generate_vector_of_points(pA, pB, ptsA, ptsB);

  const size_t N = a1;
  CTicTac tictac;

  mrpt::poses::CPose3DQuat out_pose;
  double out_scale;

  tictac.Tic();
  for (size_t i = 0; i < N; i++) mrpt::tfest::se3_l2(ptsA, ptsB, out_pose, out_scale);

  const double T = tictac.Tac() / N;
  return T;
}

// ------------------------------------------------------
//				Benchmark:  leastSquareErrorRigidTransformation
// ------------------------------------------------------
template <bool DISABLE_SIMD = false>
double tfest_test_4(int nCorrs, int nRepets)
{
  const bool savedFeatSSE2 = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  if (DISABLE_SIMD) mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, false);

  TPoints pA, pB;
  generate_points(pA, pB);

  vector<double> qu;

  TMatchingPairList in_correspondences;
  mrpt::math::TPose2D out_pose;

  in_correspondences.resize(nCorrs);
  for (int i = 0; i < nCorrs; i++)
  {
    TMatchingPair& m = in_correspondences[i];
    m.globalIdx = i;
    m.localIdx = i;
    m.global.x = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
    m.global.y = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
    m.global.z = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
    m.local.x = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
    m.local.y = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
    m.local.z = mrpt::random::getRandomGenerator().drawUniform(-10, 10);
  }

  const size_t N = nRepets;
  CTicTac tictac;

  tictac.Tic();
  for (size_t i = 0; i < N; i++)
  {
    mrpt::tfest::se2_l2(in_correspondences, out_pose);
  }

  const double T = tictac.Tac() / N;

  if (DISABLE_SIMD) mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, savedFeatSSE2);

  return T;
}

// ------------------------------------------------------
// register_tests_scan_matching
// ------------------------------------------------------
void register_tests_scan_matching()
{
  lstTests.emplace_back("tfest: se3_l2 [CPose3DQuat]", tfest_test_1, 1e4);
  lstTests.emplace_back("tfest: se3_l2 [vector TPoint3D]", tfest_test_3, 1e4);

  // clang-format off
	lstTests.emplace_back("tfest: se2_l2 [x10 corrs]", tfest_test_4<false>, 10, 1e6);
	lstTests.emplace_back("tfest: se2_l2 [x100 corrs]", tfest_test_4<false>, 100, 1e6);
	lstTests.emplace_back("tfest: se2_l2 [x1000 corrs]", tfest_test_4<false>, 1000, 1e5);
	lstTests.emplace_back("tfest: se2_l2 [x10000 corrs]", tfest_test_4<false>, 10000, 1e4);

	lstTests.emplace_back("tfest: se2_l2 [x10 corrs] [SSE2 disabled]", tfest_test_4<true>, 10, 1e6);
	lstTests.emplace_back("tfest: se2_l2 [x100 corrs] [SSE2 disabled]", tfest_test_4<true>, 100, 1e6);
	lstTests.emplace_back("tfest: se2_l2 [x1000 corrs] [SSE2 disabled]", tfest_test_4<true>, 1000, 1e5);
	lstTests.emplace_back("tfest: se2_l2 [x10000 corrs] [SSE2 disabled]", tfest_test_4<true>, 10000, 1e4);
  // clang-format on
}
