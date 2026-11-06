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

/** Math checks for the node registration deciders: the uncertainty of the
 *  odometry edge accumulated since the last node, and the 3D angular
 *  registration threshold.
 */

#include <gtest/gtest.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/NRD/CFixedIntervalsNRD.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

using graph_t = mrpt::graphs::CNetworkOfPoses2DInf;
using decider_t = mrpt::graphslam::deciders::CFixedIntervalsNRD<graph_t>;
using mrpt::poses::CPose2D;
using mrpt::poses::CPose3D;

namespace
{
class TestNRD : public decider_t
{
 public:
  using decider_t::checkRegistrationCondition;
};

mrpt::obs::CActionCollection::Ptr makeOdometryAction(const CPose2D& increment)
{
  mrpt::obs::CActionRobotMovement2D act;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  opts.gaussianModel.a1 = 0.01;
  opts.gaussianModel.a2 = 0.001;
  opts.gaussianModel.a3 = 0.01;
  opts.gaussianModel.a4 = 0.01;
  opts.gaussianModel.minStdXY = 0.005;
  opts.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.2);
  act.computeFromOdometry(increment, opts);

  auto acts = mrpt::obs::CActionCollection::Create();
  acts->insert(act);
  return acts;
}

/** Mean and covariance of the increment exactly as the decider reads it */
void incrementPDF(
    const mrpt::obs::CActionCollection::Ptr& acts, Eigen::Vector3d& mean, Eigen::Matrix3d& cov)
{
  mrpt::poses::CPose3DPDFGaussian p3;
  ASSERT_TRUE(acts->getFirstMovementEstimation(p3));
  mrpt::poses::CPosePDFGaussian p2;
  p2.copyFrom(p3);
  mean = Eigen::Vector3d(p2.mean.x(), p2.mean.y(), p2.mean.phi());
  cov = p2.cov.asEigen();
}

void composeFirstOrder(
    Eigen::Vector3d& x,
    Eigen::Matrix3d& cov_x,
    const Eigen::Vector3d& u,
    const Eigen::Matrix3d& cov_u)
{
  const double c = std::cos(x[2]);
  const double s = std::sin(x[2]);
  Eigen::Matrix3d J1 = Eigen::Matrix3d::Identity();
  J1(0, 2) = -s * u[0] - c * u[1];
  J1(1, 2) = c * u[0] - s * u[1];
  Eigen::Matrix3d J2 = Eigen::Matrix3d::Identity();
  J2(0, 0) = c;
  J2(0, 1) = -s;
  J2(1, 0) = s;
  J2(1, 1) = c;
  cov_x = J1 * cov_x * J1.transpose() + J2 * cov_u * J2.transpose();
  x = Eigen::Vector3d(x[0] + c * u[0] - s * u[1], x[1] + s * u[0] + c * u[1], x[2] + u[2]);
}

double geodesicAngle(const CPose3D& a, const CPose3D& b)
{
  const Eigen::Matrix3d R =
      a.getRotationMatrix().asEigen().transpose() * b.getRotationMatrix().asEigen();
  return Eigen::AngleAxisd(R).angle();
}
}  // namespace

TEST(CFixedIntervalsNRD, odometry_edge_covariance_is_the_composition_of_the_increments)
{
  graph_t graph;
  decider_t d;
  d.setMinLoggingLevel(mrpt::system::LVL_ERROR);
  d.setGraphPtr(&graph);
  d.params.registration_max_distance = 0.5;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);

  const std::vector<CPose2D> increments = {
      CPose2D(0.2, 0.0, 0.2), CPose2D(0.2, 0.02, -0.1), CPose2D(0.2, 0.0, 0.1)};
  auto sf = mrpt::obs::CSensoryFrame::Create();
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  bool registered = false;
  for (const auto& inc : increments)
  {
    auto acts = makeOdometryAction(inc);
    Eigen::Vector3d u;
    Eigen::Matrix3d cov_u;
    incrementPDF(acts, u, cov_u);
    composeFirstOrder(x, cov, u, cov_u);
    registered = d.updateState(acts, sf, nullptr);
  }
  ASSERT_TRUE(registered);
  ASSERT_EQ(graph.edgeCount(), 1U);

  const auto& edge = graph.edges.begin()->second;
  EXPECT_NEAR(edge.mean.x(), x[0], 1e-9);
  EXPECT_NEAR(edge.mean.y(), x[1], 1e-9);
  EXPECT_NEAR(edge.mean.phi(), x[2], 1e-9);

  const Eigen::Matrix3d edge_cov = edge.cov_inv.asEigen().inverse();
  EXPECT_LT((edge_cov - cov).norm(), 1e-2 * cov.norm()) << "edge cov:\n"
                                                        << edge_cov << "\nexpected:\n"
                                                        << cov;
}

TEST(CFixedIntervalsNRD, registration_condition_3D_uses_the_rotation_angle)
{
  TestNRD d;
  d.params.registration_max_distance = 10.0;
  d.params.registration_max_angle = mrpt::DEG2RAD(15.0);

  // 10 deg on each Euler angle is a 16.8 deg rotation:
  const CPose3D o;
  const CPose3D p(0, 0, 0, mrpt::DEG2RAD(10.0), mrpt::DEG2RAD(10.0), mrpt::DEG2RAD(10.0));
  ASSERT_GT(geodesicAngle(o, p), mrpt::DEG2RAD(15.0));
  EXPECT_TRUE(d.checkRegistrationCondition(o, p));

  // A 2 deg rotation at 89 deg of pitch changes yaw and roll by ~63 deg:
  const CPose3D p1(0, 0, 0, 0, mrpt::DEG2RAD(89.0), 0);
  const CPose3D p2 = p1 + CPose3D(0, 0, 0, mrpt::DEG2RAD(2.0), 0, 0);
  ASSERT_NEAR(geodesicAngle(p1, p2), mrpt::DEG2RAD(2.0), 1e-9);
  EXPECT_FALSE(d.checkRegistrationCondition(p1, p2));
}

TEST(CFixedIntervalsNRD, registration_condition_3D_translation_is_euclidean)
{
  TestNRD d;
  d.params.registration_max_distance = 0.5;
  d.params.registration_max_angle = mrpt::DEG2RAD(60.0);
  const CPose3D o(1.0, 2.0, 3.0, 0.3, 0.1, -0.2);
  // 0.3^2 + 0.3^2 + 0.3^2 = 0.27 -> 0.52 m:
  EXPECT_TRUE(d.checkRegistrationCondition(o, CPose3D(1.3, 2.3, 3.3, 0.3, 0.1, -0.2)));
  // 0.28 -> 0.485 m:
  EXPECT_FALSE(d.checkRegistrationCondition(o, CPose3D(1.28, 2.28, 3.28, 0.3, 0.1, -0.2)));
}
