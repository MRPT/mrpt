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
#include <mrpt/kinematics/CKinematicChain.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <vector>

using namespace mrpt::kinematics;

namespace
{
/** A 2-link planar arm: both links revolute, of length `a`, in the XY plane. */
CKinematicChain make_planar_arm(double a = 1.0)
{
  CKinematicChain c;
  c.addLink(.0 /*theta*/, .0 /*d*/, a /*a*/, .0 /*alpha*/, false /*revolute*/);
  c.addLink(.0, .0, a, .0, false);
  return c;
}
}  // namespace

TEST(CKinematicChain, default_constructed_is_empty)
{
  CKinematicChain c;
  EXPECT_EQ(c.size(), 0U);
  EXPECT_EQ(c.getOriginPose(), mrpt::poses::CPose3D());

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  EXPECT_EQ(poses.size(), 1U);  // just the base frame
}

TEST(CKinematicChain, add_get_and_remove_links)
{
  CKinematicChain c;
  c.addLink(0.1, 0.2, 0.3, 0.4, false);
  c.addLink(1.1, 1.2, 1.3, 1.4, true);
  ASSERT_EQ(c.size(), 2U);

  const auto& l0 = c.getLink(0);
  EXPECT_NEAR(l0.theta, 0.1, 1e-12);
  EXPECT_NEAR(l0.d, 0.2, 1e-12);
  EXPECT_NEAR(l0.a, 0.3, 1e-12);
  EXPECT_NEAR(l0.alpha, 0.4, 1e-12);
  EXPECT_FALSE(l0.is_prismatic);
  EXPECT_TRUE(c.getLink(1).is_prismatic);

  c.getLinkRef(0).theta = 0.9;
  EXPECT_NEAR(c.getLink(0).theta, 0.9, 1e-12);

  EXPECT_ANY_THROW(c.getLink(2));
  EXPECT_ANY_THROW(c.getLinkRef(2));
  EXPECT_ANY_THROW(c.removeLink(2));

  c.removeLink(0);
  ASSERT_EQ(c.size(), 1U);
  EXPECT_NEAR(c.getLink(0).theta, 1.1, 1e-12);

  c.clear();
  EXPECT_EQ(c.size(), 0U);
}

TEST(CKinematicChain, configuration_get_and_set)
{
  CKinematicChain c;
  c.addLink(0.5, .0, 1.0, .0, false);  // revolute: q is `theta`
  c.addLink(.0, 0.7, 1.0, .0, true);   // prismatic: q is `d`

  std::vector<double> q;
  c.getConfiguration(q);
  ASSERT_EQ(q.size(), 2U);
  EXPECT_NEAR(q[0], 0.5, 1e-12);
  EXPECT_NEAR(q[1], 0.7, 1e-12);

  c.setConfiguration(std::vector<double>{1.5, 2.5});
  EXPECT_NEAR(c.getLink(0).theta, 1.5, 1e-12);
  EXPECT_NEAR(c.getLink(1).d, 2.5, 1e-12);
  // The "unused" DOF of each link must be left untouched:
  EXPECT_NEAR(c.getLink(0).d, .0, 1e-12);
  EXPECT_NEAR(c.getLink(1).theta, .0, 1e-12);

  EXPECT_ANY_THROW(c.setConfiguration(std::vector<double>{1.0}));
}

TEST(CKinematicChain, configuration_works_with_mrpt_vectors)
{
  CKinematicChain c = make_planar_arm();

  mrpt::math::CVectorDouble q;
  c.getConfiguration(q);
  ASSERT_EQ(q.size(), 2);

  q[0] = 0.25;
  q[1] = -0.25;
  c.setConfiguration(q);
  EXPECT_NEAR(c.getLink(0).theta, 0.25, 1e-12);
  EXPECT_NEAR(c.getLink(1).theta, -0.25, 1e-12);
}

TEST(CKinematicChain, forward_kinematics_of_a_stretched_planar_arm)
{
  const double a = 1.0;
  CKinematicChain c = make_planar_arm(a);

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  ASSERT_EQ(poses.size(), 3U);

  EXPECT_NEAR(poses[0].x(), .0, 1e-9);
  EXPECT_NEAR(poses[1].x(), a, 1e-9);
  EXPECT_NEAR(poses[2].x(), 2 * a, 1e-9);
  EXPECT_NEAR(poses[2].y(), .0, 1e-9);
  EXPECT_NEAR(poses[2].z(), .0, 1e-9);
}

TEST(CKinematicChain, forward_kinematics_of_a_folded_planar_arm)
{
  const double a = 1.0;
  CKinematicChain c = make_planar_arm(a);
  c.setConfiguration(std::vector<double>{.0, M_PI * 0.5});

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  ASSERT_EQ(poses.size(), 3U);

  // 2nd link turns +90deg, so the tip ends at (1,1,0):
  EXPECT_NEAR(poses[2].x(), a, 1e-9);
  EXPECT_NEAR(poses[2].y(), a, 1e-9);
  EXPECT_NEAR(poses[2].z(), .0, 1e-9);
}

TEST(CKinematicChain, prismatic_link_translates_along_z)
{
  CKinematicChain c;
  c.addLink(.0, 0.5 /*d*/, .0, .0, true);

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  ASSERT_EQ(poses.size(), 2U);
  EXPECT_NEAR(poses[1].z(), 0.5, 1e-9);

  c.setConfiguration(std::vector<double>{1.5});
  c.recomputeAllPoses(poses);
  EXPECT_NEAR(poses[1].z(), 1.5, 1e-9);
}

TEST(CKinematicChain, origin_pose_offsets_the_whole_chain)
{
  CKinematicChain c = make_planar_arm();
  const mrpt::poses::CPose3D org(10, 20, 30, .0, .0, .0);
  c.setOriginPose(org);
  EXPECT_EQ(c.getOriginPose(), org);

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_NEAR(poses[0].x(), 10.0, 1e-9);
  EXPECT_NEAR(poses[2].x(), 12.0, 1e-9);
  EXPECT_NEAR(poses[2].y(), 20.0, 1e-9);
  EXPECT_NEAR(poses[2].z(), 30.0, 1e-9);
}

TEST(CKinematicChain, serialization_roundtrip)
{
  CKinematicChain c = make_planar_arm(0.75);
  c.getLinkRef(1).is_prismatic = true;
  c.setOriginPose(mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << c;
  buf.Seek(0);

  CKinematicChain c2;
  arch >> c2;

  ASSERT_EQ(c2.size(), c.size());
  for (size_t i = 0; i < c.size(); i++)
  {
    EXPECT_NEAR(c2.getLink(i).theta, c.getLink(i).theta, 1e-12);
    EXPECT_NEAR(c2.getLink(i).d, c.getLink(i).d, 1e-12);
    EXPECT_NEAR(c2.getLink(i).a, c.getLink(i).a, 1e-12);
    EXPECT_NEAR(c2.getLink(i).alpha, c.getLink(i).alpha, 1e-12);
    EXPECT_EQ(c2.getLink(i).is_prismatic, c.getLink(i).is_prismatic);
  }
  EXPECT_NEAR(c2.getOriginPose().x(), 1.0, 1e-9);
  EXPECT_NEAR(c2.getOriginPose().yaw(), 0.1, 1e-9);
}

TEST(CKinematicChain, visualization_objects_are_built_and_updated)
{
  CKinematicChain c = make_planar_arm();
  c.addLink(.0, 0.5, .0, .0, true);  // a prismatic link too

  auto gl = mrpt::viz::CSetOfObjects::Create();
  std::vector<mrpt::poses::CPose3D> poses;
  c.getAs3DObject(gl, &poses);

  EXPECT_EQ(poses.size(), c.size() + 1);
  EXPECT_EQ(gl->size(), c.size() + 1);

  // Moving the arm and refreshing must not throw nor change the object count:
  const size_t nObjs = gl->size();
  c.setConfiguration(std::vector<double>{0.3, -0.3, 1.0});
  EXPECT_NO_THROW(c.update3DObject(&poses));
  EXPECT_EQ(gl->size(), nObjs);
  EXPECT_EQ(poses.size(), c.size() + 1);
}

TEST(CKinematicChain, getAs3DObject_requires_a_non_null_object)
{
  CKinematicChain c = make_planar_arm();
  mrpt::viz::CSetOfObjects::Ptr gl;  // null
  EXPECT_ANY_THROW(c.getAs3DObject(gl));
}

TEST(CKinematicChain, update3DObject_without_prior_getAs3DObject_throws)
{
  CKinematicChain c = make_planar_arm();
  EXPECT_ANY_THROW(c.update3DObject());
}

TEST(TKinematicLink, stream_operators_roundtrip)
{
  TKinematicLink l(0.1, 0.2, 0.3, 0.4, true);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << l;
  buf.Seek(0);

  TKinematicLink l2;
  arch >> l2;
  EXPECT_NEAR(l2.theta, 0.1, 1e-12);
  EXPECT_NEAR(l2.d, 0.2, 1e-12);
  EXPECT_NEAR(l2.a, 0.3, 1e-12);
  EXPECT_NEAR(l2.alpha, 0.4, 1e-12);
  EXPECT_TRUE(l2.is_prismatic);
}
