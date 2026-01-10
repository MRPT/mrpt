/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/math/TPose3D.h>

TEST(TOrientedBox, Constructor)
{
  const mrpt::math::TOrientedBox the_box;

  EXPECT_TRUE(the_box.pose() == mrpt::math::TPose3D::Identity());
  EXPECT_TRUE(the_box.size() == mrpt::math::TPoint3D(0, 0, 0));
}

TEST(TOrientedBox, assign)
{
  const auto box_center_pose = mrpt::math::TPose3D::Identity();
  const auto box_size = mrpt::math::TPoint3D(2.0, 2.0, 2.0);

  const mrpt::math::TOrientedBox the_box{box_center_pose, box_size};

  EXPECT_TRUE(the_box.pose() == box_center_pose);
  EXPECT_TRUE(the_box.size() == box_size);
}

TEST(TOrientedBox, setters)
{
  using mrpt::literals::operator""_deg;

  const auto box_center_pose = mrpt::math::TPose3D::Identity();
  const auto box_size = mrpt::math::TPoint3D(2.0, 2.0, 2.0);

  const mrpt::math::TOrientedBox the_box{box_center_pose, box_size};

  {
    auto box2 = the_box;

    auto box_center_pose2 = mrpt::math::TPose3D(1, 2, 3, 90.0_deg, 0.0_deg, 0.0_deg);
    const auto box_size2 = mrpt::math::TPoint3D(10.0, 11.0, 12.0);

    box2.setPose(box_center_pose2);
    box2.setSize(box_size2);

    EXPECT_TRUE(box2.pose() == box_center_pose2);
    EXPECT_TRUE(box2.size() == box_size2);
  }

  {
    const auto boxf = the_box.cast<float>();
    EXPECT_TRUE(boxf.size() == box_size.cast<float>());
  }
}

TEST(TOrientedBox, ComputeVertices)
{
  using mrpt::literals::operator""_deg;

  const auto box_size = mrpt::math::TPoint3D(2.0, 4.0, 6.0);
  const auto box_center_pose = mrpt::math::TPose3D(1, 2, 3, 90.0_deg, 0.0_deg, 0.0_deg);

  const mrpt::math::TOrientedBox the_box{box_center_pose, box_size};

  const auto& vertices = the_box.vertices();
  ASSERT_EQ(vertices.size(), 8U);

  // (See box/types.hpp ascii diagram for the order of vertices)
  const std::array<mrpt::math::TPoint3D, 8> expected = {
      {
       {+3., 1., 0.}, // [0]
 {+3., 3., 0.}, // [1]
 {-1., 1., 0.}, // [2]
 {-1., 3., 0.}, // [3]
 {+3., 1., 6.}, // [4]
 {+3., 3., 6.}, // [5]
 {-1., 1., 6.}, // [6]
 {-1., 3., 6.}, // [7]
      }
  };

  for (std::size_t i = 0; i < vertices.size(); i++)
  {
    EXPECT_DOUBLE_EQ(expected[i].x, vertices[i].x)
        << "  index=" << i << "\n expected[i]=" << expected[i] << "\n vertices[i]=" << vertices[i];
    EXPECT_DOUBLE_EQ(expected[i].y, vertices[i].y);
    EXPECT_DOUBLE_EQ(expected[i].z, vertices[i].z);
  }
}

TEST(TOrientedBox, getAxisAlignedBox)
{
  using mrpt::literals::operator""_deg;

  const auto box_size = mrpt::math::TPoint3D(2.0, 4.0, 6.0);
  const auto box_center_pose = mrpt::math::TPose3D(1, 2, 3, 90.0_deg, 0.0_deg, 0.0_deg);

  const mrpt::math::TOrientedBox the_box{box_center_pose, box_size};

  const auto aab = the_box.getAxisAlignedBox();

  EXPECT_NEAR(aab.min.x, -1.0, 1e-6);
  EXPECT_NEAR(aab.min.y, 1.0, 1e-6);
  EXPECT_NEAR(aab.min.z, 0.0, 1e-6);

  EXPECT_NEAR(aab.max.x, 3.0, 1e-6);
  EXPECT_NEAR(aab.max.y, 3.0, 1e-6);
  EXPECT_NEAR(aab.max.z, 6.0, 1e-6);
}
