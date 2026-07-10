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
#include <mrpt/obs/CObservationReflectivity.h>

using namespace mrpt::obs;

// CObservationReflectivity is a small, concrete CObservation subclass with a
// trivial getSensorPose()/setSensorPose(CPose3D) pair, convenient for
// exercising the CObservation base-class logic in CObservation.cpp.
// (Note: swap() is protected in CObservation, so it is exercised indirectly
// through the public swap() of concrete subclasses that call it, e.g.
// CObservation3DRangeScan::swap(), already tested elsewhere.)

TEST(CObservation, GetSetSensorPoseTPose3DOverloads)
{
  CObservationReflectivity o;
  o.sensorPose = mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3);

  // These TPose3D overloads are declared (non-virtual) in the CObservation
  // base class, so they must be called through a base-class reference since
  // the derived class hides them by declaring its own getSensorPose()/
  // setSensorPose(const CPose3D&) overrides.
  CObservation& base = o;

  mrpt::math::TPose3D outPose;
  base.getSensorPose(outPose);
  EXPECT_NEAR(outPose.x, 1.0, 1e-9);
  EXPECT_NEAR(outPose.y, 2.0, 1e-9);

  const mrpt::math::TPose3D newPose(5, 6, 7, 0, 0, 0);
  base.setSensorPose(newPose);
  EXPECT_NEAR(o.sensorPose.x(), 5.0, 1e-9);
}

TEST(CObservation, AsString)
{
  CObservationReflectivity o;
  o.sensorLabel = "refl";
  const std::string s = o.asString();
  EXPECT_NE(s.find("refl"), std::string::npos);
  EXPECT_NE(s.find("ClassName"), std::string::npos);
}
