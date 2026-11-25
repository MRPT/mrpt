\defgroup mrpt_tfest_grp [mrpt-tfest]

Algorithms to find optimal transformations from sets of correspondences.

[TOC]

# Library mrpt-tfest


This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-tfest-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

<b>T</b>rans<b>f</b>ormation <b>est</b>imation (tfest): This module provides
functions in charge of solving the optimization problem of aligning a set of 2D
or 3D corresponding points, estimating the optimal transformation between the
two frames of reference.

Note that this does not include the related iterative ICP algorithm (see
mrpt::slam::CICP), included in the library \ref mrpt_slam_grp

See list of all functions: mrpt::tfest

## Overview

Given pairs of corresponding points (stored in
mrpt::tfest::TMatchingPairList), these functions find the pose `q` such that
`global = q (+) local`. Typical uses are each iteration of ICP, aligning
landmark maps, or registering points measured in two reference frames.

## Key functions

- mrpt::tfest::se2_l2(): closed-form least-squares SE(2) (x, y, yaw), with
  optional covariance of the estimate.
- mrpt::tfest::se3_l2(): closed-form least-squares SE(3) (Horn's method),
  optionally also estimating a scale factor.
- mrpt::tfest::se2_l2_robust(): RANSAC-based SE(2) estimation for sets with
  outliers, returning a sum of Gaussians with all the hypotheses.
- mrpt::tfest::se3_l2_robust(): RANSAC-based SE(3) estimation, returning the
  best transformation and the indices of the inliers.

## Example

```cpp
#include <mrpt/core/bits_math.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/tfest/se2.h>

// Ground truth, to generate synthetic correspondences:
const mrpt::poses::CPose2D gt(1.0, 2.0, mrpt::DEG2RAD(30.0));

mrpt::tfest::TMatchingPairList pairs;
for (uint32_t i = 0; i < 10; i++)
{
  const mrpt::math::TPoint3D local(0.5 * i, (i % 3) * 1.0, 0.0);
  const mrpt::math::TPoint3D global = gt.composePoint(local);
  // Arguments: global index, local index, global point, local point
  pairs.emplace_back(i, i, global, local);
}

mrpt::math::TPose2D q;
if (mrpt::tfest::se2_l2(pairs, q))
{
  std::cout << "Estimated: " << q << "\n";  // close to (1, 2, 30 deg)
}
```

## Examples

- Python: [mrpt_tfest_example.py](pymrpt_example_mrpt_tfest_example.html)
  (robust SE(3) estimation with outliers).
- C++ ICP examples using this module internally: \ref slam_icp_simple_example,
  \ref slam_icp3d_simple_example.

# Library contents
