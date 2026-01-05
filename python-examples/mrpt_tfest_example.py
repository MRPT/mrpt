#!/usr/bin/env python3
"""
MRPT tfest (Transformation Estimation) Example
Demonstrates robust SE(3) alignment between two sets of point correspondences.
"""

import numpy as np
import mrpt.math
import mrpt.poses
import mrpt.tfest


def create_correspondences(local_pts, global_pts):
    """
    Convenience function to create a TMatchingPairList from two Nx3 numpy arrays.
    """
    import numpy as np
    pair_list = mrpt.tfest.TMatchingPairList()
    for i in range(len(local_pts)):
        p = mrpt.tfest.TMatchingPair()
        p.global_pt = mrpt.math.TPoint3Df(global_pts[i])
        p.local_pt = mrpt.math.TPoint3Df(local_pts[i])
        p.globalIdx = i
        p.localIdx = i
        pair_list.push_back(p)
    return pair_list


def main():
    # 1. Generate Synthetic Data
    # ---------------------------------------------------------
    print("Generating synthetic point clouds...")
    num_points = 50
    # Create random points in 'this' frame
    pts_this = np.random.uniform(-5, 5, (num_points, 3))

    # Define a ground truth transformation (Translation + Yaw rotation)
    gt_pose = mrpt.poses.CPose3D(1.5, -0.8, 0.2, np.radians(30), 0, 0)

    # Transform points to 'other' frame: p_this = pose (+) p_other
    # Since we want p_other, we apply the inverse transformation
    inv_gt = gt_pose.getInverseHomogeneousMatrix()  # returns 4x4 numpy-compatible

    # Manual transformation for the example:
    pts_other = []
    for p in pts_this:
        # p_other = gt_pose.inverseComposePoint(p)
        p_other = gt_pose.inverseComposePoint(p[0], p[1], p[2])
        pts_other.append(p_other)
    pts_other = np.array(pts_other)

    # Add some outliers to test RANSAC robustness
    pts_other[0] += [10.0, 10.0, 10.0]
    pts_other[1] += [-5.0, 20.0, 0.0]

    # 2. Create Correspondence List
    # ---------------------------------------------------------
    # We use the convenience helper from our tfest/__init__.py
    print(f"Creating {num_points} correspondences...")
    corr_list = create_correspondences(pts_this, pts_other)

    # 3. Configure and Run Robust SE(3) Estimation
    # ---------------------------------------------------------
    print("Running Robust SE(3) L2 (RANSAC)...")

    params = mrpt.tfest.TSE3RobustParams()
    params.ransac_minSetSize = 5
    params.ransac_nmaxSimulations = 200
    # params.ransac_mahalanobisDistanceThreshold = 0.05

    # Run estimator: returns (success, results_struct)
    ok, results = mrpt.tfest.se3_l2_robust(corr_list, params)

    if ok:
        print("\nEstimation Successful!")
        print("-" * 30)
        # The transformation is a CPose3DQuat object
        print(f"Estimated Pose:  {results.transformation}")
        print(f"Ground Truth:    {gt_pose}")

        # Check inliers
        inliers = results.inliers_idx
        print(f"Inliers found:   {len(inliers)} / {num_points}")

        # 4. Use the result with NumPy
        # ---------------------------------------------------------
        # Convert the rotation part of the result to a numpy matrix
        # Note: CPose3DQuat can be converted to CPose3D
        final_pose = mrpt.poses.CPose3D(results.transformation)
        rot_mat = np.array(final_pose.getRotationMatrix())

        print("\nRotation Matrix (NumPy):")
        print(rot_mat)
    else:
        print("Estimation failed!")


if __name__ == "__main__":
    main()
