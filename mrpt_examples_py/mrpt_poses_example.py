#!/usr/bin/env python3

"""
Example usage of the mrpt-poses module.
"""

import math
import mrpt


def test_2d_poses():
    print("------------------------------------------")
    print("Testing 2D Poses (CPose2D)")
    print("------------------------------------------")

    # 1. Initialization
    p1 = mrpt.poses.CPose2D(1.0, 2.0, math.radians(45.0))
    print(f"p1: {p1}")

    # 2. Properties (Getters/Setters)
    # Note: We wrapped these as properties, so no brackets () needed
    p1.x += 0.5
    p1.phi = math.radians(90.0)
    print(f"p1 modified: {p1}")
    print(f"p1 norm: {p1.norm():.3f}")

    # 3. Composition (Operator Overloading)
    p2 = mrpt.poses.CPose2D(2.0, 0.0, 0.0)
    p3 = p1 + p2  # Operator + is wrapped
    print(f"p3 (p1 + p2): {p3}")

    # 4. Inverse
    p3_inv = p3.inverse()
    print(f"p3 inverse: {p3_inv}")


def test_3d_poses():
    print("\n------------------------------------------")
    print("Testing 3D Poses (CPose3D)")
    print("------------------------------------------")

    # 1. Static Builders
    # Create a pose at x=1, y=2, z=3 with some rotation
    p3d = mrpt.poses.CPose3D.FromXYZYawPitchRoll(
        1.0, 2.0, 3.0, math.radians(30), 0, 0)
    print(f"p3d: {p3d}")

    # 2. Accessing coordinates
    # We mapped z() to property .z
    print(f"Current Z: {p3d.z}")
    p3d.z = 5.0
    print(f"New Z: {p3d.z}")

    # 3. Matrix conversion
    rot_mat = p3d.getRotationMatrix()
    print(f"Rotation Matrix:\n{rot_mat}")


def test_pdfs():
    print("\n------------------------------------------")
    print("Testing PDFs (CPose3DPDFGaussian)")
    print("------------------------------------------")

    # 1. Create a PDF centered at origin
    mean_pose = mrpt.poses.CPose3D(0, 0, 0, 0, 0, 0)
    pdf = mrpt.poses.CPose3DPDFGaussian(mean_pose)

    # 2. Access Mean
    print(f"Original Mean: {pdf.mean}")

    # 3. Modify Mean via property
    pdf.mean = mrpt.poses.CPose3D(10, 0, 0, 0, 0, 0)
    print(f"New Mean: {pdf.mean}")

    # 4. Draw a random sample
    # (Requires MRPT to have been compiled with random seed initialized usually)
    sample = pdf.drawSingleSample()
    print(f"Random Sample: {sample}")


def test_averaging():
    print("\n------------------------------------------")
    print("Testing Pose Averaging (SE_average3)")
    print("------------------------------------------")

    avg_tool = mrpt.poses.SE_average3()

    # Create noisy poses around (10, 0, 0)
    p_ref = mrpt.poses.CPose3D(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Append a few poses (conceptually)
    avg_tool.append(p_ref)
    avg_tool.append(mrpt.poses.CPose3D(10.1, 0.1, 0.0, 0.0, 0.0, 0.0))
    avg_tool.append(mrpt.poses.CPose3D(9.9, -0.1, 0.0, 0.0, 0.0, 0.0))

    # Calculate average
    result = avg_tool.get_average()
    print(f"Average Pose: {result}")


if __name__ == "__main__":
    test_2d_poses()
    test_3d_poses()
    test_pdfs()
    test_averaging()
