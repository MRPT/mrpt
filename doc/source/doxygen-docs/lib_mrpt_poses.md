\defgroup mrpt_poses_grp [mrpt-poses]

SE(2)/SE(3) poses and probability distributions

[TOC]

# Library mrpt-poses

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-poses-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

`mrpt-poses` provides SE(2)/SE(3) rigid-body transformations and their
probability distributions:

## Deterministic poses

| Class | Description |
|-------|-------------|
| mrpt::poses::CPose2D | 2D pose (x, y, φ) |
| mrpt::poses::CPose3D | 3D pose using yaw/pitch/roll angles |
| mrpt::poses::CPose3DQuat | 3D pose using a unit quaternion |
| mrpt::poses::CPoint2D | 2D point |
| mrpt::poses::CPoint3D | 3D point |

Lightweight (non-serializable) equivalents live in `mrpt-math`:
`mrpt::math::TPose2D`, `mrpt::math::TPose3D`, `mrpt::math::TPoint3D`.

## Probabilistic pose PDFs

All PDF classes derive from `mrpt::poses::CPosePDF` (2D) or
`mrpt::poses::CPose3DPDF` (3D):

| Class | Representation |
|-------|----------------|
| mrpt::poses::CPosePDFGaussian | Gaussian (mean + 3×3 covariance) |
| mrpt::poses::CPosePDFGaussianInf | Gaussian in information form |
| mrpt::poses::CPosePDFParticles | Particle filter — discrete samples |
| mrpt::poses::CPosePDFSOG | Sum of Gaussians |
| mrpt::poses::CPose3DPDFGaussian | 3D Gaussian (mean + 6×6 covariance) |
| mrpt::poses::CPose3DPDFParticles | 3D particle-filter PDF |
| mrpt::poses::CPose3DPDFSOG | 3D sum of Gaussians |

## Pose interpolation and trajectories

mrpt::poses::CPoseInterpolatorBase and its derived
mrpt::poses::CPoses2DSequence / mrpt::poses::CPoses3DSequence store
timestamped pose sequences and can interpolate between them with
linear or spline methods.

# Library contents
