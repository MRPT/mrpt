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

All PDF classes derive from `mrpt::poses::CPosePDF` (2D),
`mrpt::poses::CPose3DPDF` (3D, yaw/pitch/roll) or
`mrpt::poses::CPose3DQuatPDF` (3D, quaternion). See also the
[PDFs over spatial transformations tutorial](tutorial-pdf-over-poses.html).

| Class | Representation |
|-------|----------------|
| mrpt::poses::CPosePDFGaussian | Gaussian (mean + 3x3 covariance) |
| mrpt::poses::CPosePDFGaussianInf | Gaussian in information form |
| mrpt::poses::CPosePDFParticles | Set of weighted samples (particles) |
| mrpt::poses::CPosePDFSOG | Sum of Gaussians |
| mrpt::poses::CPosePDFGrid | Discrete grid over (x, y, phi) |
| mrpt::poses::CPose3DPDFGaussian | 3D Gaussian (mean + 6x6 covariance) |
| mrpt::poses::CPose3DPDFGaussianInf | 3D Gaussian in information form |
| mrpt::poses::CPose3DPDFParticles | 3D set of weighted samples (particles) |
| mrpt::poses::CPose3DPDFSOG | 3D sum of Gaussians |
| mrpt::poses::CPose3DPDFGrid | Discrete grid over the 6D pose space |
| mrpt::poses::CPose3DQuatPDFGaussian | 3D Gaussian (mean + 7x7 covariance) |
| mrpt::poses::CPose3DQuatPDFGaussianInf | 3D Gaussian in information form (quaternion) |

## Pose interpolation and trajectories

mrpt::poses::CPose2DInterpolator and mrpt::poses::CPose3DInterpolator (both
derived from mrpt::poses::CPoseInterpolatorBase) store timestamped pose
sequences and interpolate between them with the methods in
mrpt::poses::TInterpolatorMethod (linear, splines, SLERP for rotations).
mrpt::poses::CPoses2DSequence / mrpt::poses::CPoses3DSequence store sequences
of relative (incremental) poses instead.

# Library contents
