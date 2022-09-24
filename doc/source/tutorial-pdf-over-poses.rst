.. _tutorial-pdf-over-poses:

===================================================================
Probability Density Functions (PDFs) over spatial transformations
===================================================================

These distributions can be used as to represent the robot positining belief 
and the map uncertainty in many localization and SLAM algorithms.

They include unimodal Gaussians, sum of Gaussians, sets of particles, 
and grid representations, methods to convert between them and to draw 
an arbitrary number of samples from any kind of distribution:


- Points:

  - R^3 points: `mrpt::poses::CPointPDF <class_mrpt_poses_CPointPDF.html>`_.
  
- SE(2) poses:

  - `mrpt::poses::CPosePDF <class_mrpt_poses_CPosePDF.html>`_

- SE(3) poses:

  - `mrpt::poses::CPose3DPDF <class_mrpt_poses_CPose3DPDF.html>`_ (for poses as rotation matrix and yaw/pitch/roll).

  - `mrpt::poses::CPose3DQuatPDF <class_mrpt_poses_CPose3DQuatPDF.html>`_  (for poses as quaternions) poses.

Each of the above abstract classes has implementations for different
kinds of representing the spatial uncertainty: 
particles using importance sampling, a single monomodal gaussian,
or a sum of gaussians.

The technical report :cite:`blanco_se3_tutorial` contains most
of the derivations of the implemented Jacobians.
