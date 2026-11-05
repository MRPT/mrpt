.. _tutorial-pdf-over-poses:

===================================================================
Probability Density Functions (PDFs) over spatial transformations
===================================================================

These distributions can be used to represent the robot positioning belief
and the map uncertainty in many localization and SLAM algorithms.

They include unimodal Gaussians (in covariance or information form), sums of
Gaussians, sets of particles, and grid representations, together with methods
to convert between them and to draw an arbitrary number of samples from any
kind of distribution.

The abstract base classes, and their implementations, are:

- 2D points:

  - `mrpt::poses::CPoint2DPDF <class_mrpt_poses_CPoint2DPDF.html>`_:
    `CPoint2DPDFGaussian <class_mrpt_poses_CPoint2DPDFGaussian.html>`_.

- 3D points:

  - `mrpt::poses::CPointPDF <class_mrpt_poses_CPointPDF.html>`_:
    `CPointPDFGaussian <class_mrpt_poses_CPointPDFGaussian.html>`_,
    `CPointPDFSOG <class_mrpt_poses_CPointPDFSOG.html>`_,
    `CPointPDFParticles <class_mrpt_poses_CPointPDFParticles.html>`_.

- SE(2) poses:

  - `mrpt::poses::CPosePDF <class_mrpt_poses_CPosePDF.html>`_:
    `CPosePDFGaussian <class_mrpt_poses_CPosePDFGaussian.html>`_,
    `CPosePDFGaussianInf <class_mrpt_poses_CPosePDFGaussianInf.html>`_,
    `CPosePDFSOG <class_mrpt_poses_CPosePDFSOG.html>`_,
    `CPosePDFParticles <class_mrpt_poses_CPosePDFParticles.html>`_,
    `CPosePDFGrid <class_mrpt_poses_CPosePDFGrid.html>`_.

- SE(3) poses, with the rotation as yaw/pitch/roll angles:

  - `mrpt::poses::CPose3DPDF <class_mrpt_poses_CPose3DPDF.html>`_:
    `CPose3DPDFGaussian <class_mrpt_poses_CPose3DPDFGaussian.html>`_,
    `CPose3DPDFGaussianInf <class_mrpt_poses_CPose3DPDFGaussianInf.html>`_,
    `CPose3DPDFSOG <class_mrpt_poses_CPose3DPDFSOG.html>`_,
    `CPose3DPDFParticles <class_mrpt_poses_CPose3DPDFParticles.html>`_,
    `CPose3DPDFGrid <class_mrpt_poses_CPose3DPDFGrid.html>`_.

- SE(3) poses, with the rotation as a quaternion:

  - `mrpt::poses::CPose3DQuatPDF <class_mrpt_poses_CPose3DQuatPDF.html>`_:
    `CPose3DQuatPDFGaussian <class_mrpt_poses_CPose3DQuatPDFGaussian.html>`_,
    `CPose3DQuatPDFGaussianInf <class_mrpt_poses_CPose3DQuatPDFGaussianInf.html>`_.

The technical report :cite:`blanco_se3_tutorial` contains most
of the derivations of the implemented Jacobians.
