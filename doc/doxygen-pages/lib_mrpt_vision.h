/** \page mrpt-vision Library overview: mrpt-vision
 *

<small> <a href="index.html#libs">Back to list of libraries</a> </small>
<br>

<h2>mrpt-vision</h2>
<hr>

This library includes some extensions to OpenCV functionality, plus some
original classes:

- The namespace mrpt::vision::pinhole contains several projection and Jacobian auxiliary functions for projective cameras.

- Sparse Bundle Adjustment algorithms. Implemented versions are:
	- mrpt::vision::bundle_adj_full : Levenberg-Marquart full optimization of camera frames (6D) + feature points (3D).

- A versatile feature tracker. See mrpt::vision::CGenericFeatureTracker and the implementation mrpt::vision::CFeatureTracker_KL

- mrpt::vision::CFeature: A generic representation of a visual feature, with or without
patch, with or without a set of descriptors.

- mrpt::vision::CFeatureExtraction: A hub for a number of detection algorithms and
 different descriptors.

- mrpt::vision::CVideoFileWriter: A class to write video files.

- This library defines a new type of observation (mrpt::slam::CObservationVisualLandmarks)
  and a new type of map (mrpt::slam::CLandmarksMap).

See all the classes in mrpt::vision


*/

