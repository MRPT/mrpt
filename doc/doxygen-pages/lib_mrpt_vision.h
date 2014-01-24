/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_vision_grp [mrpt-vision]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-vision</code></h2>
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

- mrpt::vision::TSIFTDescriptorsKDTreeIndex, mrpt::vision::find_descriptor_pairings() and others: KD-tree-based SIFT/SURF feature matching.

- mrpt::vision::CVideoFileWriter: A class to write video files.

- mrpt::vision::CUndistortMap: A cache of the map for undistorting image, very efficient for sequences of images all with the same distortion parameters.

- This library defines a new type of observation (mrpt::slam::CObservationVisualLandmarks)
  and a new type of map (mrpt::slam::CLandmarksMap).

- Notice that sets of parameters for monocular and stereo cameras are defined in [mrpt-base] for convenience, in the classes:
	- mrpt::utils::TCamera
	- mrpt::utils::TStereoCamera
	
- mrpt::vision::CDifodo: A class which implements a Visual Odometry algorithm based on depth images and the "range flow constraint equation".

See all the classes in mrpt::vision


*/

