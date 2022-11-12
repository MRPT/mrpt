.. _tutorial-icp-alignment:

===================================================================
Iterative Closest Point (ICP) and other registration algorithms
===================================================================

Originally introduced in :cite:`besl1992method`, the ICP algorithm aims at
finding the **transformation between a point
cloud** **and** some reference surface (or **another point cloud**), by
minimizing the **square errors** between the corresponding entities. The
''iterative'' of ICP comes from the fact that the correspondences are
reconsidered as the solution comes closer to the error local minimum. As
any gradient descent method, the ICP is applicable when we have **a
relatively good starting point in advance**. Otherwise, it will be
trapped into the first local minimum and the solution will be useless.
In the field of mobile robots, ICP has been extensively employed to
match 2D and 3D laser scans, a problem called ''scan matching''.
See also :cite:`censi2008icp,segal2009generalized`.

ICP algorithms in MRPT can take as input:

-  Two planar (2D) maps, either:

   -  A reference map as a **cloud of points**, and a map to be aligned
      as a **cloud of points**, or
   -  A reference map as an **occupancy grid map**, and a map to be
      aligned as a **cloud of points**.

-  Two 3D maps, both represented as **clouds of points**.

For the case of point maps, a **KD-tree** is used to accelerate the
search of nearest neighbours. The ICP method is implemented in the class
`mrpt::slam::CICP <class_mrpt_slam_CICP.html>`__.
The output is a pdf (probability density function) of the relative pose
between the maps, that is, an uncertainty bound is also computed
associated to the optimal registration. An example of typical usage is
(see also the example in the directory
`MRPT/samples/icp <https://github.com/MRPT/mrpt/tree/master/samples/slam_icp_simple_example>`__):

.. code:: lang:c++

    CICP icp;
    // set ICP parameters:
    icp.options.maxIterations = 50;
    //...

    // Reference map:
    CSimplePointsMap refMap;

    // Map to be aligned:
    CSimplePointsMap alignMap;

    // Initial guess, used in the first ICP iteration:
    CPose2D initialGuess(0,0,0);
    CPosePDFPtr pdf = icp.AlignPDF(
      &refMap, // Reference map
      &alignMap, // Map to be aligned
      initialGuess // Starting estimate
      );
    CPose2D icpEstimateMean = pdf->getMeanVal();
    cout << icpEstimateMean << endl;

The different ICP algorithms implemented in the MRPT C++ library
(**explained below**) are:

-  The "classic ICP".
-  A Levenberg-Marquardt iterative method.

The following animation shows how the **threshold distance** for
establishing correspondences may have a great impact in the convergence
(or not) of ICP:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/B_OAlxjDhEM" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

Examples of usage
------------------------------

-  For **2D alignment**, refer to the example:
   `https://raw.github.com/MRPT/mrpt/master/samples/icp/ <https://github.com/MRPT/mrpt/tree/master/samples/slam_icp_simple_example>`__
-  For **`3D aligment <http://www.mrpt.org/3D-ICP_Example>`__** (full 6D
   poses), see the example:
   `https://raw.github.com/MRPT/mrpt/master/samples/icp3D/ <https://github.com/MRPT/mrpt/tree/master/samples/slam_icp_simple_example>`__

Provided algorithms
-------------------------------


The "classic" ICP algorithm
########################################

This algorithm can be invoked in MRPT via the methods
``mrpt::slam::CICP::AlignPDF()``, ``::Align()`` (or their 3D equivalent
versions) by setting ``ICP_algorithm = CICP::icpClassic`` in the
structure\ ``CICP::options``. The specific algorithm implemented in MRPT
performs a kind of **progressive refinement** as it approaches
convergence. If you want to disable the refinement stages, set the
parameter ``ALFA=0``. We show first a simplified version of the method
in **pseudo-code**, then we'll describe the more interesting parts and
finally we give the complete list of parameters and their meaning:

.. code:: lang:default

    i=0 // Iteration counter
    P(i)=P0 // Initial guess given by the user
    WHILE( i<max_iterations OR thres_dist>thres_dist_min )
      Matchings = ComputeMatching of m1 with m2 displaced by P(i) with thres_dist & thres_ang
      P(i+1) = LeastSquare(Matchings)
      IF (all components of |P(i+1)-P(i)|<1e-6)
        thres_dist *= alpha
        thres_ang *= alpha
        IF (thres_dist < thres_dist_min)
          BREAK; // End of the WHILE loop: we reached convergence
        END-IF
      END-IF
      i++;
    END-WHILE

In words: the **matching** of the transformed point cloud with the
reference point map is determined using *thres\_dist* and *thres\_ang*,
then a solver is executed to obtain the 2D or 3D **transformation that
best matches** those pairings. This is **repeated** **until
convergence** and, if ``ALFA>0`` (which is the default) the **tresholds
are reduced** and the entire process repeated. The above algorithm is
controlled by means of the following parameters in
`mrpt::slam::CICP <class_mrpt_slam_CICP.html>`__::options:

-  ``TICPAlgorithm ICP_algorithm``: ...
-  ``bool onlyClosestCorrespondences``: ...
-  ``bool onlyUniqueRobust``: ...
-  ``unsigned int maxIterations``: ...
-  ``float thresholdDist,thresholdAng``: When determining matchings
   between two point clouds, two nearby poins are considered as
   "candidate pairings" only if their distance is below
   ``thresholdDist + D*thresholdAng ``, which D being the distance of
   the point in the "to align" map to the map origin of coordinates.
   Mathematically, it models an uncertainty in the angular component of
   the pose between point clouds.
-  ``float ALFA``: ...
-  ``float smallestThresholdDist``: ...
-  ``float covariance_varPoints``: ...
-  ``bool skip_cov_calculation``: ...
-  ``bool doRANSAC``: ...
-  ``unsigned int ransac_minSetSize,ransac_maxSetSize,ransac_nSimulations``:
   ...
-  ``float ransac_mahalanobisDistanceThreshold``: ...
-  ``float normalizationStd``: ...
-  ``bool ransac_fuseByCorrsMatch``: ...
-  ``float ransac_fuseMaxDiffXY, ransac_fuseMaxDiffPhi``: ...
-  ``float kernel_rho``: ...
-  ``bool use_kernel``: ...
-  ``float Axy_aprox_derivatives``: ...
-  ``float LM_initial_lambda``: ...
-  ``uint32_t corresponding_points_decimation``: Each point in m2 is
   tested for its nearest neighbor in m1 via a KD-tree. Queries to this
   KD-tree actually are the most time-consuming part of the entire ICP
   process. Thus is why it may be a good idea, when m2 is a dense point
   cloud, to downsample it. This parameter controls that downsampling
   (default=5), but can be changed to 1 to perform an exact matching
   search. However, the heuristics give very good results and the time
   improvement is drastic, so it's recommended to set this parameter as
   high as possible while not degrading the accuracy of the result.
   Notice that only one out of "corresponding\_points\_decimation"
   points are matched against m1, but after each threshold scaling by
   "alfa", the offset of these point index is shifted, so after a
   complete ICP alignment all points from "m2" have been considered.
   Only, that not all at the same time.

The Levenberg-Marquardt ICP algorithm
########################################

In this case, the only difference with the pseudo-code above is the
replacement of this step:

::

    LeastSquare(Matchings)

by:

::

    NonLinearLeastSquare(Matchings)

where the optimizer that minimizes the average square error between
pairings is implemented following the
`Levenberg-Marquardt <page_tutorial_math_levenberg_marquardt.html>`_
algorithm. Jacobians are determined numerically to capture as well as
possible the actual distribution of points. Credits for this algorithm
are due to Dr. `Paul Newman <https://business-asset.com/eng/wiki-blog/varia/paul-newman-1001.html>`_, on
whose code was MRPT's implementation based.

Optimizing sets of correspondences
########################################

.. note::
   More general registration algorithms are provided in the more recent project `mp2p_icp <https://github.com/MOLAorg/mp2p_icp>`_

SE(2) Least Square Rigid transformation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a set of correspondences between two sets of points, this method
computes the transformation that minimizes the square error. Implemented
in
`tfest <group__mrpt__tfest__grp.html>`_::leastSquareErrorRigidTransformation.


SE(3) Least Square Rigid transformation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a set of correspondences between two sets of points, this method
computes the transformation that minimizes the square error. Implemented
in
`tfest <group__mrpt__tfest__grp.html>`_::leastSquareErrorRigidTransformation6D.


SE(2) Robust Rigid transformation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a set of correspondences between two sets of points, this method
computes a Sum of Gaussians (SOG) over the potential transformations
using a **robust RANSAC stage**. Implemented
in\ `tfest <group__mrpt__tfest__grp.html>`_::robustRigidTransformation.

