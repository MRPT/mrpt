/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_math_grp [mrpt-math]

Math C++ library: vectors and matrices, probability distributions, statistics,
geometry, etc.

[TOC]

# Library `mrpt-math`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-math-dev

See: \ref mrpt_from_cmake

Main classes and concepts associated with this library:

 - \ref mrpt_math_lwgeom_grp: TPose2D, TPose3D, TPoint3D, TLine3D, etc. (See
also: \ref mrpt_poses_grp)
 - xxx


MRPT defines a number of generic <i> math containers</i>, which are:

<ul>
<li><b>Matrices:</b> Dynamic-size matrices (see mrpt::math::CMatrixDouble) and
compile-time fixed-size matrices (see mrpt::math::CMatrixFixed,
mrpt::math::CMatrixDouble33, etc.).  </li> <li><b>Arrays:</b> Fixed-size
vectors, just like plain C arrays but with support for STL-like iterators and
much more. See mrpt::math::CVectorFixedDouble<>. </li>
</ul>

For a more in-depth description of these types, and their relation to the base
Eigen classes, read <a
href="http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes"
>this page</a>.

Notice that fixed-size containers should be preferred where possible, since they
allow more compile-time optimizations.

Apart from the containers, this namespace contains much more functionality:

<ul>
<li>A templatized RANSAC algorithm. </li>
<li>Probability distribution functions. </li>
<li>Statistics: mean, covariance, covariance of weighted samples, etc... from
sets of data.</li> <li>A huge amount of geometry-related functions: Lines
(mrpt::math::TLine3D), planes (mrpt::math::TPlane3D), segments, polygons,
intersections between them, etc. </li> <li>Graph-related stuff: generic directed
graphs (mrpt::math::CDirectedGraph) and trees (mrpt::math::CDirectedTree).</li>
<li>PDF transformations (uncertainty propagation): See
mrpt::math::transform_gaussian_linear,
mrpt::math::transform_gaussian_montecarlo,
mrpt::math::transform_gaussian_unscented.</li> <li>A templatized implementation
of quaternions, mrpt::math::CQuaternion, with support for rotation matrix
convertions, Jacobians, etc.</li>
</ul>



*/
