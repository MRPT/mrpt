/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_base_grp [mrpt-base]

# mrpt-base: Obsolete in MRPT 2.0.0!

In MRPT 2.0, `mrpt-base` has been split into many smaller libraries for the
sake of modularity of easy of code reusability.

TODO: Move these comments to separate lib_*.h files

A comprehensive collection of geometry-related classes to represent all kind of
2D and 3D geomtry transformations in different formats (Euler angles, rotation
matrices, quaternions), as well as networks of pose constrains (as used
typically in SLAM problems).

There are also implemented representations for probability distributions over
all of these transformations, in a generic way that allow mono and multi-modal
Gaussians and particle-based representations.

See mrpt::poses for the complete list of classes here.


<ul>

<li><b>XML-based databases:</b> Simple databases can be mantained, loaded and
saved to files with mrpt::db::CSimpleDatabase. </li>

<li><b>Configuration files:</b> There is one base virtual class
(mrpt::config::CConfigFileBase) which can be used to read/write configuration
files (including basic types, vectors, matrices,...) from any "configuration
source" transparently (an actual configuration file, a text block created on the
fly, etc.). </li>


</ul>

<h3>mrpt::math</h3>

MRPT defines a number of generic <i> math containers</i>, which are:

<ul>
<li><b>Matrices:</b> Dynamic-size matrices (see mrpt::math::CMatrixDouble) and
compile-time fixed-size matrices (see mrpt::math::CMatrixFixedNumeric,
mrpt::math::CMatrixDouble33, etc.).  </li> <li><b>Arrays:</b> Fixed-size
vectors, just like plain C arrays but with support for STL-like iterators and
much more. See mrpt::math::CArrayDouble<>. </li>
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


<h3>mrpt::compress</h3>

GZip compression methods can be found in this namespace.


*/
