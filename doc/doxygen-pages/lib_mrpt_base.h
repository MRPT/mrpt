/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_base_grp [mrpt-base]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-base</code></h2>
<hr>

This is the most fundamental library in MRPT, since it provides a vast amount of utilities and OS-abstraction classes upon which
the rest of MRPT is built. Here resides critical functionality such as mathematics, linear algebra, serialization, smart pointers
and multi-threading.
l
This library comprises classes in a number of namespaces, briefly described below (click on the namespaces names to see the
complete list of its classes):


<h3>mrpt::poses</h3>

A comprehensive collection of geometry-related classes to represent all kind of 2D and 3D geomtry transformations in different formats
(Euler angles, rotation matrices, quaternions), as well as networks of pose constrains (as used typically in SLAM problems).

There are also implemented representations for probability distributions over all of these transformations, in a generic way that
allow mono and multi-modal Gaussians and particle-based representations.

See mrpt::poses for the complete list of classes here.


<h3>mrpt::utils</h3>

<ul>
<li><b>RTTI (RunTime Type Information):</b> A cross-platform, compiler-independent RTTI system is built around the base class
mrpt::utils::CObject.</li>

<li><b>Smart pointers:</b> Based on the STLplus library, any class CFoo inheriting from CObject, automatically has an associated smart
pointer class CFooPtr. MRPT implements advanced smart pointers capable of multi-thread safe usage and smart pointer typecasting with
runtime check for correct castings (<a href="http://www.mrpt.org/Smart_pointers" >tutorial</a>).</li>

<li><b>Image handling:</b> The class mrpt::utils::CImage represents a wrapper around OpenCV IplImage's, plus extra functionality
such as on-the-fly loading of images stored in disk upon first usage. The internal IplImage is always available so OpenCV's functions can
be still used to operate on MRPT images. </li>

<li><b>Serialization/Persistence:</b> Object serialization in a simple but powerful (including versioning) format
is supported by dozens of MRPT classes, all based on mrpt::utils::CSerializable. </li>

<li><b>Streams:</b> Stream classes (see the base mrpt::utils::CStream) allow serialization of MRPT objects. There are classes
for tranparent GZ-compressed files, sockets, serial ports, etc.  </li>

<li><b>XML-based databases:</b> Simple databases can be mantained, loaded and saved to files with mrpt::utils::CSimpleDatabase. </li>

<li><b>Name-based argument passing:</b> See the structure mrpt::utils::TParameters </li>

<li><b>Configuration files:</b> There is one base virtual class (mrpt::utils::CConfigFileBase) which can be used to read/write configuration
files (including basic types, vectors, matrices,...) from any "configuration source" transparently (an actual configuration file, a text block
created on the fly, etc.). </li>


</ul>

<h3>mrpt::math</h3>

MRPT defines a number of generic <i> math containers</i>, which are:

<ul>
<li><b>Matrices:</b> Dynamic-size matrices (see mrpt::math::CMatrixDouble) and compile-time fixed-size matrices (see mrpt::math::CMatrixFixedNumeric, mrpt::math::CMatrixDouble33, etc.).  </li>
<li><b>Arrays:</b> Fixed-size vectors, just like plain C arrays but with support for STL-like iterators and much more. See mrpt::math::CArrayDouble<>. </li>
</ul>

For a more in-depth description of these types, and their relation to the base Eigen classes,
read <a href="http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes" >this page</a>.

Notice that fixed-size containers should be preferred where possible, since they allow more compile-time optimizations.

Apart from the containers, this namespace contains much more functionality:

<ul>
<li>A templatized RANSAC algorithm. </li>
<li>Probability distribution functions. </li>
<li>Statistics: mean, covariance, covariance of weighted samples, etc... from sets of data.</li>
<li>A huge amount of geometry-related functions: Lines (mrpt::math::TLine3D), planes (mrpt::math::TPlane3D), segments, polygons, intersections between them, etc. </li>
<li>Graph-related stuff: generic directed graphs (mrpt::math::CDirectedGraph) and trees (mrpt::math::CDirectedTree).</li>
<li>PDF transformations (uncertainty propagation): See mrpt::math::transform_gaussian_linear, mrpt::math::transform_gaussian_montecarlo, mrpt::math::transform_gaussian_unscented.</li>
<li>A templatized implementation of quaternions, mrpt::math::CQuaternion, with support for rotation matrix convertions, Jacobians, etc.</li>
</ul>



<h3>mrpt::synch</h3>

This namespace includes threading tools such as critical sections, semaphores or utilities such as the template mrpt::synch::CThreadSafeVariable
that converts any variable into a pair variable-critical section.


<h3>mrpt::system</h3>

Here can be found functions for filesystem managing, watching directories, creating and handling threads in an OS-independent way, etc.


<h3>mrpt::compress</h3>

GZip compression methods can be found in this namespace.


*/

