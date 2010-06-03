/** \page mrpt-base Library overview: mrpt-base
 *

<small> <a href="index.html#libs">Back to list of libraries</a> </small>
<br>

<h2>mrpt-base</h2>
<hr>

This is the most fundamental library in MRPT, since it provides a vast amount of utilities and OS-abstraction classes upon which 
the rest of MRPT is built. Here resides critical functionality such as mathematics, linear algebra, serialization, smart pointers
and multi-threading.

This library comprises classes in a number of namespaces, briefly described below (click on the namespaces names to see the 
complete list of its classes):


<h3>mrpt::poses</h3>

A comprehensive collection of geometry-related classes to represent all kind of 2D and 3D geomtry transformations in different formats 
(Euler angles, rotation matrices, quaternions), as well as networks of pose constrains (as used typically in SLAM problems). 

There are also implemented representations for probability distributions over all of these transformations, in a generic way that
allow mono and multi-modal Gaussians and particle-based representations. See mrpt::poses for the complete list of classes here.


<h3>mrpt::utils</h3>

<ul>
<li><b>RTTI (RunTime Type Information):</b> A cross-platform, compiler-independent RTTI system is built around the base class
mrpt::utils::CObject.</li>

<li><b>Smart pointers:</b> Based on the STLplus library, any class CFoo inheriting from CObject, automatically has an associated smart
pointer class CFooPtr. MRPT implements advanced smart pointers capable of multi-thread safe usage and smart pointer typecasting with 
runtime check for correct castings. </li> 

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
<li><b>Matrix views:</b> Proxy classes that allow operating on the transpose, a part of, or the diagonal of another matrix as if it was a plain matrix 
object. See mrpt::math::CMatrixView.   </li>
<li><b>Vectors:</b> Dynamic-size vectors. See mrpt::vector_double, which inherits from a standard STL vector<double>.  </li>
<li><b>Arrays:</b> Fixed-size vectors, just like plain C arrays but with support for STL-like iterators and much more. See mrpt::math::CArrayDouble<>. </li>
</ul>

These contaners have a number of characteristics in common (STL-like iterators and typedefs, etc.) and can be 
mixed in operations without problems. For example, matrices of any kind can be operated together, a vector can be
added to an array, or the results of a matrix operation stored in a matrix view. 

Notice that fixed-size contaners should be preferred where possible, since they allow more compile-time optimizations.

Apart from the containers, this namespace contains much more functionality:

<ul>
<li>A templatized RANSAC algorithm. </li>
<li>Probability distribution functions. </li>
<li>Statistics: mean, covariance, covariance of weighted samples, etc... from sets of data.</li>
<li>A huge amount of geometry-related functions: Lines (mrpt::math::TLine3D), planes (mrpt::math::TPlane3D), segments, polygons, intersections between them, etc. </li>
</ul>


<h3>mrpt::bayes</h3>

Here there are two main family of algorithms:
<ul>
<li><b>Kalman filters:</b> A generic, templatized Kalman filter implementation (includes EKF,IEKF and in the future, UKF), which 
only requires from the programmer to provide the system models and (optinally) the Jacobians. See mrpt::bayes::CKalmanFilterCapable </li>
<li><b>Particle filters:</b> A set of helper classes and functions to perform particle filtering. In this case the 
algorithms are not as generic as in Kalman filtering, but the classes serve to organize and unify the interface of different 
PF algorithms in MRPT. See mrpt::bayes::CParticleFilter. </li>
</ul>


<h3>mrpt::synch</h3>

This namespace includes threading tools such as critical sections, semaphores or utilities such as the template mrpt::synch::CThreadSafeVariable 
that converts any variable into a pair variable-critical section.


<h3>mrpt::scan_matching</h3>

Under this name we find functions in charge of solving the optimization problem of aligning a set of correspondences, both in 2D and in 3D. 
Note that this does not includes the iterative ICP algorithm, included in another library (mrpt-slam). 


<h3>mrpt::system</h3>

Here can be found functions for filesystem managing, watching directories, creating and handling threads in an OS-independent way, etc.


<h3>mrpt::compress</h3>

GZip compression methods can be found in this namespace. 


*/

