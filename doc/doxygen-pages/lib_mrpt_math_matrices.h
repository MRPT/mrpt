/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

// clang-format off

/** \defgroup mrpt_math_vectors_matrices_grp Vectors, matrices, linear Algebra
\ingroup mrpt_math_grp

Dynamic and fixed-size vectors and matrices, basic linear Algebra

[TOC]

# Design rationale

Unlike in older MRPT 1.x versions, matrices and vectors since MRPT v2.0.0 **do
no longer inherit** from Eigen classes. Instead, they are now thin wrappers
around static/dynamic memory containers, which *can* be casted to
Eigen-compatible classes, but which allow most common operations to be done
without Eigen.

The reason for this scheme is two-fold:
 - Including Eigen in *all* headers significantly slows down build times.
 - Backwards-compatible MRPT code required using Eigen's plugin mechanism to
inject code inside Eigen::DenseBase. This was a source of problems when using
MRPT in an application that uses Eigen on its own, since an error would be
raised in MRPT math headers were not always included first.

# Important facts

 - Fixed-size containers should be preferred where possible, since they
  allow more compile-time optimizations and avoid dynamic memory
(de)allocations.

 - Most common uses of vectors and matrices require `#include`-ing just the
corresponding MRPT headers.

 - If some particular feature is not exposed in the MRPT API, then you can map
the MRPT container into an Eigen Map and use the regular Eigen API.

 - For fixed-sized matrices and vectors, only a subset of all infinite possible
sizes are explicitly instantiated in the mrpt-math library. This means that if
you use a non-supported size, you will not have any build error but your program
will fail to link. The alternative then is to either use `.asEigen()` or casting
to a dynamic-sized container.

 - Exact list of explicitly-instantiated vectors:
  - mrpt::math::CVectorDynamic<T>: For `T`=`float` and `T`=`double`. Any dynamic size.
  - mrpt::math::CVectorFixed<T,N>: For `T`=`float` and `T`=`double`, and `N`=2,3,4,5,6,7,12.

 - Exact list of explicitly-instantiated matrices:
   - mrpt::math::CMatrixDynamic<T>: For `T`=`float` and `T`=`double`. Any dynamic size.
   - mrpt::math::CMatrixFixed<T,N,N>: For `T`=`float` and `T`=`double`, and `N`=2,3,4,6,7.

 - All matrices and vectors support build-time generation of constexpr strings
describing their types, via: \ref mrpt_typemeta_grp

 - For binary serialization of dynamic-sized matrices, the following classes are
provided which implements the CSerializable interface (see: \ref
mrpt_serialization_grp):
   - mrpt::math::CMatrixF (float, NxM).
   - mrpt::math::CMatrixD (double, NxM).

## Example: matrix sum (MRPT methods, no explicit call to Eigen)

```
#include <mrpt/math/CMatrixDynamic.h>
#include <iostream>
// ...
mrpt::math::CMatrixDouble M1;
M1.setDiagonal(4,0.1);
//M1.loadFromTextFile("M1.txt");

auto M2 = mrpt::math::CMatrixDouble::Identity(4);

// Sum:
mrpt::math::CMatrixDouble R = M1 + M2;
std::cout << "R:\n" << R << "\n";
R.saveToTextFile("R.txt");
```

## Example: QR-based linear system solving (With explicit call to Eigen)

```
#include <mrpt/math/CMatrixDynamic.h>
#include <Eigen/Dense>  // Must add this one to use .asEigen()
#include <iostream>
// ...
mrpt::math::CMatrixDouble33 A;
A.setDiagonal(3,0.2);

mrpt::math::CVectorDouble<3> b;
b.fill(0.1);;

mrpt::math::CVectorDouble<3> x;

// Solve Ax=b
x.asEigen() = A.asEigen().fullPivHouseholderQr().solve(x);

std::cout << "x:\n" << x.asString() << "\n";
```

See list of classes below.


*/
