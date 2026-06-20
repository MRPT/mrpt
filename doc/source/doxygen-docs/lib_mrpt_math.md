\defgroup mrpt_math_grp [mrpt-math]

Math C++ library: vectors and matrices, probability distributions, statistics, geometry, etc.

[TOC]

# Library mrpt-math

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-math-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

The main classes and concepts associated with this library:

 - \ref mrpt_math_vectors_matrices_grp: Vectors and matrices (compatible with
the Eigen library).
 - \ref mrpt_math_lwgeom_grp: TPose2D, TPose3D, TPoint3D, TLine3D, geometry
functions, etc. (See also: \ref mrpt_poses_grp)

Other important elements:
 - \ref filtering_grp
 - \ref fresnel_integrals_grp
 - \ref fourier_grp
 - \ref gausspdf_transform_grp
 - \ref interpolation_grp
 - \ref kdtree_grp
 - \ref polynomial_roots
 - \ref ransac_grp
 - \ref stats_grp

## Eigen integration (MRPT 3.x)

MRPT uses Eigen 3 for all matrix/vector types. Key points for MRPT 3 users:

- `mrpt::math::CMatrixDynamic<T>` and `mrpt::math::CMatrixFixed<T,R,C>` wrap
  Eigen matrices and are fully serializable via `mrpt::serialization::CArchive`.
- Use `.asEigen()` on any MRPT matrix to get the underlying Eigen expression for
  use with Eigen algorithms (decompositions, solvers, etc.).
- `mrpt::math::CVectorDynamic<T>` is a column-vector equivalent.
- Sparse matrices: use `Eigen::SparseMatrix<double>` directly; MRPT does not
  wrap sparse types but provides helpers in
  `mrpt::math::CSparseMatrix` (CXSparse-backed) for serialization.
- Geometric operations (e.g. point clouds): lightweight structs
  `mrpt::math::TPoint2Df`, `mrpt::math::TPoint3Df` (float) and their double
  variants are used for performance; they convert freely to/from Eigen vectors.

## KD-tree nearest-neighbor search

mrpt::math::KDTreeCapable is a CRTP adapter around
[nanoflann](https://github.com/jlblanco/nanoflann) providing:

- `kdTreeClosestPoint2D()` / `kdTreeClosestPoint3D()` — single NN query.
- `kdTreeNClosestPoint2D()` / `kdTreeNClosestPoint3D()` — k-NN query.
- `kdTreeRadiusSearch2D()` / `kdTreeRadiusSearch3D()` — radius search.

Derived classes must implement `kdtree_get_point_count()` and
`kdtree_get_pt(idx, dim)`. The KD-tree index is built lazily on first query
and invalidated by calling `kdtree_mark_as_outdated()`.

# Library contents
