/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/poses_frwds.h>

/** \defgroup mrpt_poses_lie_grp Lie Algebra methods for SO(2),SO(3),SE(2),SE(3)
 * \ingroup poses_grp */

namespace mrpt::poses::Lie
{
/** \addtogroup mrpt_poses_lie_grp
 *  @{ */

/** Traits for SE(n), rigid-body transformations in R^n space.
 * \ingroup mrpt_poses_lie_grp
 */
template <unsigned int n>
struct SE;

/** Traits for SE(3), rigid-body transformations in R^3 space.
 * See indidual members for documentation, or \cite blanco_se3_tutorial for a
 * general overview.
 * \ingroup mrpt_poses_lie_grp
 */
template <>
struct SE<3>
{
	/** Number of actual degrees of freedom for this transformation */
	constexpr static size_t DOFs = 6;

	/** Dimensionality of the matrix manifold (3x4=12 upper part of the 4x4) */
	constexpr static size_t MANIFOLD_DIM = 3 * 4;

	using tangent_vector = mrpt::math::CVectorFixedDouble<DOFs>;
	using manifold_vector = mrpt::math::CVectorFixedDouble<MANIFOLD_DIM>;

	using type = CPose3D;
	using light_type = mrpt::math::TPose3D;

	/** Type for Jacobian: tangent space -> SO(n) matrix */
	using tang2mat_jacob = mrpt::math::CMatrixDouble12_6;

	/** Type for Jacobian: SO(n) matrix -> tangent space */
	using mat2tang_jacob = mrpt::math::CMatrixDouble6_12;

	/** Type for Jacobians between so(n) vectors on the tangent space */
	using matrix_TxT = mrpt::math::CMatrixDouble66;

	/** Type for Jacobians between SO(n) 3x4 (sub)matrices in the manifold */
	using matrix_MxM = mrpt::math::CMatrixFixed<double, 12, 12>;

	/** Retraction to SE(3), a **pseudo-exponential** map \f$ x \rightarrow
	 * PseudoExp(x^\wedge) \f$ and its Jacobian.
	 * - Input: 6-len vector in Lie algebra se(3) [x,y,z, rx,ry,rz]
	 * - Output: translation and rotation in SE(3) as CPose3D
	 * Note that this method implements retraction via a **pseudo-exponential**,
	 * where only the rotational part undergoes a real matrix exponential,
	 * while the translation is left unmodified. This is done for computational
	 * efficiency, and does not change the results of optimizations as long
	 * as the corresponding local coordinates (pseudo-logarithm) are used as
	 * well.
	 *
	 * See section 9.4.2 in \cite blanco_se3_tutorial
	 */
	static type exp(const tangent_vector& x);

	/** SE(3) **pseudo-logarithm** map \f$ \mathbf{R} \rightarrow
	 * \log(\mathbf{R}^\vee)\f$
	 * - Input: translation and rotation in SE(3) as CPose3D
	 * - Output: 6-len vector in Lie algebra se(3) [x,y,z, rx,ry,rz]
	 *
	 * See exp() for the explanation about the "pseudo" name.
	 * For the formulas, see section 9.4.2 in \cite blanco_se3_tutorial
	 */
	static tangent_vector log(const type& P);

	/** Returns a vector with all manifold matrix elements in column-major
	 * order. For SE(3), it is a 3x4=12 vector. */
	static manifold_vector asManifoldVector(const type& pose);

	/** The inverse operation of asManifoldVector() */
	static type fromManifoldVector(const manifold_vector& v);

	/** Jacobian for the pseudo-exponential exp().
	 * See 10.3.1 in \cite blanco_se3_tutorial
	 *
	 * - Input: 6-len vector in Lie algebra se(3)
	 * - Jacobian: Jacobian of the 3x4 matrix (stacked as a column major
	 * 12-vector) wrt the vector in the tangent space.
	 */
	static tang2mat_jacob jacob_dexpe_de(const tangent_vector& x);

	/** Jacobian for the pseudo-logarithm log()
	 * See 10.3.11 in \cite blanco_se3_tutorial
	 *
	 * - Input: a SE(3) pose as a CPose3D object
	 * - Jacobian: Jacobian of the tangent space vector wrt the 3x4 matrix
	 * elements (stacked as a column major 12-vector).
	 */
	static mat2tang_jacob jacob_dlogv_dv(const type& P);

	/** Jacobian d (e^eps * D) / d eps , with eps=increment in Lie Algebra.
	 * \note Section 10.3.3 in \cite blanco_se3_tutorial
	 */
	static tang2mat_jacob jacob_dexpeD_de(const CPose3D& D);

	/** Jacobian d (D * e^eps) / d eps , with eps=increment in Lie Algebra.
	 * \note Section 10.3.4 in \cite blanco_se3_tutorial
	 */
	static tang2mat_jacob jacob_dDexpe_de(const CPose3D& D);

	/** Jacobian d (A * e^eps * D) / d eps , with eps=increment in Lie
	 * Algebra.
	 * \note Eq. 10.3.7 in \cite blanco_se3_tutorial
	 */
	static tang2mat_jacob jacob_dAexpeD_de(const CPose3D& A, const CPose3D& D);

	/** Jacobian of the pose composition A*B for SE(3) 3x4 (sub)matrices,
	 * with respect to A.
	 * \note See section 7.3.1 of in \cite blanco_se3_tutorial
	 */
	static matrix_MxM jacob_dAB_dA(const type& A, const type& B);

	/** Jacobian of the pose composition A*B for SE(3) 3x4 (sub)matrices,
	 * with respect to B.
	 * \note See section 7.3.1 of in \cite blanco_se3_tutorial
	 */
	static matrix_MxM jacob_dAB_dB(const type& A, const type& B);

	/** Return one or both of the following 6x6 Jacobians, useful in
	 * graph-slam problems:
	 * \f[ \frac{\partial pseudoLn(D^{-1} P_1^{-1} P_2}{\partial \epsilon_1} \f]
	 * \f[  \frac{\partial pseudoLn(D^{-1} P_1^{-1} P_2}{\partial \epsilon_1}\f]
	 * With \f$ \epsilon_1 \f$ and \f$ \epsilon_2 \f$ increments in the
	 * linearized manifold for P1 and P2.
	 * \note Section 10.3.10 in \cite blanco_se3_tutorial
	 */
	static void jacob_dDinvP1invP2_de1e2(
		const type& Dinv, const type& P1, const type& P2,
		mrpt::optional_ref<matrix_TxT> df_de1 = std::nullopt,
		mrpt::optional_ref<matrix_TxT> df_de2 = std::nullopt);
};

/** Traits for SE(2), rigid-body transformations in R^2 space.
 * See indidual members for documentation, or \cite blanco_se3_tutorial for a
 * general overview.
 * \ingroup mrpt_poses_lie_grp
 */
template <>
struct SE<2>
{
	/** Number of actual degrees of freedom for this transformation */
	constexpr static size_t DOFs = 3;

	/** Dimensionality of the matrix manifold; this should be 3x3=9 for SE(2),
	 * but for efficiency, we define it as simply 3x1=3 and use the (x,y,phi)
	 * representation as well as the "manifold". This is done for API
	 * consistency with SE(3), where the actual matrix is used instead. */
	constexpr static size_t MANIFOLD_DIM = 3;

	using tangent_vector = mrpt::math::CVectorFixedDouble<DOFs>;
	using manifold_vector = mrpt::math::CVectorFixedDouble<MANIFOLD_DIM>;

	using type = CPose2D;
	using light_type = mrpt::math::TPose2D;

	/** Type for Jacobians between SO(n) transformations */
	using matrix_TxT = mrpt::math::CMatrixDouble33;

	/** In SE(3), this type represents Jacobians between SO(n) (sub)matrices in
	 * the manifold, but in SE(2) it simply models Jacobians between (x,y,phi)
	 * vectors. */
	using matrix_MxM = mrpt::math::CMatrixDouble33;

	/** Type for Jacobian: tangent space -> SO(n) matrix */
	using tang2mat_jacob = mrpt::math::CMatrixDouble33;
	/** Type for Jacobian: SO(n) matrix -> tangent space */
	using mat2tang_jacob = mrpt::math::CMatrixDouble33;

	/** Exponential map in SE(2), takes [x,y,phi] and returns a CPose2D */
	static type exp(const tangent_vector& x);

	/** Logarithm map in SE(2), takes a CPose2D and returns [X,Y, phi] */
	static tangent_vector log(const type& P);

	/** Returns a vector with all manifold matrix elements in column-major
	 * order. For SE(2), though, it directly returns the vector [x,y,phi] for
	 * efficiency in comparison to 3x3 homogeneous coordinates. */
	static manifold_vector asManifoldVector(const type& pose);

	/** The inverse operation of asManifoldVector() */
	static type fromManifoldVector(const manifold_vector& v);

	/** Jacobian of the pose composition A*B for SE(2) with respect to A.
	 * \note See appendix B of \cite blanco_se3_tutorial
	 */
	static matrix_MxM jacob_dAB_dA(const type& A, const type& B);

	/** Jacobian of the pose composition A*B for SE(2) with respect to B.
	 * \note See appendix B of \cite blanco_se3_tutorial
	 */
	static matrix_MxM jacob_dAB_dB(const type& A, const type& B);

	/** Jacobian d (D * e^eps) / d eps , with eps=increment in Lie Algebra.
	 * \note See appendix B in \cite blanco_se3_tutorial
	 */
	static tang2mat_jacob jacob_dDexpe_de(const type& D);

	/** Return one or both of the following 3x3 Jacobians, useful in
	 * graph-slam problems:
	 * \f[ \frac{\partial pseudoLn(D^{-1} P_1^{-1} P_2}{\partial \epsilon_1} \f]
	 * \f[  \frac{\partial pseudoLn(D^{-1} P_1^{-1} P_2}{\partial \epsilon_1}\f]
	 * With \f$ \epsilon_1 \f$ and \f$ \epsilon_2 \f$ increments in the
	 * linearized manifold for P1 and P2.
	 * \note See appendix B in \cite blanco_se3_tutorial
	 */
	static void jacob_dDinvP1invP2_de1e2(
		const type& Dinv, const type& P1, const type& P2,
		mrpt::optional_ref<matrix_TxT> df_de1 = std::nullopt,
		mrpt::optional_ref<matrix_TxT> df_de2 = std::nullopt);
};

}  // namespace mrpt::poses::Lie
