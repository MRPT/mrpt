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

namespace mrpt::poses::Lie
{
/** Traits for SO(n), rotations in R^n space.
 * \ingroup mrpt_poses_lie_grp
 */
template <unsigned int n>
struct SO;

/** Traits for SO(3), rotations in R^3 space.
 * \ingroup mrpt_poses_lie_grp
 */
template <>
struct SO<3>
{
	constexpr static size_t DOFs = 3;
	using tangent_vector = mrpt::math::CVectorFixedDouble<DOFs>;
	using type = mrpt::math::CMatrixDouble33;

	/** Type for Jacobian: tangent space -> SO(n) matrix */
	using tang2mat_jacob = mrpt::math::CMatrixDouble93;

	/** Type for Jacobian: SO(n) matrix -> tangent space */
	using mat2tang_jacob = mrpt::math::CMatrixDouble39;

	/** SO(3) exponential map \f$ x \rightarrow \exp(x^\wedge) \f$.
	 * This is exactly the same than the Rodrigues formula.
	 * See 9.4.1 in \cite blanco_se3_tutorial for the exponential map
	 * definition.
	 *
	 * - Input: 3-len vector in Lie algebra so(3)
	 * - Output: 3x3 rotation matrix in SO(3)
	 */
	static type exp(const tangent_vector& x);

	/** SO(3) logarithm map \f$ \mathbf{R} \rightarrow \log(\mathbf{R}^\vee)\f$.
	 * See 10.3.1 in \cite blanco_se3_tutorial
	 *
	 * - Input: 3x3 rotation matrix in SO(3)
	 * - Output: 3-len vector in Lie algebra so(3)
	 */
	static tangent_vector log(const type& R);

	/** Returns the 3x3 SO(3) rotation matrix from yaw, pitch, roll angles.
	 * See CPose3D for the axis conventions and a picture. */
	static type fromYPR(
		const double yaw, const double pitch, const double roll);

	/** Returns vee(R-R'), which is an approximation to 2*vee(logmat(R)) for
	 * small rotations. */
	static tangent_vector vee_RmRt(const type& R);

	/** Jacobian for exp(). See 10.3.1 in \cite blanco_se3_tutorial
	 *
	 * - Input: 3-len vector in Lie algebra so(3)
	 * - Jacobian: Jacobian of the 3x3 matrix (stacked as a column major
	 * 9-vector) wrt the vector in the tangent space.
	 */
	static tang2mat_jacob jacob_dexpe_de(const tangent_vector& x);

	/** Jacobian for log()
	 * See 10.3.2 in \cite blanco_se3_tutorial
	 *
	 * - Input: 3x3 rotation matrix in SO(3)
	 * - Jacobian: Jacobian of the tangent space vector wrt the 3x3 matrix
	 * (stacked as a column major 9-vector).
	 */
	static mat2tang_jacob jacob_dlogv_dv(const type& R);
};

/** Traits for SO(2), rotations in R^2 space.
 * \ingroup mrpt_poses_lie_grp
 */
template <>
struct SO<2>
{
	constexpr static size_t DOFs = 1;
	using tangent_vector = mrpt::math::CVectorFixedDouble<DOFs>;
	using type = double;

	/** Type for Jacobian: tangent space to SO(n) matrix */
	using tang2mat_jacob = mrpt::math::CMatrixFixed<double, 1, 1>;
	/** Type for Jacobian: SO(n) matrix to tangent space */
	using mat2tang_jacob = mrpt::math::CMatrixFixed<double, 1, 1>;

	/** SO(2) exponential map \f$ x \rightarrow \exp(x^\wedge) \f$.
	 * - Input: 1-len vector in Lie algebra so(3)
	 * - Output: angle for the rotation (radians)
	 */
	static type exp(const tangent_vector& x);

	/** SO(2) logarithm map \f$ \mathbf{R} \rightarrow \log(\mathbf{R}^\vee)\f$.
	 * - Input: rotation angle [radians]
	 * - Output: 1-len vector in Lie algebra so(2) (with the same value)
	 */
	static tangent_vector log(const type& R);

	/** Jacobian for exp(), the identity matrix `[ 1 ]` */
	static tang2mat_jacob jacob_dexpe_de(const tangent_vector& x);

	/** Jacobian for log(), the identity matrix `[ 1 ]` */
	static mat2tang_jacob jacob_dlogv_dv(const type& R);
};

}  // namespace mrpt::poses::Lie
