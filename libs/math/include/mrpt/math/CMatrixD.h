/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/point_poses2vectors.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for
 * "CMatrixDynamic<double>".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixD : public mrpt::serialization::CSerializable,
				 public CMatrixDynamic<double>
{
	DEFINE_SERIALIZABLE(CMatrixD)
	DEFINE_SCHEMA_SERIALIZABLE()
   public:
	using Base = CMatrixDynamic<double>;

	/** Constructor */
	CMatrixD() : Base(1, 1) {}
	/** Constructor */
	CMatrixD(size_t row, size_t col) : Base(row, col) {}

	/** Copy constructor */
	explicit CMatrixD(const Base& m) : Base(m) {}

	/** Copy constructor  */
	explicit CMatrixD(const CMatrixFloat& m) : Base(0, 0) { *this = m; }

	/*! Assignment operator from any other Eigen class */
	template <typename Other>
	inline CMatrixD& operator=(const Other& other)
	{
		Base::operator=(other);
		return *this;
	}
	/*! Constructor from any other Eigen class */
	template <class Other>
	explicit CMatrixD(const Other& other)
	{
		Base::operator=(other);
	}

	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixD)

};  // end of class definition
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CMatrixD::Ptr& pObj);

}  // namespace mrpt::math
