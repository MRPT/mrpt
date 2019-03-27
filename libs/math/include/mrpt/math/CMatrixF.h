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
/**  This class is a "CSerializable" wrapper for "CMatrixFloat".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixF : public mrpt::serialization::CSerializable, public CMatrixFloat
{
	DEFINE_SERIALIZABLE(CMatrixF)
	DEFINE_SCHEMA_SERIALIZABLE()

   public:
	/** Constructor  */
	CMatrixF() : CMatrixFloat(1, 1) {}

	/** Constructor */
	CMatrixF(size_t row, size_t col) : CMatrixFloat(row, col) {}

	/** Copy constructor */
	explicit CMatrixF(const CMatrixFloat& m) : CMatrixFloat(m) {}

	/** Copy constructor */
	explicit CMatrixF(const CMatrixDynamic<double>& m) : CMatrixFloat(m) {}
	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixF)

	/** Assignment operator for float matrixes
	 */
	template <class OTHERMAT>
	inline CMatrixF& operator=(const OTHERMAT& m)
	{
		CMatrixFloat::operator=(m);
		return *this;
	}

	/*! Assignment operator from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrixF& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		CMatrixDynamic<float>::operator=(other);
		return *this;
	}
	/*! Constructor from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrixF(const Eigen::MatrixBase<OtherDerived>& other)
		: CMatrixDynamic<float>(other)
	{
	}

};  // end of class definition
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CMatrixF::Ptr& pObj);

}  // namespace mrpt::math
