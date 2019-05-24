/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/core/is_defined.h>
#include <mrpt/core/optional_ref.h>
#include <algorithm>  // fill()
#include <cstddef>  // size_t
#include <iosfwd>
#include <string>
#include <vector>

namespace mrpt::math
{
template <class T>
class CVectorDynamic;
template <class T>
class CMatrixDynamic;
template <typename T, std::size_t ROWS, std::size_t COLS>
class CMatrixFixed;

template <typename DER>
void internalAssertEigenDefined();

/*! Selection of the number format in MatrixVectorBase::saveToTextFile()
 * \ingroup mrpt_math_grp */
enum TMatrixTextFileFormat
{
	/** engineering format '%e' */
	MATRIX_FORMAT_ENG = 0,
	/** fixed floating point '%f' */
	MATRIX_FORMAT_FIXED = 1,
	/** intergers '%i' */
	MATRIX_FORMAT_INT = 2
};

/**  Base CRTP class for all MRPT vectors and matrices.
 *
 * Template methods whose implementation is not in this header file are
 * explicitly instantiated and exported for these derived types:
 * - CMatrixDynamic and CVectorDynamic, for `Scalar`: `float`, `double`
 * - CMatrixFixed and CVectorFixed, for `Scalar`: `float`, `double`, and sizes
 * from 2x2 to 6x6 and 2 to 6, respectively.
 *
 * \sa CMatrixFixed
 * \ingroup mrpt_math_grp
 */
template <typename Scalar, class Derived>
class MatrixVectorBase
{
   public:
	Derived& mvbDerived() { return static_cast<Derived&>(*this); }
	const Derived& mvbDerived() const
	{
		return static_cast<const Derived&>(*this);
	}

	/** @name Initialization methods
	 * @{ */

	/*! Fill all the elements with a given value (Note: named "fillAll" since
	 * "fill" will be used by child classes) */
	void fill(const Scalar& val)
	{
		std::fill(mvbDerived().begin(), mvbDerived().end(), val);
	}

	inline void setZero() { fill(0); }
	inline void setZero(size_t nrows, size_t ncols)
	{
		mvbDerived().resize(nrows, ncols);
		fill(0);
	}

	static Derived Zero()
	{
		ASSERTMSG_(
			Derived::RowsAtCompileTime > 0 && Derived::ColsAtCompileTime > 0,
			"Zero() without arguments can be used only for fixed-size "
			"matrices/vectors");
		Derived m;
		m.setZero();
		return m;
	}
	static Derived Zero(size_t nrows, size_t ncols)
	{
		Derived m;
		m.setZero(nrows, ncols);
		return m;
	}

	inline void assign(const std::size_t N, const Scalar value)
	{
		mvbDerived().resize(N);
		fill(value);
	}
	/** @} */

	/** @name Operations that DO require `#include <Eigen/Dense>` in user code
	 * @{ */

	/** non-const block(): Returns an Eigen::Block reference to the block */
	template <int BLOCK_ROWS, int BLOCK_COLS>
	auto block(int start_row = 0, int start_col = 0)
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().template block<BLOCK_ROWS, BLOCK_COLS>(
			start_row, start_col);
	}

	auto block(int start_row, int start_col, int BLOCK_ROWS, int BLOCK_COLS)
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().block(
			start_row, start_col, BLOCK_ROWS, BLOCK_COLS);
	}
	auto block(
		int start_row, int start_col, int BLOCK_ROWS, int BLOCK_COLS) const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().block(
			start_row, start_col, BLOCK_ROWS, BLOCK_COLS);
	}

	auto transpose()
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().transpose();
	}
	auto transpose() const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().transpose();
	}

	auto array()
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().array();
	}
	auto array() const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen().array();
	}

	auto operator-() const
	{
		internalAssertEigenDefined<Derived>();
		return -mvbDerived().asEigen();
	}

	template <typename S2, class D2>
	auto operator+(const MatrixVectorBase<S2, D2>& m2) const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen() + m2.mvbDerived().asEigen();
	}
	template <typename S2, class D2>
	void operator+=(const MatrixVectorBase<S2, D2>& m2)
	{
		internalAssertEigenDefined<Derived>();
		mvbDerived().asEigen() += m2.mvbDerived().asEigen();
	}

	template <typename S2, class D2>
	auto operator-(const MatrixVectorBase<S2, D2>& m2) const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen() - m2.mvbDerived().asEigen();
	}
	template <typename S2, class D2>
	void operator-=(const MatrixVectorBase<S2, D2>& m2)
	{
		internalAssertEigenDefined<Derived>();
		mvbDerived().asEigen() -= m2.mvbDerived().asEigen();
	}

	template <typename S2, class D2>
	auto operator*(const MatrixVectorBase<S2, D2>& m2) const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen() * m2.mvbDerived().asEigen();
	}
	auto operator*(const Scalar s) const
	{
		internalAssertEigenDefined<Derived>();
		return mvbDerived().asEigen() * s;
	}

	template <int N>
	CMatrixFixed<Scalar, N, 1> tail() const
	{
		return CMatrixFixed<Scalar, N, 1>(
			mvbDerived().asEigen().template tail<N>());
	}
	template <int N>
	CMatrixFixed<Scalar, N, 1> head() const
	{
		return CMatrixFixed<Scalar, N, 1>(
			mvbDerived().asEigen().template head<N>());
	}

	/** @} */

	/** @name Standalone operations (do NOT require `#include <Eigen/Dense>`)
	 * @{ */

	Scalar& coeffRef(int r, int c) { return mvbDerived()(r, c); }
	const Scalar& coeff(int r, int c) const { return mvbDerived()(r, c); }

	/** const blockCopy(): Returns a *copy* of the given block */
	template <int BLOCK_ROWS, int BLOCK_COLS>
	CMatrixFixed<Scalar, BLOCK_ROWS, BLOCK_COLS> blockCopy(
		int start_row = 0, int start_col = 0) const
	{
		return mvbDerived().template extractMatrix<BLOCK_ROWS, BLOCK_COLS>(
			start_row, start_col);
	}

	/** Minimum value in the matrix/vector */
	Scalar minCoeff() const;
	Scalar minCoeff(std::size_t& outIndexOfMin) const;
	Scalar minCoeff(std::size_t& rowIdx, std::size_t& colIdx) const;

	/** Maximum value in the matrix/vector */
	Scalar maxCoeff() const;
	Scalar maxCoeff(std::size_t& outIndexOfMax) const;
	Scalar maxCoeff(std::size_t& rowIdx, std::size_t& colIdx) const;

	/** returns true if matrix is NxN */
	bool isSquare() const { return mvbDerived().cols() == mvbDerived().rows(); }

	/** returns true if matrix/vector has size=0 */
	bool empty() const
	{
		return mvbDerived().cols() == 0 && mvbDerived().rows() == 0;
	}

	/** Compute the norm-infinite of a vector ($f[ ||\mathbf{v}||_\infnty $f]),
	 * ie the maximum absolute value of the elements. */
	Scalar norm_inf() const;

	/** Compute the L2 norm of a vector/array/matrix (the Euclidean distance
	 * to the origin, taking all the elements as a single vector). \sa norm */
	Scalar norm() const;

	void operator+=(Scalar s);
	void operator-=(Scalar s);
	void operator*=(Scalar s);

	CMatrixDynamic<Scalar> operator*(const CMatrixDynamic<Scalar>& v);

	Derived operator+(const Derived& m2) const
	{
		if constexpr (
			Derived::RowsAtCompileTime == Derived::ColsAtCompileTime ||
			Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == -1)
		{
			return impl_op_add(m2);
		}
		else
		{
			throw std::runtime_error(
				"Explicit instantiation not provided for this matrix size: use "
				"asEigen()");
		}
	}
	void operator+=(const Derived& m2)
	{
		if constexpr (
			Derived::RowsAtCompileTime == Derived::ColsAtCompileTime ||
			Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == -1)
		{
			impl_op_selfadd(m2);
		}
		else
		{
			throw std::runtime_error(
				"Explicit instantiation not provided for this matrix size: use "
				"asEigen()");
		}
	}
	Derived operator-(const Derived& m2) const
	{
		if constexpr (
			Derived::RowsAtCompileTime == Derived::ColsAtCompileTime ||
			Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == -1)
		{
			return impl_op_subs(m2);
		}
		else
		{
			throw std::runtime_error(
				"Explicit instantiation not provided for this matrix size: use "
				"asEigen()");
		}
	}
	void operator-=(const Derived& m2)
	{
		if constexpr (
			Derived::RowsAtCompileTime == Derived::ColsAtCompileTime ||
			Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == -1)
		{
			impl_op_selfsubs(m2);
		}
		else
		{
			throw std::runtime_error(
				"Explicit instantiation not provided for this matrix size: use "
				"asEigen()");
		}
	}
	Derived operator*(const Derived& m2) const;

   private:
	Derived impl_op_add(const Derived& m2) const;
	void impl_op_selfadd(const Derived& m2);
	Derived impl_op_subs(const Derived& m2) const;
	void impl_op_selfsubs(const Derived& m2);

   public:
	/** dot product of `this \cdot v ` */
	Scalar dot(const CVectorDynamic<Scalar>& v) const;
	Scalar dot(const MatrixVectorBase<Scalar, Derived>& v) const;

	/** this = A * b , with `A` and `b` a dynamic matrix & vector */
	void matProductOf_Ab(
		const CMatrixDynamic<Scalar>& A, const CVectorDynamic<Scalar>& b);

	/** this = A<sup>T</sup> * b , with `A` and `b` a dynamic matrix & vector */
	void matProductOf_Atb(
		const CMatrixDynamic<Scalar>& A, const CVectorDynamic<Scalar>& b);

	/** Sum of all elements in matrix/vector. */
	Scalar sum() const;

	/** Sum of the absolute value of all elements in matrix/vector. */
	Scalar sum_abs() const;

	/** Returns a string representation of the vector/matrix, using Eigen's
	 * default settings. */
	std::string asStr() const;

	/** Reads a matrix from a string in Matlab-like format, for example:
	 *  "[1 0 2; 0 4 -1]"
	 * The string must start with '[' and end with ']'. Rows are separated by
	 * semicolons ';' and columns in each row by one or more whitespaces
	 * ' ', tabs '\t' or commas ','.
	 *
	 * This format is also used for CConfigFile::read_matrix.
	 *
	 * \return true on success. false if the string is malformed, and then the
	 * matrix will be resized to 0x0.
	 * \sa inMatlabFormat, CConfigFile::read_matrix
	 */
	bool fromMatlabStringFormat(
		const std::string& s,
		mrpt::optional_ref<std::ostream> dump_errors_here = std::nullopt);

	/** Exports the matrix as a string compatible with Matlab/Octave.
	 * \sa fromMatlabStringFormat()
	 */
	std::string inMatlabFormat(const std::size_t decimal_digits = 6) const;

	/** Saves the vector/matrix to a file compatible with MATLAB/Octave
	 * text format.
	 * \param file The target filename.
	 * \param fileFormat See TMatrixTextFileFormat. The format of the numbers in
	 * the text file.
	 * \param appendMRPTHeader Insert this header to the file "% File generated
	 * by MRPT. Load with MATLAB with: VAR=load(FILENAME);"
	 * \param userHeader Additional text to be written at the head of the file.
	 * Typically MALAB comments "% This file blah blah". Final end-of-line is
	 * not needed. \sa loadFromTextFile, CMatrixDynamic::inMatlabFormat,
	 * SAVE_MATRIX
	 */
	void saveToTextFile(
		const std::string& file,
		mrpt::math::TMatrixTextFileFormat fileFormat =
			mrpt::math::MATRIX_FORMAT_ENG,
		bool appendMRPTHeader = false,
		const std::string& userHeader = std::string()) const;

	/** Loads a vector/matrix from a text file, compatible with MATLAB text
	 * format. Lines starting with '%' or '#' are interpreted as comments and
	 * ignored.
	 * \exception std::runtime_error On format error.
	 * \sa saveToTextFile, fromMatlabStringFormat
	 */
	void loadFromTextFile(std::istream& f);

	/// \overload
	void loadFromTextFile(const std::string& file);

	template <typename OTHERMATVEC>
	bool operator==(const OTHERMATVEC& o) const
	{
		const auto& d = mvbDerived();
		if (d.cols() != o.cols() || d.rows() != o.rows()) return false;
		for (typename OTHERMATVEC::Index r = 0; r < d.rows(); r++)
			for (typename OTHERMATVEC::Index c = 0; c < d.cols(); c++)
				if (d(r, c) != o(r, c)) return false;
		return true;
	}
	template <typename OTHERMATVEC>
	bool operator!=(const OTHERMATVEC& o) const
	{
		return !(*this == o);
	}

	/** @} */
};

/** Issues a static_assert() error if trying to compile a method that
 * requires Eigen headers, without including them. */
template <typename DER>
void internalAssertEigenDefined()
{
	if constexpr (!mrpt::is_defined_v<typename DER::eigen_t>)
	{
		static_assert(
			mrpt::is_defined_v<typename DER::eigen_t>,
			"Using this method requires including `<Eigen/Dense>` in the "
			"calling C++ file");
	}
}

/** Stream as text. Implemented for all matrices and vectors, except for
 * non-square fixed-size matrices. */
template <
	typename Scalar, class Derived,
	typename = std::enable_if_t<
		Derived::RowsAtCompileTime == Derived::ColsAtCompileTime ||
		(Derived::ColsAtCompileTime == 1)>>
std::ostream& operator<<(
	std::ostream& o, const MatrixVectorBase<Scalar, Derived>& m)
{
	return o << m.asStr();
}

}  // namespace mrpt::math
