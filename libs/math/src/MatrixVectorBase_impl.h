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
#include <mrpt/math/MatrixVectorBase.h>
#include <cstdint>
#include <cstdio>  // fopen(),...
#include <ctime>  // time(),...
#include <fstream>  // ifstream
#include <sstream>  // stringstream
#include <stdexcept>
#include <vector>

namespace mrpt::math
{
template <typename Scalar, class Derived>
bool MatrixVectorBase<Scalar, Derived>::fromMatlabStringFormat(
	const std::string& s, mrpt::optional_ref<std::ostream> dump_errors_here)
{
	// Start with a (0,0) matrix:
	if (Derived::RowsAtCompileTime == Eigen::Dynamic) mvbDerived().resize(0, 0);

	// Look for starting "[".
	size_t ini = s.find_first_not_of(" \t\r\n");
	if (ini == std::string::npos || s[ini] != '[')
	{
		return false;
	}

	size_t end = s.find_last_not_of(" \t\r\n");
	if (end == std::string::npos || s[end] != ']') return false;

	if (ini > end) return false;

	std::vector<Scalar> lstElements;

	size_t i = ini + 1;
	size_t nRow = 0;

	while (i < end)
	{
		// Extract one row:
		size_t end_row = s.find_first_of(";]", i);
		if (end_row == std::string::npos)
		{
			return false;
		}

		// We have one row in s[ i : (end_row-1) ]
		std::stringstream ss(s.substr(i, end_row - i));
		lstElements.clear();
		try
		{
			while (!ss.eof())
			{
				Scalar val;
				ss >> val;
				if (ss.bad() || ss.fail()) break;
				lstElements.push_back(val);
			}
		}
		catch (...)
		{
		}  // end of line

		// Empty row? Only for the first row, then this is an empty matrix:
		if (lstElements.empty())
		{
			if (nRow > 0)
				return false;
			else
			{
				// Else, this may be an empty matrix... if there is no next row,
				// we'll return with a (0,0) matrix
				if (Derived::RowsAtCompileTime == Eigen::Dynamic)
					mvbDerived() = Derived();
			}
		}
		else
		{
			const size_t N = lstElements.size();

			// Check valid width: All rows must have the same width
			if ((nRow > 0 && size_t(mvbDerived().cols()) != N) ||
				(nRow == 0 && Derived::ColsAtCompileTime != Eigen::Dynamic &&
				 Derived::ColsAtCompileTime != int(N)))
			{
				if (dump_errors_here)
					dump_errors_here->get()
						<< "[fromMatlabStringFormat] Row " << nRow + 1
						<< " has invalid number of columns.\n";
				return false;
			}

			// Append to the matrix:
			if (Derived::RowsAtCompileTime == Eigen::Dynamic ||
				Derived::ColsAtCompileTime == Eigen::Dynamic)
				mvbDerived().resize(nRow + 1, N);
			else if (
				Derived::RowsAtCompileTime != Eigen::Dynamic &&
				int(nRow) >= Derived::RowsAtCompileTime)
			{
				if (dump_errors_here)
					dump_errors_here->get()
						<< "[fromMatlabStringFormat] Read more "
						   "rows than the capacity of the "
						   "fixed sized matrix.\n";
				return false;
			}
			for (size_t q = 0; q < N; q++)
				mvbDerived()(nRow, q) = lstElements[q];
			// Go for the next row:
			nRow++;
		}
		i = end_row + 1;
	}
	// For fixed sized matrices, check size:
	if (Derived::RowsAtCompileTime != Eigen::Dynamic &&
		int(nRow) != Derived::RowsAtCompileTime)
	{
		if (dump_errors_here)
			dump_errors_here->get()
				<< "[fromMatlabStringFormat] Read less rows "
				   "than the capacity of the fixed sized "
				   "matrix.\n";
		return false;
	}
	return true;  // Ok
}

template <typename Scalar, class Derived>
std::string MatrixVectorBase<Scalar, Derived>::inMatlabFormat(
	const std::size_t decimal_digits) const
{
	using Index = typename Derived::Index;
	std::stringstream s;
	s << "[" << std::scientific;
	s.precision(decimal_digits);
	for (Index i = 0; i < mvbDerived().rows(); i++)
	{
		for (Index j = 0; j < mvbDerived().cols(); j++)
			s << mvbDerived().coeff(i, j) << " ";
		if (i < mvbDerived().rows() - 1) s << ";";
	}
	s << "]";
	return s.str();
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::saveToTextFile(
	const std::string& file, mrpt::math::TMatrixTextFileFormat fileFormat,
	bool appendMRPTHeader, const std::string& userHeader) const
{
	using Index = typename Derived::Index;
	// Use a secure version in Visual Studio 2005+
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
	FILE* f;
	if (0 != ::fopen_s(&f, file.c_str(), "wt")) f = nullptr;
#else
	FILE* f = ::fopen(file.c_str(), "wt");
#endif
	if (!f)
		throw std::runtime_error(
			std::string("saveToTextFile: Error opening file ") + file +
			std::string("' for writing a matrix as text."));

	if (!userHeader.empty()) fprintf(f, "%s", userHeader.c_str());

	if (appendMRPTHeader)
	{
		time_t rawtime;
		::time(&rawtime);
		// Use a secure version in Visual Studio 2005+
#if defined(_MSC_VER) && (_MSC_VER >= 1400)
		struct tm timeinfo_data;
		struct tm* timeinfo;
		if (0 != ::localtime_s(&timeinfo_data, &rawtime))
			timeinfo = nullptr;
		else
			timeinfo = &timeinfo_data;
#else
		struct tm* timeinfo = ::localtime(&rawtime);
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
		char strTimeBuf[100];
		if (0 != asctime_s(strTimeBuf, sizeof(strTimeBuf), timeinfo))
			strTimeBuf[0] = '\0';
		char* strTime = &strTimeBuf[0];
#else
		char* strTime = asctime(timeinfo);
#endif
		fprintf(
			f,
			"%% File generated with mrpt-math at %s\n"
			"%%------------------------------------\n",
			strTime);
	}

	const auto& m = mvbDerived();
	for (Index i = 0; i < m.rows(); i++)
	{
		for (Index j = 0; j < m.cols(); j++)
		{
			switch (fileFormat)
			{
				case mrpt::math::MATRIX_FORMAT_ENG:
					::fprintf(f, "%.16e", static_cast<double>(m(i, j)));
					break;
				case mrpt::math::MATRIX_FORMAT_FIXED:
					::fprintf(f, "%.16f", static_cast<double>(m(i, j)));
					break;
				case mrpt::math::MATRIX_FORMAT_INT:
					::fprintf(f, "%i", static_cast<int>(m(i, j)));
					break;
				default:
					throw std::runtime_error(
						"Unsupported value for the parameter 'fileFormat'!");
			};
			// Separating blank space
			if (j < (mvbDerived().cols() - 1)) ::fprintf(f, " ");
		}
		::fprintf(f, "\n");
	}
	::fclose(f);
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::loadFromTextFile(std::istream& f)
{
	using Index = typename Derived::Index;
	std::string str;
	std::vector<double> fil(512);
	size_t nRows = 0;
	while (!f.eof() && !f.fail())
	{
		std::getline(f, str);
		if (str.size() && str[0] != '#' && str[0] != '%')
		{
			// Parse row to floats:
			const char* ptr = str.c_str();
			char* ptrEnd = nullptr;
			size_t i = 0;
			// Process each number in this row:
			while (ptr[0] && ptr != ptrEnd)
			{
				// Find next number: (non white-space character):
				while (ptr[0] &&
					   (ptr[0] == ' ' || ptr[0] == ',' || ptr[0] == '\t' ||
						ptr[0] == '\r' || ptr[0] == '\n'))
					ptr++;
				if (fil.size() <= i) fil.resize(fil.size() + (fil.size() >> 1));
				// Convert to "double":
				fil[i] = strtod(ptr, &ptrEnd);
				// A valid conversion has been done?
				if (ptr != ptrEnd)
				{
					i++;  // Yes
					ptr = ptrEnd;
					ptrEnd = nullptr;
				}
			};  // end while procesing this row

			if (!i && nRows == 0)
				throw std::runtime_error("loadFromTextFile: Empty first line!");

			// "i": # of columns:
			if ((Derived::ColsAtCompileTime != Eigen::Dynamic &&
				 Index(i) != Derived::ColsAtCompileTime))
				throw std::runtime_error(
					"loadFromTextFile: The matrix in the text file does not "
					"match fixed matrix size");
			if (Derived::ColsAtCompileTime == Eigen::Dynamic && nRows > 0 &&
				Index(i) != mvbDerived().cols())
				throw std::runtime_error(
					"loadFromTextFile: The matrix in the text file does not "
					"have the same number of columns in all rows");

			// Append to the matrix:
			if (Derived::RowsAtCompileTime == Eigen::Dynamic ||
				Derived::ColsAtCompileTime == Eigen::Dynamic)
			{
				if (mvbDerived().rows() < static_cast<int>(nRows + 1) ||
					mvbDerived().cols() < static_cast<int>(i))
				{
					const size_t extra_rows =
						std::max(static_cast<size_t>(1), nRows >> 1);
					mvbDerived().resize(nRows + extra_rows, i);
				}
			}
			else if (
				Derived::RowsAtCompileTime != Eigen::Dynamic &&
				int(nRows) >= Derived::RowsAtCompileTime)
				throw std::runtime_error(
					"loadFromTextFile: Read more rows than the capacity of the "
					"fixed sized matrix.");

			for (size_t q = 0; q < i; q++)
				mvbDerived()(nRows, q) = Scalar(fil[q]);

			nRows++;
		}  // end if fgets
	}  // end while not feof

	// Final resize to the real size (in case we allocated space in advance):
	if (Derived::RowsAtCompileTime == Eigen::Dynamic ||
		Derived::ColsAtCompileTime == Eigen::Dynamic)
		mvbDerived().resize(nRows, mvbDerived().cols());

	// Report error as exception
	if (!nRows)
		throw std::runtime_error(
			"loadFromTextFile: Error loading from text file");
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::loadFromTextFile(
	const std::string& file)
{
	std::ifstream f(file.c_str());
	if (f.fail())
		throw std::runtime_error(
			std::string("loadFromTextFile: can't open file:") + file);
	loadFromTextFile(f);
}

template <typename Scalar, class Derived>
std::string MatrixVectorBase<Scalar, Derived>::asString() const
{
	std::stringstream ss;
	ss << mvbDerived().asEigen();
	return ss.str();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::sum() const
{
	return mvbDerived().asEigen().array().sum();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::sum_abs() const
{
	return mvbDerived().asEigen().array().abs().sum();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::minCoeff() const
{
	return mvbDerived().asEigen().minCoeff();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::maxCoeff() const
{
	return mvbDerived().asEigen().maxCoeff();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::minCoeff(std::size_t& outIdx) const
{
	if constexpr (Derived::ColsAtCompileTime == 1)
	{
		typename Derived::Index idx;
		auto r = mvbDerived().asEigen().minCoeff(&idx);
		outIdx = static_cast<std::size_t>(idx);
		return r;
	}
	else
		throw std::runtime_error(
			"minCoeff(idx): Signature only valid for column vectors");
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::maxCoeff(std::size_t& outIdx) const
{
	if constexpr (Derived::ColsAtCompileTime == 1)
	{
		typename Derived::Index idx;
		auto r = mvbDerived().asEigen().maxCoeff(&idx);
		outIdx = static_cast<std::size_t>(idx);
		return r;
	}
	else
		throw std::runtime_error(
			"minCoeff(idx): Signature only valid for column vectors");
}
template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::minCoeff(
	std::size_t& rowIdx, std::size_t& colIdx) const
{
	typename Derived::Index row, col;
	auto r = mvbDerived().asEigen().minCoeff(&row, &col);
	rowIdx = static_cast<std::size_t>(row);
	colIdx = static_cast<std::size_t>(col);
	return r;
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::maxCoeff(
	std::size_t& rowIdx, std::size_t& colIdx) const
{
	typename Derived::Index row, col;
	auto r = mvbDerived().asEigen().maxCoeff(&row, &col);
	rowIdx = static_cast<std::size_t>(row);
	colIdx = static_cast<std::size_t>(col);
	return r;
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::operator+=(Scalar s)
{
	mvbDerived().asEigen().array() += s;
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::operator-=(Scalar s)
{
	mvbDerived().asEigen().array() -= s;
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::operator*=(Scalar s)
{
	mvbDerived().asEigen().array() *= s;
}

template <typename Scalar, class Derived>
CMatrixDynamic<Scalar> MatrixVectorBase<Scalar, Derived>::operator*(
	const CMatrixDynamic<Scalar>& v)
{
	return CMatrixDynamic<Scalar>(
		(mvbDerived().asEigen() * v.asEigen()).eval());
}

template <typename Scalar, class Derived>
Derived MatrixVectorBase<Scalar, Derived>::impl_op_add(const Derived& m2) const
{
	Derived ret(mvbDerived().rows(), mvbDerived().cols());
	ret.asEigen() = mvbDerived().asEigen() + m2.asEigen();
	return ret;
}
template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::impl_op_selfadd(const Derived& m2)
{
	mvbDerived().asEigen() += m2.asEigen();
}
template <typename Scalar, class Derived>
Derived MatrixVectorBase<Scalar, Derived>::impl_op_subs(const Derived& m2) const
{
	Derived ret(mvbDerived().rows(), mvbDerived().cols());
	ret.asEigen() = mvbDerived().asEigen() - m2.asEigen();
	return ret;
}
template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::impl_op_selfsubs(const Derived& m2)
{
	mvbDerived().asEigen() -= m2.asEigen();
}
template <typename Scalar, class Derived>
Derived MatrixVectorBase<Scalar, Derived>::operator*(const Derived& m2) const
{
	ASSERTMSG_(
		mvbDerived().cols() == mvbDerived().rows(),
		"Operator* implemented only for square matrices. Use `A.asEigen() * "
		"B.asEigen()` for general matrix products.");
	Derived ret(mvbDerived().rows(), mvbDerived().rows());
	if constexpr (Derived::RowsAtCompileTime == Derived::ColsAtCompileTime)
	{
		ret.asEigen() = mvbDerived().asEigen() * m2.asEigen();
	}
	return ret;
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::matProductOf_Ab(
	const CMatrixDynamic<Scalar>& A, const CVectorDynamic<Scalar>& b)
{
	mvbDerived() = A.asEigen() * b.asEigen();
}

template <typename Scalar, class Derived>
void MatrixVectorBase<Scalar, Derived>::matProductOf_Atb(
	const CMatrixDynamic<Scalar>& A, const CVectorDynamic<Scalar>& b)
{
	mvbDerived() = A.asEigen().transpose() * b.asEigen();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::norm_inf() const
{
	return mvbDerived().asEigen().template lpNorm<Eigen::Infinity>();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::norm() const
{
	return mvbDerived().asEigen().norm();
}

template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::dot(
	const CVectorDynamic<Scalar>& v) const
{
	if constexpr (Derived::ColsAtCompileTime == 1)
	{
		return mvbDerived().asEigen().dot(v.mvbDerived().asEigen());
	}
	else
	{
		ASSERTMSG_(false, "dot(): Implemented for column vectors only.");
	}
}
template <typename Scalar, class Derived>
Scalar MatrixVectorBase<Scalar, Derived>::dot(
	const MatrixVectorBase<Scalar, Derived>& v) const
{
	if constexpr (Derived::ColsAtCompileTime == 1)
	{
		return mvbDerived().asEigen().dot(v.mvbDerived().asEigen());
	}
	else
	{
		ASSERTMSG_(false, "dot(): Implemented for column vectors only.");
	}
}

}  // namespace mrpt::math
