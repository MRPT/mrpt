/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_EIGEN_PLUGINS_IMPL_H
#define MRPT_EIGEN_PLUGINS_IMPL_H

// -------------------------------------------------------------------------
//  This file implements some templates which had to be left only declared
//   in the "plug-in" headers "eigen_plugins.h" within Eigen::MatrixBase<>
// -------------------------------------------------------------------------

namespace internal_mrpt
{
	// Generic version for all kind of matrices:
	template<int R, int C>
	struct MatOrVecResizer
	{
		template <typename S, int Opt, int MaxR, int MaxC>
		static inline void doit(Eigen::Matrix<S,R,C,Opt,MaxR,MaxC> &mat, size_t new_rows,size_t new_cols)
		{
			::mrpt::math::detail::TAuxResizer<Eigen::Matrix<S,R,C,Opt,MaxR,MaxC>,Eigen::Matrix<S,R,C,Opt,MaxR,MaxC>::SizeAtCompileTime>::internal_resize(mat,new_rows,new_cols);
			//mat.derived().conservativeResize(new_rows,new_cols);
		}
	};
	// Specialization for column matrices:
	template<int R>
	struct MatOrVecResizer<R,1>
	{
		template <typename S, int Opt, int MaxR, int MaxC>
		static inline void doit(Eigen::Matrix<S,R,1,Opt,MaxR,MaxC> &mat, size_t new_rows,size_t new_cols)
		{
			::mrpt::math::detail::TAuxResizer<Eigen::Matrix<S,R,1,Opt,MaxR,MaxC>,Eigen::Matrix<S,R,1,Opt,MaxR,MaxC>::SizeAtCompileTime>::internal_resize(mat,new_rows);
			//mat.derived().conservativeResize(new_rows);
		}
	};
	// Specialization for row matrices:
	template<int C>
	struct MatOrVecResizer<1,C>
	{
		template <typename S, int Opt, int MaxR, int MaxC>
		static inline void doit(Eigen::Matrix<S,1,C,Opt,MaxR,MaxC> &mat, size_t new_rows,size_t new_cols)
		{
			::mrpt::math::detail::TAuxResizer<Eigen::Matrix<S,1,C,Opt,MaxR,MaxC>,Eigen::Matrix<S,1,C,Opt,MaxR,MaxC>::SizeAtCompileTime>::internal_resize(mat,new_cols);
			//mat.derived().conservativeResize(new_cols);
		}
	};
	template<>
	struct MatOrVecResizer<1,1>
	{
		template <typename S, int Opt, int MaxR, int MaxC>
		static inline void doit(Eigen::Matrix<S,1,1,Opt,MaxR,MaxC> &mat, size_t new_rows,size_t new_cols)
		{
			::mrpt::math::detail::TAuxResizer<Eigen::Matrix<S,1,1,Opt,MaxR,MaxC>,Eigen::Matrix<S,1,1,Opt,MaxR,MaxC>::SizeAtCompileTime>::internal_resize(mat,new_cols);
			//mat.derived().conservativeResize(new_cols);
		}
	};
}

/** Compute the eigenvectors and eigenvalues, both returned as matrices: eigenvectors are the columns, and eigenvalues
  */
template <class Derived>
template <class MATRIX1,class MATRIX2>
EIGEN_STRONG_INLINE void Eigen::MatrixBase<Derived>::eigenVectors( MATRIX1 & eVecs, MATRIX2 & eVals ) const
{
	Matrix<Scalar,Dynamic,1> evals;
	eigenVectorsVec(eVecs,evals);
	eVals.resize(evals.size(),evals.size());
	eVals.setZero();
	eVals.diagonal()=evals;
}

/** Compute the eigenvectors and eigenvalues, both returned as matrices: eigenvectors are the columns, and eigenvalues
  */
template <class Derived>
template <class MATRIX1,class VECTOR1>
EIGEN_STRONG_INLINE void Eigen::MatrixBase<Derived>::eigenVectorsVec( MATRIX1 & eVecs, VECTOR1 & eVals ) const
{
	Eigen::EigenSolver< Derived > es(*this, true);
	eVecs = es.eigenvectors().real(); // Keep only the real part of complex matrix
	eVals = es.eigenvalues().real(); // Keep only the real part of complex matrix

	// Sort by ascending eigenvalues:
	std::vector<std::pair<Scalar,Index> > D;
	D.reserve(eVals.size());
	for (Index i=0;i<eVals.size();i++)
		D.push_back(std::make_pair<Scalar,Index>(eVals.coeff(i,0),i));
	std::sort(D.begin(),D.end());
	MATRIX1 sortedEigs;
	sortedEigs.resizeLike(eVecs);
	for (int i=0;i<eVals.size();i++)
	{
		eVals.coeffRef(i,0)=D[i].first;
		sortedEigs.col(i)=eVecs.col(D[i].second);
	}
	eVecs = sortedEigs;
}

/** Compute the eigenvectors and eigenvalues, both returned as matrices: eigenvectors are the columns, and eigenvalues
  */
template <class Derived>
template <class MATRIX1,class MATRIX2>
EIGEN_STRONG_INLINE void Eigen::MatrixBase<Derived>::eigenVectorsSymmetric( MATRIX1 & eVecs, MATRIX2 & eVals ) const
{
	Matrix<Scalar,Dynamic,1> evals;
	eigenVectorsSymmetricVec(eVecs,evals);
	eVals.resize(evals.size(),evals.size());
	eVals.setZero();
	eVals.diagonal()=evals;
}

/** Compute the eigenvectors and eigenvalues, both returned as matrices: eigenvectors are the columns, and eigenvalues
  */
template <class Derived>
template <class MATRIX1,class VECTOR1>
EIGEN_STRONG_INLINE void Eigen::MatrixBase<Derived>::eigenVectorsSymmetricVec( MATRIX1 & eVecs, VECTOR1 & eVals ) const
{
	// This solver returns the eigenvectors already sorted.
	Eigen::SelfAdjointEigenSolver<Derived> eigensolver(*this);
	eVecs = eigensolver.eigenvectors();
	eVals = eigensolver.eigenvalues();
}


template <class Derived>
bool Eigen::MatrixBase<Derived>::fromMatlabStringFormat(const std::string &s, bool dumpErrorMsgToStdErr)
{
	// Start with a (0,0) matrix:
	if ( Derived::RowsAtCompileTime==Eigen::Dynamic)
		(*this) = Derived();

	// Look for starting "[".
	size_t  ini = s.find_first_not_of(" \t\r\n");
	if (ini==std::string::npos || s[ini]!='[') { return false; }

	size_t  end = s.find_last_not_of(" \t\r\n");
	if (end==std::string::npos || s[end]!=']') return false;

	if (ini>end) return false;

	std::vector<Scalar> lstElements;

	size_t i = ini+1;
	size_t nRow = 0;

	while (i<end)
	{
		// Extract one row:
		size_t end_row = s.find_first_of(";]",i);
		if (end_row==std::string::npos) { return false; }

		// We have one row in s[ i : (end_row-1) ]
		std::stringstream  ss(s.substr(i, end_row-i ));
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
		} catch (...) { }  // end of line

		// Empty row? Only for the first row, then this is an empty matrix:
		if (lstElements.empty())
		{
			if (nRow>0)
				return false;
			else
			{
				// Else, this may be an empty matrix... if there is no next row, we'll return with a (0,0) matrix
				if ( Derived::RowsAtCompileTime==Eigen::Dynamic )
					(*this) = Derived();
			}
		}
		else
		{
			const size_t N = lstElements.size();

			// Check valid width: All rows must have the same width
			if ((nRow>0 && size_t(cols())!=N) ||
				(nRow==0 && Derived::ColsAtCompileTime!=Eigen::Dynamic && Derived::ColsAtCompileTime!=int(N)) )
			{
				if (dumpErrorMsgToStdErr)
					std::cerr << "[fromMatlabStringFormat] Row " << nRow+1 << " has invalid number of columns.\n";
				return false;
			}

			// Append to the matrix:
			if ( Derived::RowsAtCompileTime==Eigen::Dynamic || Derived::ColsAtCompileTime==Eigen::Dynamic )
				internal_mrpt::MatOrVecResizer<Derived::RowsAtCompileTime,Derived::ColsAtCompileTime>::doit(derived(),nRow+1,N);
			else if (Derived::RowsAtCompileTime!=Eigen::Dynamic && int(nRow)>=Derived::RowsAtCompileTime)
			{
				if (dumpErrorMsgToStdErr)
					std::cerr << "[fromMatlabStringFormat] Read more rows than the capacity of the fixed sized matrix.\n";
				return false;
			}

			for (size_t q=0;q<N;q++)
				coeffRef(nRow,q) = lstElements[q];

			// Go for the next row:
			nRow++;
		}

		i = end_row+1;
	}
	// For fixed sized matrices, check size:
	if (Derived::RowsAtCompileTime!=Eigen::Dynamic && int(nRow)!=Derived::RowsAtCompileTime)
	{
		if (dumpErrorMsgToStdErr)
			std::cerr << "[fromMatlabStringFormat] Read less rows than the capacity of the fixed sized matrix.\n";
		return false;
	}
	return true; // Ok
}

template <class Derived>
std::string  Eigen::MatrixBase<Derived>::inMatlabFormat(const size_t decimal_digits) const
{
	std::stringstream  s;
	s << "[" << std::scientific;
	s.precision(decimal_digits);
	for (Index i=0;i<rows();i++)
	{
		for (Index j=0;j<cols();j++)
			s << coeff(i,j) << " ";
		if (i<rows()-1)	s << ";";
	}
	s << "]";
	return s.str();
}

template <class Derived>
void Eigen::MatrixBase<Derived>::saveToTextFile(
	const std::string &file,
	mrpt::math::TMatrixTextFileFormat fileFormat,
	bool    appendMRPTHeader,
	const std::string &userHeader
	) const
{
	FILE *f= ::fopen(file.c_str(),"wt");
	if (!f)
		throw std::runtime_error(std::string("saveToTextFile: Error opening file ")+file+std::string("' for writing a matrix as text."));

	if (!userHeader.empty())
		fprintf(f,"%s",userHeader.c_str() );

	if (appendMRPTHeader)
	{
		time_t rawtime;
		::time(&rawtime);
		struct tm * timeinfo = ::localtime(&rawtime);

		fprintf(f,"%% File generated with MRPT %s at %s\n%%-----------------------------------------------------------------\n",
			mrpt::system::MRPT_getVersion().c_str(),
			asctime(timeinfo) );
	}

	for (Index i=0; i < rows(); i++)
	{
		for (Index j=0; j < cols(); j++)
		{
			switch(fileFormat)
			{
			case mrpt::math::MATRIX_FORMAT_ENG: ::fprintf(f,"%.16e",static_cast<double>(coeff(i,j))); break;
			case mrpt::math::MATRIX_FORMAT_FIXED: ::fprintf(f,"%.16f",static_cast<double>(coeff(i,j))); break;
			case mrpt::math::MATRIX_FORMAT_INT: ::fprintf(f,"%i",static_cast<int>(coeff(i,j))); break;
			default:
				throw std::runtime_error("Unsupported value for the parameter 'fileFormat'!");
			};
			// Separating blank space
			if (j<(cols()-1)) ::fprintf(f," ");
		}
		::fprintf(f,"\n");
	}
	::fclose(f);
}


template <class Derived>
void Eigen::MatrixBase<Derived>::loadFromTextFile(const std::string &file)
{
	std::ifstream f(file.c_str());
	if (f.fail()) throw std::runtime_error(std::string("loadFromTextFile: can't open file:") + file);
	loadFromTextFile(f);
}

template <class Derived>
void Eigen::MatrixBase<Derived>::loadFromTextFile(std::istream &f)
{
	// This matrix is NROWS x NCOLS
	std::string		str;
	std::vector<double>	fil(512);
	size_t	nRows = 0;
	while ( !f.eof() )
	{
		std::getline(f,str);
		if (str.size() && str[0]!='#' && str[0]!='%')
		{
			// Parse row to floats:
			const char *ptr = str.c_str();
			char *ptrEnd = NULL;
			size_t i=0;
			// Process each number in this row:
			while ( ptr[0] && ptr!=ptrEnd )
			{
				// Find next number: (non white-space character):
				while (ptr[0] && (ptr[0]==' ' || ptr[0]=='\t' || ptr[0]=='\r' || ptr[0]=='\n'))
					ptr++;
				if (fil.size()<=i)	fil.resize(fil.size()+512);
				// Convert to "double":
				fil[i] = strtod(ptr,&ptrEnd);
				// A valid conversion has been done?
				if (ptr!=ptrEnd)
				{
					i++;	// Yes
					ptr = ptrEnd;
					ptrEnd = NULL;
				}
			}; // end while procesing this row

			// "i": # of columns:
			if ((Derived::ColsAtCompileTime!=Eigen::Dynamic && Index(i)!=Derived::ColsAtCompileTime) )
				throw std::runtime_error("loadFromTextFile: The matrix in the text file does not match fixed matrix size");
			if (Derived::ColsAtCompileTime==Eigen::Dynamic && nRows>0 && Index(i)!=cols() )
				throw std::runtime_error("loadFromTextFile: The matrix in the text file does not have the same number of columns in all rows");

			// Append to the matrix:
			if ( Derived::RowsAtCompileTime==Eigen::Dynamic || Derived::ColsAtCompileTime==Eigen::Dynamic )
				internal_mrpt::MatOrVecResizer<Derived::RowsAtCompileTime,Derived::ColsAtCompileTime>::doit(derived(),nRows+1,i);
			else if (Derived::RowsAtCompileTime!=Eigen::Dynamic && int(nRows)>=Derived::RowsAtCompileTime)
				throw std::runtime_error("loadFromTextFile: Read more rows than the capacity of the fixed sized matrix.");

			for (size_t q=0;q<i;q++)
				coeffRef(nRows,q) = Scalar(fil[q]);

			nRows++;
		} // end if fgets
	} // end while not feof

	// Report error as exception
	if (!nRows)
		throw std::runtime_error("loadFromTextFile: Error loading from text file");
}


#endif // guard define
