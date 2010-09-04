/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef  mrpt_math_matrix_ops_H
#define  mrpt_math_matrix_ops_H

#include <mrpt/math/math_frwds.h>  // Fordward declarations

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CVectorTemplate.h>

#include <mrpt/math/ops_containers.h>		// Many generic operations
#include <mrpt/math/ops_matrices_eigen.h>	// Eigenvectors are apart because it's a mess of code...
#include <mrpt/math/matrices_metaprogramming.h>  // TMatrixProductType, ...

#include <mrpt/utils/metaprogramming.h>  // for copy_container_typecasting()


/** \file ops_matrices.h
  * This file implements miscelaneous matrix and matrix/vector operations, plus internal functions in mrpt::math::detail
  *
  */

namespace mrpt
{
	namespace math
	{
		/** Read operator from a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in, CMatrixFixedNumeric<float,NROWS,NCOLS> & M) {
			CMatrix  aux;
			in.ReadObject(&aux);
			ASSERTMSG_(M.size()==aux.size(), format("Size mismatch: deserialized is %ux%u, expected is %ux%u",(unsigned)aux.getRowCount(),(unsigned)aux.getColCount(),(unsigned)NROWS,(unsigned)NCOLS))
			M = aux;
			return in;
		}
		/** Read operator from a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in, CMatrixFixedNumeric<double,NROWS,NCOLS> & M) {
			CMatrixD  aux;
			in.ReadObject(&aux);
			ASSERTMSG_(M.size()==aux.size(), format("Size mismatch: deserialized is %ux%u, expected is %ux%u",(unsigned)aux.getRowCount(),(unsigned)aux.getColCount(),(unsigned)NROWS,(unsigned)NCOLS))
			M = aux;
			return in;
		}

		/** Write operator for writing into a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const CMatrixFixedNumeric<float,NROWS,NCOLS> & M) {
			CMatrix aux = CMatrixFloat(M);
			out.WriteObject(&aux);
			return out;
		}
		/** Write operator for writing into a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const CMatrixFixedNumeric<double,NROWS,NCOLS> & M) {
			CMatrixD aux = CMatrixDouble(M);
			out.WriteObject(&aux);
			return out;
		}

		/** Equal comparison (==)  */
		template <class T,size_t NROWS, size_t NCOLS>
		bool operator == (const CMatrixFixedNumeric<T,NROWS,NCOLS>& M1, const CMatrixFixedNumeric<T,NROWS,NCOLS>& M2)
		{
			for (size_t i=0; i < NROWS; i++)
				for (size_t j=0; j < NCOLS; j++)
					if (M1.get_unsafe(i,j)!=M2.get_unsafe(i,j))
						return false;
			return true;
		}

		/** Textual output stream function.
		  *    Use only for text output, for example:  "std::cout << mat;"
		  */
		template <class MATRIX>
		RET_TYPE_ASSERT_MRPTMATRIX(MATRIX, std::ostream)& // This expands into "std::ostream &"
		operator << (std::ostream& ostrm, const MATRIX& m)
		{
			ostrm << std::setprecision(4);
			for (size_t i=0; i < m.getRowCount(); i++)
			{
				for (size_t j=0; j < m.getColCount(); j++)
					ostrm << std::setw(13) << m(i,j);
				ostrm << std::endl;
			}
			return ostrm;
		}




	// ------ Implementatin of detail functions -------------
	namespace detail
	{
		/** Cholesky factorization: in = out' · out  (Upper triangular version: M=U'*U )
		  *	Given a positive-definite symmetric matrix, this routine constructs its Cholesky decomposition.
		  * On input, only the upper triangle of "IN" need be given; it is not modified.
		  *  The Cholesky factorization is returned in "out" in the upper triangle.
		  *  (by AJOGD @ JAN-2007)
		  * Redone for efficiency (Pablo Moreno, Jan 2010).
		  * \return True on success, false on singular (i.e. det=0) or not positive semidefinite matrix
		  * \exception logic_error On non-square matrix.
		  * \note Complexity O(N^3)
		  */
		template<class MATRIX1,class MATRIX2>
		bool chol(const MATRIX1 &in, MATRIX2 &out)
		{
			typedef typename MATRIX1::value_type T; // Elements type
			const size_t N=in.getRowCount();
			if (in.getColCount()!=in.getRowCount()) throw std::logic_error("Error in Cholesky factorization. Matrix is not square");
			out.setSize(N,N);
			{	//Special case of the main loop: i=0.
				T coef=in.get_unsafe(0,0);
				if (coef<=0) return false;
				else out.get_unsafe(0,0)=(coef=sqrt(coef));
				for (size_t j=1;j<N;++j) out.get_unsafe(0,j)=in.get_unsafe(0,j)/coef;
			}
			T sum;
			for (size_t i=1;i<N;++i)	{
				out.set_unsafe(i,0,0);
				{	//Special case of the "j" loop: j=i.
					sum=in.get_unsafe(i,i);
					for (size_t k=i-1;k<N;--k) sum-=square(out.get_unsafe(k,i));
					if (sum<=0) return false;
					else out.get_unsafe(i,i)=sqrt(sum);
				}
				for (size_t j=i+1;j<N;++j)	{
					sum=in.get_unsafe(i,j);
					for (size_t k=i-1;k<N;--k) sum-=out.get_unsafe(k,i)*out.get_unsafe(k,j);
					out.get_unsafe(i,j)=sum/out.get_unsafe(i,i);
					out.get_unsafe(j,i)=0;
				}
			}
			return true;
		} // end "chol"

		/** Save matrix to a text file, compatible with MATLAB text format (see also the methods of matrix classes themselves).
			* \param theMatrix It can be a CMatrixTemplate or a CMatrixFixedNumeric.
			* \param file The target filename.
			* \param fileFormat See TMatrixTextFileFormat. The format of the numbers in the text file.
			* \param appendMRPTHeader Insert this header to the file "% File generated by MRPT. Load with MATLAB with: VAR=load(FILENAME);"
			* \param userHeader Additional text to be written at the head of the file. Typically MALAB comments "% This file blah blah". Final end-of-line is not needed.
			* \sa loadFromTextFile, CMatrixTemplate::inMatlabFormat, SAVE_MATRIX
			*/
		template <class MAT>
		void saveMatrixToTextFile(
			const MAT &theMatrix,
			const std::string &file,
			TMatrixTextFileFormat fileFormat,
			bool    appendMRPTHeader,
			const std::string &userHeader
			)
		{
			using namespace mrpt::system;

			MRPT_START;

			FILE	*f=os::fopen(file.c_str(),"wt");
			if (!f)
				THROW_EXCEPTION_CUSTOM_MSG1("saveToTextFile: Error opening file '%s' for writing a matrix as text.", file.c_str());

			if (!userHeader.empty())
				fprintf(f,"%s",userHeader.c_str() );

			if (appendMRPTHeader)
				fprintf(f,"%% File generated with MRPT %s at %s\n%%-----------------------------------------------------------------\n",
					mrpt::system::MRPT_getVersion().c_str(),
					mrpt::system::dateTimeLocalToString( mrpt::system::now() ).c_str() );

			for (size_t i=0; i < theMatrix.getRowCount(); i++)
			{
				for (size_t j=0; j < theMatrix.getColCount(); j++)
				{
					switch(fileFormat)
					{
					case MATRIX_FORMAT_ENG: os::fprintf(f,"%.16e",static_cast<double>(theMatrix(i,j))); break;
					case MATRIX_FORMAT_FIXED: os::fprintf(f,"%.16f",static_cast<double>(theMatrix(i,j))); break;
					case MATRIX_FORMAT_INT: os::fprintf(f,"%i",static_cast<int>(theMatrix(i,j))); break;
					default:
						THROW_EXCEPTION("Unsupported value for the parameter 'fileFormat'!");
					};
					// Separating blank space
					if (j<(theMatrix.getColCount()-1)) os::fprintf(f," ");
				}
				os::fprintf(f,"\n");
			}
			os::fclose(f);
			MRPT_END;
		}

		/** Dump matrix in matlab format.
		  *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
		  */
		template <class MATRIX>
		std::string  matrix_inMatlabFormat(const MATRIX &m,const size_t decimal_digits)
		{
			std::stringstream  s;
			s << "[" << std::scientific;
			s.precision(decimal_digits);
			for (size_t i=0;i<m.getRowCount();i++)
			{
				for (size_t j=0;j<m.getColCount();j++)
					s << m.get_unsafe(i,j) << " ";
				if (i<m.getRowCount()-1) s << ";";
			}
			s << "]";
			return s.str();
		}


		/** The trace of a matrix (the sum of its diagonal)
		  */
		template<class MATRIX1>
		typename MATRIX1::value_type trace(const MATRIX1 &m)
		{
			if (!m.IsSquare()) throw std::logic_error("Error in trace: matrix is not square");
			const size_t N = size(m,1);
			typename MATRIX1::value_type ret=0;
			for (size_t i=0;i<N;i++)
				ret+=m.get_unsafe(i,i);
			return ret;
		}

		/** Return the transpose of a matrix */
		template <class MAT1,class MAT2>
		inline void matrix_transpose(const MAT1& m_in, MAT2 &m_out)
		{
			const size_t R=m_in.getRowCount();
			const size_t C=m_in.getColCount();
			m_out.setSize(C,R); // transpose
			for (size_t i=0;i<R;i++)
				for (size_t j=0;j<C;j++)
					m_out.get_unsafe(j,i) = m_in.get_unsafe(i,j);
		}
		//! \overload
		template <class MAT>
		inline MAT_TYPE_TRANSPOSE_OF(MAT) matrix_transpose(const MAT& m)
		{
			MAT_TYPE_TRANSPOSE_OF(MAT) temp;
			matrix_transpose(m,temp);
			return temp;
		}

		/** Only for vectors/arrays "v" of length3, compute out = A * Skew(v), where Skew(v) is the skew symmetric matric generated from \a v (see mrpt::math::skew_symmetric3)
		  */
		template <class MAT_A,class SKEW_3VECTOR,class MAT_OUT>
		void multiply_A_skew3(const MAT_A &A,const SKEW_3VECTOR &v, MAT_OUT &out)
		{
			MRPT_START
			ASSERT_EQUAL_(size(A,2),3)
			ASSERT_EQUAL_(v.size(),3)
			const size_t N = size(A,1);
			out.setSize(N,3);
			for (size_t i=0;i<N;i++)
			{
				out.set_unsafe(i,0, A.get_unsafe(i,1)*v[2]-A.get_unsafe(i,2)*v[1] );
				out.set_unsafe(i,1,-A.get_unsafe(i,0)*v[2]+A.get_unsafe(i,2)*v[0] );
				out.set_unsafe(i,2, A.get_unsafe(i,0)*v[1]-A.get_unsafe(i,1)*v[0] );
			}
			MRPT_END
		}

		/** Only for vectors/arrays "v" of length3, compute out = Skew(v) * A, where Skew(v) is the skew symmetric matric generated from \a v (see mrpt::math::skew_symmetric3)
		  */
		template <class SKEW_3VECTOR,class MAT_A,class MAT_OUT>
		void multiply_skew3_A(const SKEW_3VECTOR &v, const MAT_A &A,MAT_OUT &out)
		{
			MRPT_START
			ASSERT_EQUAL_(size(A,1),3)
			ASSERT_EQUAL_(v.size(),3)
			const size_t N = size(A,2);
			out.setSize(3,N);
			for (size_t i=0;i<N;i++)
			{
				out.set_unsafe(0,i,-A.get_unsafe(1,i)*v[2]+A.get_unsafe(2,i)*v[1] );
				out.set_unsafe(1,i, A.get_unsafe(0,i)*v[2]-A.get_unsafe(2,i)*v[0] );
				out.set_unsafe(2,i,-A.get_unsafe(0,i)*v[1]+A.get_unsafe(1,i)*v[0] );
			}
			MRPT_END
		}

		/** Computes the vector v = A * a, where "a" is a column vector of the appropriate length. */
		template<class MATRIX1, class OTHERVECTOR1,class OTHERVECTOR2>
		void multiply_Ab(const MATRIX1 &m, const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput ) {
			MRPT_START
			size_t N=m.getRowCount();
			size_t M=m.getColCount();
			ASSERT_(vIn.size()==M)
			vOut.resize(N);
			if (!accumToOutput) for (size_t i=0;i<N;++i) vOut[i]=typename OTHERVECTOR2::value_type(0);
			for (size_t i=0;i<N;++i)	{
				typename MATRIX1::value_type &accum=vOut[i];
				for (size_t j=0;j<M;++j) accum+=m.get_unsafe(i,j)*vIn[j];
			}
			MRPT_END
		}

		/** Computes the vector v = A<sup>T</sup> * a, where "a" is a column vector of the appropriate length. */
		template<class MATRIX1, class OTHERVECTOR1,class OTHERVECTOR2>
		void multiply_Atb(const MATRIX1 &m, const OTHERVECTOR1 &vIn,OTHERVECTOR2 &vOut,bool accumToOutput ) {
			MRPT_START
			size_t N=m.getColCount();
			size_t M=m.getRowCount();
			ASSERT_(vIn.size()==M)
			vOut.resize(N);
			if (!accumToOutput) for (size_t i=0;i<N;++i) vOut[i]=typename OTHERVECTOR2::value_type(0);
			for (size_t i=0;i<N;++i)	{
				typename MATRIX1::value_type &accum=vOut[i];
				for (size_t j=0;j<M;++j) accum+=m.get_unsafe(j,i)*vIn[j];
			}
			MRPT_END
		}

		/** RESULT = A * A^t  */
		template <class MATRIX1,class MATRIX2>
		void multiply_AAt( const MATRIX1& m1, MATRIX2& RESULT )
		{
			typedef typename MATRIX2::value_type T;
			const size_t M1R = m1.getRowCount();
			const size_t M1C = m1.getColCount();
			RESULT.setSize(M1R,M1R);

			// If m1 is me, make a copy:
			if ((void*)&m1==(void*)&RESULT)
			{
				// Save result in a temporary matrix:
				std::vector<T>  temp(M1R*M1R); // Passed to vector() due to dynamic size...

				T  *ptr = &temp[0];
				size_t i;
				for (i=0; i < M1R; i++)
				{
					for (size_t j=i; j < M1R; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m1.get_unsafe(j,k);
						*(ptr++) = accum;
					}
				}
				// Copy from temp:
				ptr = &temp[0];
				for (i=0; i < M1R; i++)
					for (size_t j=i; j < M1R; j++)
						RESULT.get_unsafe(i,j) = RESULT.get_unsafe(j,i) = *(ptr++);
			}
			else
			{
				// Work directly over the data:
				for (size_t i=0; i < M1R; i++)
					for (size_t j=i; j < M1R; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m1.get_unsafe(j,k);
						RESULT.get_unsafe(i,j) = RESULT.get_unsafe(j,i) = accum;
					}
			}
		}

		/** RESULT = A^t * A  */
		template <class MATRIX1,class MATRIX2>
		void multiply_AtA( const MATRIX1& m1, MATRIX2& RESULT )
		{
			typedef typename MATRIX2::value_type T;
			const size_t M1R = m1.getRowCount();
			const size_t M1C = m1.getColCount();
			RESULT.setSize(M1C,M1C);

			// If m1 is me, make a copy:
			if ((void*)&m1==(void*)&RESULT)
			{
				// Save result in a temporary matrix:
				std::vector<T>  temp(M1C*M1C); // Passed to vector() due to dynamic size...

				T  *ptr = &temp[0];
				size_t i;
				for (i=0; i < M1C; i++)
				{
					for (size_t j=i; j < M1C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1R; k++)
							accum += m1.get_unsafe(i,k) * m1.get_unsafe(j,k);
						*(ptr++) = accum;
					}
				}
				// Copy from temp:
				ptr = &temp[0];
				for (i=0; i < M1C; i++)
					for (size_t j=i; j < M1C; j++)
						RESULT.get_unsafe(i,j) = RESULT.get_unsafe(j,i) = *(ptr++);
			}
			else
			{
				// Work directly over the data:
				for (size_t i=0; i < M1C; i++)
				{
					for (size_t j=i; j < M1C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1R; k++)
							accum += m1.get_unsafe(k,i) * m1.get_unsafe(k,j);
						RESULT.get_unsafe(i,j) = RESULT.get_unsafe(j,i) = accum;
					}
				}
			}
		}

		/** Multiply 2 matrices: RESULT = A * B  */
		template <class MATRIX1,class MATRIX2,class MATRIXRES>
		void multiply_AB(const MATRIX1& m1,const MATRIX2& m2, MATRIXRES& RESULT )
		{
			MRPT_START

			typedef typename MATRIXRES::value_type T;

			const size_t NROWS = m1.getRowCount();
			const size_t NCOLS = m2.getColCount();
			const size_t M1C   = m1.getColCount();
			ASSERTMSG_(M1C==m2.getRowCount(),format("Invalid matrix sizes in multiplication: %ux%u * %ux%u",(unsigned int)NROWS,(unsigned int)M1C,(unsigned int)m2.getRowCount(),(unsigned int)NCOLS ));

			RESULT.setSize(NROWS,NCOLS);

			// If one of the matrices is me, make a copy:
			if ( (void*)(&m1)==(void*)(&RESULT) || (void*)(&m2)==(void*)&RESULT)
			{
				// Save result in a temporary matrix:
				std::vector<T> temp(NROWS*NCOLS);  // This cannot be a plain array due to unknown size at compile time for all kind of matrices...
				size_t out_idx = 0;
				for (size_t i=0; i < NROWS; i++)
				{
					for (size_t j=0; j < NCOLS; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(k,j);
						temp[out_idx++] = accum;
					}
				}

				// Copy from temp:
				T* ptr = &temp[0];
				for (size_t i=0; i < NROWS; i++)
					for (size_t j=0; j < NCOLS; j++)
						RESULT.get_unsafe(i,j)= *(ptr++);
			}
			else
			{
				// Work directly over the data:
				for (size_t i=0; i < NROWS; i++)
				{
					for (size_t j=0; j < NCOLS; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(k,j);
						RESULT.get_unsafe(i,j)=accum;
					}
				}
			}

			MRPT_END
		} // end multiply

		template<typename MAT_IN_1,typename MAT_IN_2,typename MAT_OUT> void multiply_AtB(const MAT_IN_1 &A,const MAT_IN_2 &B,MAT_OUT &C)	{
			size_t sA=A.getColCount(),sB=B.getColCount();
			size_t N=A.getRowCount();
			ASSERTMSG_(N==B.getRowCount(),format("Invalid matrix sizes in multiplication: %ux%u * %ux%u",(unsigned int)sA,(unsigned int)N,(unsigned int)B.getRowCount(),(unsigned int)sB));
			if (((void *)&A==(void *)&C)||((void *)&B==(void *)&C))	{
				//We can't work directly over C!
				MAT_TYPE_PRODUCT_AtB_OF(MAT_IN_1,MAT_IN_2) tmpMat;
				for (size_t i=0;i<sA;++i) for (size_t j=0;j<sB;++j)	{
					/*
					tmpMat.set_unsafe(i,j,0);
					for (size_t k=0;k<N;++k) tmpMat.get_unsafe(i,j)+=A.get_unsafe(k,i)*B.get_unsafe(k,j);
					*/
					//OK, let's assume that N is NOT zero, which will be virtually ALL cases.
					tmpMat.set_unsafe(i,j,A.get_unsafe(0,i)*B.get_unsafe(0,j));
					for (size_t k=1;k<N;++k) tmpMat.get_unsafe(i,j)+=A.get_unsafe(k,i)*B.get_unsafe(k,j);
				}
				C.setSize(sA,sB);
				for (size_t i=0;i<sA;++i) for (size_t j=0;j<sB;++j) C.set_unsafe(i,j,tmpMat.get_unsafe(i,j));
			}	else	{
				C.setSize(sA,sB);
				for (size_t i=0;i<sA;++i) for (size_t j=0;j<sB;++j)	{
					//As in the previous case, let's assume that N is not zero. Let the degenerate case be handled externally.
					C.set_unsafe(i,j,A.get_unsafe(0,i)*B.get_unsafe(0,j));
					for (size_t k=1;k<N;++k) C.get_unsafe(i,j)+=A.get_unsafe(k,i)*B.get_unsafe(k,j);
				}
			}
		}


		/** R = H * C * H^t (with C symmetric) */
		template <typename MAT_H, typename MAT_C, typename MAT_R>
		void multiply_HCHt(
			const MAT_H &H,
			const MAT_C &C,
			MAT_R &R,
			bool accumResultInOutput,
			bool allow_submatrix_mult)
		{
			MRPT_START

			ASSERTMSG_( (void*)&C != (void*)&H, "C and H must be different matrices." )
			ASSERTMSG_( (void*)&R != (void*)&H, "R and H must be different matrices." )
			ASSERTMSG_( (void*)&C != (void*)&R,  "C and R must be different matrices.")
			ASSERT_(C.IsSquare())

			if (allow_submatrix_mult)
				 ASSERT_(C.getRowCount()>=H.getColCount())
			else ASSERT_(C.getRowCount()==H.getColCount())

			const size_t N=H.getRowCount();
			const size_t M=H.getColCount();

			R.setSize(N,N); // Set output size. For fixed size matrices this does nothing.

			// Create R with the type of MAT_H * MAT_C
			MAT_TYPE_PRODUCT_OF(MAT_H,MAT_C) R_;
			R_.setSize(N,M);

			// First compute R_ = H * C:
			for (size_t i=0;i<N;i++)
				for (size_t j=0;j<M;j++)
				{
					typename MAT_H::value_type sumAccum = 0;
					for (size_t l=0;l<M;l++)
						sumAccum += H.get_unsafe(i,l) * C.get_unsafe(l,j);
					R_.get_unsafe(i,j)  = sumAccum;
				}

			// Now compute R = R_ * (H^t):
			for (size_t i=0;i<N;i++)
				for (size_t j=i;j<N;j++)
				{
					typename MAT_H::value_type sumAccum = accumResultInOutput ? R.get_unsafe(i,j) : 0;
					for (size_t l=0;l<M;l++)
						sumAccum += R_.get_unsafe(i,l) * H.get_unsafe(j,l);
					R.get_unsafe(i,j) = R.get_unsafe(j,i) = sumAccum;
				}
			MRPT_END
		}

		/** r (a scalar) = H * C * H^t (with a vector H and a symmetric matrix C) */
		template <typename VECTOR_H, typename MAT_C>
		typename MAT_C::value_type
		multiply_HCHt_scalar(const VECTOR_H &H, const MAT_C &C)
		{
			MRPT_START
			ASSERT_( C.IsSquare() )

			const size_t M = H.size();

			// This is 1xM matrix, C is a MxM matrix
			ASSERT_( size(C,1)==M );

			typename VECTOR_H::value_type sumAccum = 0;
			typename VECTOR_H::const_iterator itL=H.begin();
			for (size_t l=0;l<M; ++l, ++itL)
			{
				typename VECTOR_H::value_type sumAccumInner = 0;
				typename VECTOR_H::const_iterator it;
				size_t k;
				for (k=0,it=H.begin();it!=H.end();++it,++k)
					sumAccumInner += *it * C.get_unsafe(k,l);
				sumAccum += sumAccumInner * (*itL);
			}
			return sumAccum;
			MRPT_END
		}

		/** R = H^t * C * H  (with C symmetric) */
		template <typename MAT_H, typename MAT_C, typename MAT_R>
		void multiply_HtCH(
			const MAT_H &H,
			const MAT_C &C,
			MAT_R &R,
			bool accumResultInOutput,
			bool allow_submatrix_mult)
		{
			MRPT_START

			ASSERTMSG_( (void*)&C != (void*)&H, "C and H must be different matrices." )
			ASSERTMSG_( (void*)&R != (void*)&H, "R and H must be different matrices." )
			ASSERTMSG_( (void*)&C != (void*)&R,  "C and R must be different matrices.")
			ASSERT_(C.IsSquare())

			if (allow_submatrix_mult)
				 ASSERT_(C.getRowCount()>=H.getRowCount())
			else ASSERT_(C.getRowCount()==H.getRowCount())

			R.setSize( H.getColCount(), H.getColCount()); // Set output size. For fixed size matrices this does nothing.

			MAT_H R_;

			const size_t M=H.getRowCount();
			const size_t N=H.getColCount();

			// First compute R_ = H * C:
			for (size_t i=0;i<N;i++)
				for (size_t j=0;j<M;j++)
				{
					typename MAT_H::value_type sumAccum = 0;
					for (size_t l=0;l<M;l++)
						sumAccum += H.get_unsafe(l,i) * C.get_unsafe(l,j);
					R_.get_unsafe(i,j)  = sumAccum;
				}

			// Now compute R = R_ * (H^t):
			for (size_t i=0;i<N;i++)
				for (size_t j=i;j<N;j++)
				{
					typename MAT_H::value_type sumAccum = accumResultInOutput ? R.get_unsafe(i,j) : 0;
					for (size_t l=0;l<M;l++)
						sumAccum += R_.get_unsafe(i,l) * H.get_unsafe(l,j);
					R.get_unsafe(i,j) = R.get_unsafe(j,i) = sumAccum;
				}
			MRPT_END
		}

		/**
		  * OUT=IN * IN^t * scalar. This is equivalent to IN*(I*scalar)*IN^t, with I
		  * being an identity matrix with a number of rows and columns equal to IN's rows.
		  */
		template<typename MAT_IN,typename MAT_OUT> void multiply_AAt_scalar(const MAT_IN &in,typename MAT_IN::value_type scalar,MAT_OUT &out)	{
			MRPT_START
			ASSERTMSG_((void*)&in!=(void*)&out,"The matrices mustn't be the same." )
			size_t r=in.getRowCount();
			size_t c=in.getColCount();
			out.setSize(r,r);
			typename MAT_IN::value_type accum=typename MAT_IN::value_type(0);
			for (size_t k=0;k<c;++k) accum+=square(in.get_unsafe(0,k));
			out.set_unsafe(0,0,accum*scalar);
			for (size_t i=1;i<r;++i)	{
				for (size_t j=0;j<i;++j)	{
					accum=typename MAT_IN::value_type(0);
					for (size_t k=0;k<c;++k) accum+=in.get_unsafe(i,k)*in.get_unsafe(j,k);
					accum*=scalar;
					out.set_unsafe(i,j,accum);
					out.set_unsafe(j,i,accum);
				}
				accum=typename MAT_IN::value_type(0);
				for (size_t k=0;k<c;++k) accum+=square(in.get_unsafe(i,k));
				out.set_unsafe(i,i,accum*scalar);
			}
			MRPT_END
		}

		/**
		  * OUT=IN^t * IN * scalar. This is equivalent to IN^t*(I*scalar)*IN, with I
		  * being an identity matrix with a number of rows and columns equal to IN's
		  * columns.
		  */
		template<typename MAT_IN,typename MAT_OUT> void multiply_AtA_scalar(const MAT_IN &in,typename MAT_IN::value_type scalar,MAT_OUT &out)	{
			MRPT_START
			ASSERTMSG_((void*)&in!=(void*)&out,"The matrices mustn't be the same." )
			size_t r=in.getRowCount();
			size_t c=in.getColCount();
			out.setSize(c,c);
			typename MAT_IN::value_type accum=typename MAT_IN::value_type(0);
			for (size_t k=0;k<c;++k) accum+=square(in.get_unsafe(k,0));
			out.set_unsafe(0,0,accum*scalar);
			for (size_t i=1;i<r;++i)	{
				for (size_t j=0;j<i;++j)	{
					accum=typename MAT_IN::value_type(0);
					for (size_t k=0;k<c;++k) accum+=in.get_unsafe(k,i)*in.get_unsafe(k,j);
					accum*=scalar;
					out.set_unsafe(i,j,accum);
					out.set_unsafe(j,i,accum);
				}
				accum=typename MAT_IN::value_type(0);
				for (size_t k=0;k<c;++k) accum+=square(in.get_unsafe(k,i));
				out.set_unsafe(i,i,accum*scalar);
			}
			MRPT_END
		}

		/** Matrix multiplication of this matrix with a submatrix of 'A', saving the result in a third matrix.
		  *   OUT = B * A
		  */
		template <class MAT_X,class MAT_A,class MAT_OUT>
		void multiply_subMatrix (
			const MAT_X &X,
			const MAT_A &A,
			MAT_OUT &outResult,
			const size_t A_cols_offset,
			const size_t A_rows_offset,
			const size_t A_col_count)
		{
			MRPT_START
			// The output will be NxM:
			const size_t  N = X.getRowCount();
			const size_t  M = A_col_count;
		#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			ASSERT_( A.getColCount() >= A_col_count + A_cols_offset );
			ASSERT_( A.getRowCount() >= N + A_rows_offset );
		#endif
			outResult.setSize(N,M);
			for (size_t i=0; i < N; i++)
				for (size_t j=0; j < M; j++)
				{
					typename MAT_OUT::value_type tmp = 0;
					for (size_t k=0; k < X.getColCount(); k++)
						tmp += X.get_unsafe(i,k) * A.get_unsafe(k+A_rows_offset,j+A_cols_offset);
					outResult.get_unsafe(i,j) = tmp;
				}
			MRPT_END
		}

		/**	RES = A*B*C   \sa multiply_ABCt */
		template <class MAT_A,class MAT_B,class MAT_C,class MAT_OUT>
		void multiply_ABC(const MAT_A &A, const MAT_B &B, const MAT_C &C, MAT_OUT & RES)
		{
			const size_t NROWS = A.getRowCount(); // A: NROWS x N1
			const size_t N1    = B.getRowCount(); // B: N1 x N2
			const size_t N2    = B.getColCount(); // C: NCOLS x N2
			const size_t NCOLS = C.getColCount();
			ASSERT_( A.getColCount()==B.getRowCount() && B.getColCount()==C.getRowCount() )

			RES.zeros();
			for (size_t i=0;i<NROWS;i++)
				for (size_t l=0;l<N2;l++)
				{
					typename MAT_OUT::value_type sumAccumInner = 0;
					for (size_t k=0;k<N1;k++)
						sumAccumInner += A.get_unsafe(i,k) * B.get_unsafe(k,l);
					for (size_t j=0;j<NCOLS;j++)
						RES.get_unsafe(i,j) += sumAccumInner * C.get_unsafe(l,j);
				}
		}

		/**	RES = A*B*(C^t)   \sa multiply_ABC */
		template <class MAT_A,class MAT_B,class MAT_C,class MAT_OUT>
		void multiply_ABCt(const MAT_A &A, const MAT_B &B, const MAT_C &C, MAT_OUT & RES)
		{
			const size_t NROWS = A.getRowCount(); // A: NROWS x N1
			const size_t N1    = B.getRowCount(); // B: N1 x N2
			const size_t N2    = B.getColCount(); // C: (NCOLS x N2)^t
			const size_t NCOLS = C.getRowCount();
			ASSERT_( A.getColCount()==B.getRowCount() && B.getColCount()==C.getColCount() )

			RES.zeros();
			for (size_t i=0;i<NROWS;i++)
				for (size_t l=0;l<N2;l++)
				{
					typename MAT_OUT::value_type sumAccumInner = 0;
					for (size_t k=0;k<N1;k++)
						sumAccumInner += A.get_unsafe(i,k) * B.get_unsafe(k,l);
					for (size_t j=0;j<NCOLS;j++)
						RES.get_unsafe(i,j) += sumAccumInner * C.get_unsafe(j,l);
				}
		}

		template <class MAT_A,class MAT_B,class MAT_OUT>
		void multiply_ABt(const MAT_A &m1,const MAT_B &m2, MAT_OUT &out)
		{
			MRPT_START
			typedef typename MAT_OUT::value_type T;
			const size_t M1R = m1.getRowCount();
			const size_t M1C = m1.getColCount();
			const size_t M2C = m2.getRowCount();

			ASSERTMSG_(m1.getColCount() == m2.getColCount(),"multiply_ABt: Inconsistent matrix sizes in multiplication!")

			// If one of the matrices is OUT, make a copy:
			if ((void*)&m1==(void*)&out|| (void*)&m2==(void*)&out)
			{
				// Save result in a temporary matrix:
				std::vector<T> temp(M1R*M2C);

				T  *ptr = &temp[0];
				for (size_t i=0; i < M1R; i++)
					for (size_t j=0; j < M2C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(j,k);
						*(ptr++) = accum;
					}
				// Copy from temp:
				out.setSize(M1R,M2C);
				ptr = &temp[0];
				for (size_t i=0; i < M1R; i++)
					for (size_t j=0; j < M2C; j++)
						out.set_unsafe(i,j,  *(ptr++) );
			}
			else
			{
				// Work directly over the data:
				out.setSize( M1R,M2C );
				for (size_t i=0; i < M1R; i++)
					for (size_t j=0; j < M2C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(j,k); //(k,j);
						out.set_unsafe(i,j,accum);
					}
			}
			MRPT_END
		}

		template <class MAT_A,class MAT_B,class MAT_OUT>
		void multiply_result_is_symmetric(const MAT_A &m1,const MAT_B &m2, MAT_OUT &out)
		{
			MRPT_START
			typedef typename MAT_OUT::value_type T;
			const size_t M1R = m1.getRowCount();
			const size_t M1C = m1.getColCount();
			const size_t M2C = m2.getColCount();

			ASSERTMSG_(m1.getColCount() == m2.getRowCount(),"multiply_result_is_symmetric: Inconsistent matrix sizes in multiplication!")

			// If one of the matrices is me, make a copy:
			if ((void*)&m1==(void*)&out || (void*)&m2==(void*)&out)
			{
				// Save result in a temporary matrix:
				std::vector<T> temp(M1R*M2C);
				T  *ptr = &temp[0];
				for (size_t i=0; i < M1R; i++)
					for (size_t j=i; j < M2C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(k,j);
						*(ptr++) = accum;
					}

				// Copy from temp:
				out.setSize(M1R,M2C);
				ptr = &temp[0];
				for (size_t i=0; i < M1R; i++)
					for (size_t j=i; j < M1C; j++)
						out.get_unsafe(i,j) = out.get_unsafe(j,i) = *(ptr++);
			}
			else
			{
				// Work directly over the data:
				out.setSize( M1R,M2C );
				for (size_t i=0; i < M1R; i++)
					for (size_t j=i; j < M2C; j++)
					{
						T accum = 0;
						for (size_t k=0; k < M1C; k++)
							accum += m1.get_unsafe(i,k) * m2.get_unsafe(k,j);
						 out.set_unsafe(i,j,accum);
					}
				for (size_t i=0; i < M1R; i++)
					for (size_t j=0; j<i; j++)
						out.get_unsafe(j,i) = out.get_unsafe(i,j);
			}
			MRPT_END
		}

		// Insert transposed matrix:
		template <class MAT1,class MAT2>
		void insertMatrixTransposeInto( MAT1 &M,const size_t nRow,const size_t nCol,const MAT2 &in)
		{
			const size_t mRows = size(in,1);
			const size_t mCols = size(in,2);
			ASSERT_BELOWEQ_( nRow+mCols, M.getRowCount() )
			ASSERT_BELOWEQ_( nCol+mRows, M.getColCount() )
			for (size_t c=0;c<mCols;c++)
				for (size_t r=0;r<mRows;r++)
					M.get_unsafe(nRow+c,nCol+r) = in.get_unsafe(r,c);
		}

		// Insert matrix
		template <class MAT1,class MAT2>
		inline void insertMatrixInto( MAT1 &M,const size_t nRow,const size_t nCol,const MAT2 &in)
		{
			const size_t mRows = size(in,1);
			const size_t mCols = size(in,2);
			ASSERT_BELOWEQ_( nRow+mRows, M.getRowCount() )
			ASSERT_BELOWEQ_( nCol+mCols, M.getColCount() )
			for (size_t r=0;r<mRows;r++)
				for (size_t c=0;c<mCols;c++)
					M.get_unsafe(nRow+r,nCol+c) = in.get_unsafe(r,c);
		}

		/** Extract a submatrix - The output matrix must be set to the required size before call. */
		template <class MATORG, class MATDEST>
		void extractMatrix(
			const MATORG &M,
			const size_t first_row,
			const size_t first_col,
			MATDEST &outMat)
		{
			const size_t NR = outMat.getRowCount();
			const size_t NC = outMat.getColCount();
			ASSERT_BELOWEQ_( first_row+NR, M.getRowCount() )
			ASSERT_BELOWEQ_( first_col+NC, M.getColCount() )
			for (size_t r=0;r<NR;r++)
				for (size_t c=0;c<NC;c++)
					outMat.get_unsafe(r,c) = M.get_unsafe(first_row+r,first_col+c);
		}


		/** @name Matrix inverses - Implementation
			@{ */
		// templates for special cases:
		template <class MATRIXIN,class MATRIXOUT>
		inline void invMatrix_special_2x2( const MATRIXIN &M, MATRIXOUT &out_inv )
		{
			typedef typename MATRIXIN::value_type T;
			// | a11 a12 |-1             |  a22 -a12 |
			// | a21 a22 |    =  1/DET * | -a21  a11 |
			//
			const T det = M.det();
			ASSERTMSG_(det!=0,"Singular matrix")
			const T det_inv = 1.0 / det;
			out_inv.setSize(2,2);
			out_inv._E(1,1) =  M._E(2,2) * det_inv;
			out_inv._E(1,2) = -M._E(1,2) * det_inv;
			out_inv._E(2,1) = -M._E(2,1) * det_inv;
			out_inv._E(2,2) =  M._E(1,1) * det_inv;
		}
		template <class MATRIXIN,class MATRIXOUT>
		inline void invMatrix_special_3x3( const MATRIXIN &M, MATRIXOUT &out_inv )
		{
			typedef typename MATRIXIN::value_type T;
			// | a11 a12 a13 |-1             |   a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13  |
			// | a21 a22 a23 |    =  1/DET * | -(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13) |
			// | a31 a32 a33 |               |   a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12  |
			const T det = M.det();
			ASSERTMSG_(det!=0,"Singular matrix")
			const T det_inv = 1.0 / det;
			out_inv.setSize(3,3);
			out_inv._E(1,1)	=  (M._E(3,3)*M._E(2,2)-M._E(3,2)*M._E(2,3) ) * det_inv;
			out_inv._E(1,2) =  (-M._E(3,3)*M._E(1,2)+M._E(3,2)*M._E(1,3) )* det_inv;
			out_inv._E(1,3) =  (M._E(2,3)*M._E(1,2)-M._E(2,2)*M._E(1,3) )* det_inv;
			out_inv._E(2,1) =  (-M._E(3,3)*M._E(2,1)+M._E(3,1)*M._E(2,3))* det_inv;
			out_inv._E(2,2) =  (M._E(3,3)*M._E(1,1)-M._E(3,1)*M._E(1,3))* det_inv;
			out_inv._E(2,3) =  (-M._E(2,3)*M._E(1,1)+M._E(2,1)*M._E(1,3))* det_inv;
			out_inv._E(3,1) =  (M._E(3,2)*M._E(2,1)-M._E(3,1)*M._E(2,2))* det_inv;
			out_inv._E(3,2) =  (-M._E(3,2)*M._E(1,1)+M._E(3,1)*M._E(1,2))* det_inv;
			out_inv._E(3,3) =  (M._E(2,2)*M._E(1,1)-M._E(2,1)*M._E(1,2))* det_inv;
		}

		// specializations that call the above special cases:
		template <> inline void invMatrix<CMatrixFixedNumeric<float,2,2>,CMatrixFixedNumeric<float,2,2> >( const CMatrixFixedNumeric<float,2,2> &M, CMatrixFixedNumeric<float,2,2> &out_inv ) { invMatrix_special_2x2(M,out_inv);	}
		template <> inline void invMatrix_destroySrc<CMatrixFixedNumeric<float,2,2>,CMatrixFixedNumeric<float,2,2> >( CMatrixFixedNumeric<float,2,2> &M, CMatrixFixedNumeric<float,2,2> &out_inv )  { invMatrix_special_2x2(M,out_inv);	}
		template <> inline void invMatrix<CMatrixFixedNumeric<double,2,2>,CMatrixFixedNumeric<double,2,2> >( const CMatrixFixedNumeric<double,2,2> &M, CMatrixFixedNumeric<double,2,2> &out_inv ) { invMatrix_special_2x2(M,out_inv);	}
		template <> inline void invMatrix_destroySrc<CMatrixFixedNumeric<double,2,2>,CMatrixFixedNumeric<double,2,2> >( CMatrixFixedNumeric<double,2,2> &M, CMatrixFixedNumeric<double,2,2> &out_inv )  { invMatrix_special_2x2(M,out_inv);	}
		template <> inline void invMatrix<CMatrixFixedNumeric<float,3,3>,CMatrixFixedNumeric<float,3,3> >( const CMatrixFixedNumeric<float,3,3> &M, CMatrixFixedNumeric<float,3,3> &out_inv ) { invMatrix_special_3x3(M,out_inv);	}
		template <> inline void invMatrix_destroySrc<CMatrixFixedNumeric<float,3,3>,CMatrixFixedNumeric<float,3,3> >( CMatrixFixedNumeric<float,3,3> &M, CMatrixFixedNumeric<float,3,3> &out_inv )  { invMatrix_special_3x3(M,out_inv);	}
		template <> inline void invMatrix<CMatrixFixedNumeric<double,3,3>,CMatrixFixedNumeric<double,3,3> >( const CMatrixFixedNumeric<double,3,3> &M, CMatrixFixedNumeric<double,3,3> &out_inv ) { invMatrix_special_3x3(M,out_inv);	}
		template <> inline void invMatrix_destroySrc<CMatrixFixedNumeric<double,3,3>,CMatrixFixedNumeric<double,3,3> >( CMatrixFixedNumeric<double,3,3> &M, CMatrixFixedNumeric<double,3,3> &out_inv )  { invMatrix_special_3x3(M,out_inv);	}

		template <class MATRIXIN,class MATRIXOUT>
		inline void  invMatrix( const MATRIXIN &M, MATRIXOUT &out_inv )
		{
			MAT_TYPE_SAMESIZE_OF(MATRIXIN) temp;
			temp.assignMatrix(M);
			invMatrix_destroySrc(temp,out_inv);  // temp is destroyed in inv_fast
		}

		template <class MATRIXIN,class MATRIXOUT>
		inline void  invMatrix_destroySrc( MATRIXIN &M, MATRIXOUT &out_inv )
		{
#if 0
			out_inv.setIdentity(M.getRowCount());
			mrpt::math::detail::fastLeftDivideSquare(out_inv,M);
			return;
#else
			typedef typename MATRIXIN::value_type T;
			ASSERTMSG_(M.IsSquare(),"Inversion of non-square matrix")
			// Check this here for dynamic-size matrices:
			const size_t NROWS = M.getRowCount();
			if (NROWS==3) return invMatrix_special_3x3(M,out_inv);
			else if (NROWS==2) return invMatrix_special_2x2(M,out_inv);

			// Generic algorithm for any-size matrices:
			T a1,a2;
			out_inv.setSize(NROWS,NROWS);
			out_inv.unit();
			for (size_t k=0; k < NROWS; k++)
			{
				int indx = M.pivot(k);
				if (indx == -1)
					THROW_EXCEPTION( "Inversion of a singular matrix");

				if (indx != 0)
					out_inv.swapRows(k,indx);	// Swap rows in out_inv - they have been ALREADY SWAPPED in M within pivot()

				a1 = M.get_unsafe(k,k);
				ASSERTDEB_(a1!=0)
				const T a1_i = 1/a1;
				for (size_t j=0; j < NROWS; j++)
				{
					M.get_unsafe(k,j) *= a1_i;
					out_inv.get_unsafe(k,j) *= a1_i;
				}
				for (size_t i=0; i < NROWS; i++)
				{
					if (i != k)
					{
						a2 = M.get_unsafe(i,k);
						for (size_t j=0; j < NROWS; j++)
						{
							M.get_unsafe(i,j)  -= a2 * M.get_unsafe(k,j);
							out_inv.get_unsafe(i,j) -= a2 * out_inv.get_unsafe(k,j);
						}
					}
				}
			}
#endif
		}

		/** @} */  // END OF MATRIX INVERSES


		/** @name Matrix determinants - implementation
		   @{ */
		// Special cases:
		template <class MATRIX> inline typename MATRIX::value_type
		detMatrix_special_2x2(const MATRIX &M)
		{
			return M._E(1,1)*M._E(2,2) - M._E(1,2)*M._E(2,1);
		}
		template <class MATRIX> inline typename MATRIX::value_type
		detMatrix_special_3x3(const MATRIX &M)
		{
			return M._E(1,1)*(M._E(3,3)*M._E(2,2)-M._E(3,2)*M._E(2,3))-
				M._E(2,1)*(M._E(3,3)*M._E(1,2)-M._E(3,2)*M._E(1,3))+
				M._E(3,1)*(M._E(2,3)*M._E(1,2)-M._E(2,2)*M._E(1,3));
		}
		template <class MATRIX> typename MATRIX::value_type
		detMatrix_special_4x4(const MATRIX &M)
		{
			typedef typename MATRIX::value_type T;
			const T D1 = M._E(1+1,1+1)*(M._E(3+1,3+1)*M._E(2+1,2+1)-M._E(3+1,2+1)*M._E(2+1,3+1))-
			             M._E(2+1,1+1)*(M._E(3+1,3+1)*M._E(1+1,2+1)-M._E(3+1,2+1)*M._E(1+1,3+1))+
			             M._E(3+1,1+1)*(M._E(2+1,3+1)*M._E(1+1,2+1)-M._E(2+1,2+1)*M._E(1+1,3+1));
			const T D2 = M._E(1+1,1)*(M._E(3+1,3+1)*M._E(2+1,2+1)-M._E(3+1,2+1)*M._E(2+1,3+1))-
			             M._E(2+1,1)*(M._E(3+1,3+1)*M._E(1+1,2+1)-M._E(3+1,2+1)*M._E(1+1,3+1))+
			             M._E(3+1,1)*(M._E(2+1,3+1)*M._E(1+1,2+1)-M._E(2+1,2+1)*M._E(1+1,3+1));
			const T D3 = M._E(1+1,1)*(M._E(3+1,3+1)*M._E(2+1,2)-M._E(3+1,2)*M._E(2+1,3+1))-
			             M._E(2+1,1)*(M._E(3+1,3+1)*M._E(1+1,2)-M._E(3+1,2)*M._E(1+1,3+1))+
			             M._E(3+1,1)*(M._E(2+1,3+1)*M._E(1+1,2)-M._E(2+1,2)*M._E(1+1,3+1));
			const T D4 = M._E(1+1,1)*(M._E(3+1,3)*M._E(2+1,2)-M._E(3+1,2)*M._E(2+1,3))-
			             M._E(2+1,1)*(M._E(3+1,3)*M._E(1+1,2)-M._E(3+1,2)*M._E(1+1,3))+
			             M._E(3+1,1)*(M._E(2+1,3)*M._E(1+1,2)-M._E(2+1,2)*M._E(1+1,3));
			return M._E(1,1)*D1 - M._E(1,2)*D2 + M._E(1,3)*D3 - M._E(1,4)*D4;
		}

		// Specializations that call the above functions:
		template <> inline float  detMatrix<CMatrixFixedNumeric<float,2,2> >(const CMatrixFixedNumeric<float,2,2> &M) { return detMatrix_special_2x2(M); }
		template <> inline double detMatrix<CMatrixFixedNumeric<double,2,2> >(const CMatrixFixedNumeric<double,2,2> &M)  { return detMatrix_special_2x2(M); }
		template <> inline float  detMatrix<CMatrixFixedNumeric<float,3,3> >(const CMatrixFixedNumeric<float,3,3> &M)  { return detMatrix_special_3x3(M); }
		template <> inline double detMatrix<CMatrixFixedNumeric<double,3,3> >(const CMatrixFixedNumeric<double,3,3> &M) { return detMatrix_special_3x3(M); }
		template <> inline float  detMatrix<CMatrixFixedNumeric<float,4,4> >(const CMatrixFixedNumeric<float,4,4> &M)  { return detMatrix_special_4x4(M); }
		template <> inline double detMatrix<CMatrixFixedNumeric<double,4,4> >(const CMatrixFixedNumeric<double,4,4> &M) { return detMatrix_special_4x4(M); }

		template <class MATRIX> RET_ELEMENT_ASSERT_MRPTCONTAINER(MATRIX)
		detMatrix(const MATRIX& M)
		{
			typedef typename MATRIX::value_type T;
			ASSERTMSG_(M.IsSquare(),"Inversion of non-square matrix")
			// Check this here for dynamic-size matrices:
			const size_t NROWS = M.getRowCount();
			if (NROWS==3) return detMatrix_special_3x3(M);
			else if (NROWS==2) return detMatrix_special_2x2(M);
			// general case:
			MAT_TYPE_SAMESIZE_OF(MATRIX) temp;
			temp.assignMatrix(M);
			T detVal = T(1);
			for (size_t k=0; k < NROWS; k++)
			{
				int	indx = temp.pivot(k);
				if (indx == -1) return 0;
				if (indx != 0)	detVal = - detVal;
				const T temp_kk = temp.get_unsafe(k,k);
				const T temp_kk_inv = T(1)/temp_kk;
				detVal = detVal * temp_kk;
				for (size_t i=k+1; i < NROWS; i++)
				{
					T piv = temp.get_unsafe(i,k)*temp_kk_inv; // was: / temp.get_unsafe(k,k);
					for (size_t j=k+1; j < NROWS; j++)
						temp.get_unsafe(i,j) -= piv * temp.get_unsafe(k,j);
				}
			}
			return detVal;
		}

		/** @} */  // END OF MATRIX DETERMINANTS

		/** Matrix pivoting - used for detMatrix, etc. */
		template <class MATRIX>
		int matrix_pivot(MATRIX &M, const size_t row)
		{
			typedef typename MATRIX::value_type T;
			size_t k = row;
			const size_t NROWS = M.getRowCount();
			T temp, amax = -1;
			for (size_t i=row; i < NROWS; i++)
				if ( (temp = std::abs( M.get_unsafe(i,row))) > amax && temp != 0)
				{
					amax = temp;
					k = i;
				}
			if (M.get_unsafe(k,row) == T(0)) return -1; // No one found.
			if (k != row)
			{	// Swap rows "k" & "row":
				M.swapRows(k,row);
				return static_cast<int>( k );
			}
			return 0;
		}

		template <class MAT,class VEC>
		void  extractRowFromMatrix(const MAT &m, size_t nRow, VEC &out, const size_t startingCol = 0)
		{
	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nRow>=size(m,1))
				THROW_EXCEPTION("extractRow: Row index out of bounds");
	#endif
			const size_t n = size(m,2) - startingCol ;
			out.resize( n );
			for (size_t i=0;i<n;i++)
				out[i] = static_cast<typename MAT::value_type>( m.get_unsafe(nRow,i+startingCol));
		}

		template <class MAT,class VEC>
		void  extractColFromMatrix(const MAT &m, size_t nCol, VEC &out, const size_t startingRow = 0)
		{
	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nCol>=size(m,2))
				THROW_EXCEPTION("extractCol: Col index out of bounds");
	#endif
			const size_t n = size(m,1) - startingRow;
			out.resize( n );
			for (size_t i=0;i<n;i++)
				out[i] = static_cast<typename MAT::value_type>( m.get_unsafe(i+startingRow,nCol));
		}

		template <class MAT,class VEC>
		void  insertRowToMatrix(MAT &m, size_t nRow, const VEC &in, const size_t startingCol=0)
		{
	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nRow>=size(m,1))
				THROW_EXCEPTION("insertRow: Row index out of bounds");
	#endif
			ASSERT_( (size(m,2)-startingCol)>=in.size() )
			const size_t n = size(m,2) - startingCol ;
			for (size_t i=0;i<n;i++)
				m.set_unsafe(nRow,i+startingCol, static_cast<typename MAT::value_type>(in[i]) );
		}

		template <class MAT,class VEC>
		void  insertColToMatrix(MAT &m, size_t nCol, const VEC &in, const size_t startingRow=0)
		{
	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (nCol>=size(m,2))
				THROW_EXCEPTION("insertCol: Col index out of bounds");
	#endif
			ASSERT_( (size(m,1)-startingRow)>=in.size() )
			const size_t n = size(m,1) - startingRow;
			for (size_t i=0;i<n;i++)
				m.set_unsafe(i+startingRow,nCol, static_cast<typename MAT::value_type>(in[i]));
		}

		template <class CONTAINER>
		void loadFromTextFileAsVector(CONTAINER &M, const std::string &file)
		{
			mrpt::math::CMatrixTemplateNumeric<double> MM;
			MM.loadFromTextFile(file);
			M.resize(MM.size());
			mrpt::utils::metaprogramming::copy_container_typecasting(MM,M);
		}

	} // end detail

	/** Just like s=H.multiply_HCHt_scalar(C), but defined in mrpt::math for backward compatibility. */
	template <class MAT1,class MAT2>
	inline typename MAT1::value_type multiply_HCHt_scalar(const MAT1 &H, const MAT2 &C) {
		return detail::multiply_HCHt_scalar(H,C);
	}

	/** Binary matrix division operator A/B = A*inv(B) */
	template <class T>
	inline CMatrixTemplateNumeric<T>  operator / (const CMatrixTemplateNumeric<T>& m1, const CMatrixTemplateNumeric<T>& m2)
	{	// JL: This is hard to generalize for any mix of dyn. fix. matrices, but I don't think it's worth in any case...
		return (m1 * !m2);
	}

	/** binary power operator */
	template <class T>
	inline CMatrixTemplateNumeric<T>  operator ^ (const CMatrixTemplateNumeric<T>& m, const unsigned int pow)
	{
		CMatrixTemplateNumeric<T>	temp(m);
		temp ^= pow;
		return temp;
	}


	/** unary transpose operator ~ */
	template <class MAT>
	inline MAT_TYPE_TRANSPOSE_OF(MAT)
		operator ~ (const MAT& m)
	{
		const size_t R=m.getRowCount();
		const size_t C=m.getColCount();
		MAT_TYPE_TRANSPOSE_OF(MAT) temp; temp.setSize(C,R); // transpose
		for (size_t i=0;i<R;i++)
			for (size_t j=0;j<C;j++)
				temp.get_unsafe(j,i) = m.get_unsafe(i,j);
		return temp;
	}

	/** Unary inversion operator. */
	template <class MATRIX>
	inline RET_MAT_ASSERT_MRPTMATRIX(MATRIX)
	operator !(const MATRIX &m) {
		RET_MAT_ASSERT_MRPTMATRIX(MATRIX) ret(UNINITIALIZED_MATRIX);
		m.inv(ret);
		return ret;
	}


	// Operator * uses MAT_TYPE_PRODUCT_OF to manage these four cases:
	//  FIX * FIX -> FIX (with the correct sizes)
	//  DYN * FIX -> DYN
	//  FIX * DYN -> DYN
	//  DYN * DYN -> DYN
	// --------------------------------------------------
	/** Matrix multiplication operator: A * B -> RES
	  *  The meaning of the lengthy macros in the declaration is:
	  *   - MAT_TYPE_PRODUCT_OF: Return type is the correct one for A*B, e.g. NxM * MxK -> NxK,  DYN*DYN -> DYN.
	  *   - RET_MAT_ASSERT_MRPTMATRIX: Assure that the compiler will use this "operator*" only with MRPT matrices.
	  */
	template <class MAT1,class MAT2>
	inline MAT_TYPE_PRODUCT_OF(MAT1,MAT2)
	operator * ( const MAT1 &A,
	             const MAT2 &B  )
	{
		MAT_TYPE_PRODUCT_OF(MAT1,MAT2)  RES(UNINITIALIZED_MATRIX);
		detail::multiply_AB(A,B, RES);
		return RES;
	}

	/** Computes the mean vector and covariance from a list of samples in an NxM matrix, where each row is a sample, so the covariance is MxM.
	  * \param v The set of data as a NxM matrix, of types: CMatrixTemplateNumeric or CMatrixFixedNumeric
	  * \param out_mean The output M-vector for the estimated mean.
	  * \param out_cov The output MxM matrix for the estimated covariance matrix, this can also be either a fixed-size of dynamic size matrix.
	  * \sa math::mean,math::stddev, math::cov
	  */
	template<class MAT_IN, class MAT_OUT>
	void meanAndCov(
		const MAT_IN &v,
		vector_double	&out_mean,
		MAT_OUT 		&out_cov
		)
	{
		const size_t N = v.getRowCount();
		ASSERTMSG_(N>0,"The input matrix contains no elements");
		const double N_inv = 1.0/N;

		const size_t M = v.getColCount();
		ASSERTMSG_(M>0,"The input matrix contains rows of length 0");

		// First: Compute the mean
		out_mean.assign(M,0);
		for (size_t i=0;i<N;i++)
			for (size_t j=0;j<M;j++)
				out_mean[j]+=v.get_unsafe(i,j);
		out_mean*=N_inv;

		// Second: Compute the covariance
		//  Save only the above-diagonal part, then after averaging
		//  duplicate that part to the other half.
		out_cov.zeros(M,M);
		for (size_t i=0;i<N;i++)
		{
			for (size_t j=0;j<M;j++)
				out_cov.get_unsafe(j,j)+=square(v.get_unsafe(i,j)-out_mean[j]);

			for (size_t j=0;j<M;j++)
				for (size_t k=j+1;k<M;k++)
					out_cov.get_unsafe(j,k)+=(v.get_unsafe(i,j)-out_mean[j])*(v.get_unsafe(i,k)-out_mean[k]);
		}
		for (size_t j=0;j<M;j++)
			for (size_t k=j+1;k<M;k++)
				out_cov.get_unsafe(k,j) = out_cov.get_unsafe(j,k);
		out_cov*=N_inv;
	}

	/** Computes the covariance matrix from a list of samples in an NxM matrix, where each row is a sample, so the covariance is MxM.
	  * \param v The set of data, as a NxM matrix.
	  * \param out_cov The output MxM matrix for the estimated covariance matrix.
	  * \sa math::mean,math::stddev, math::cov
	  */
	template<class MATRIX>
	inline MAT_TYPE_COVARIANCE_OF(MATRIX) cov( const MATRIX &v )
	{
		vector_double m;
		MAT_TYPE_COVARIANCE_OF(MATRIX) C;
		meanAndCov(v,m,C);
		return C;
	}

	/** A useful macro for saving matrixes to a file while debugging. */
	#define SAVE_MATRIX(M) \
		M.saveToTextFile(mrpt::format("%s.txt",#M));


	// ------ Implementatin of Vicinity templates -------------
	namespace detail	{
		/**
		  * Vicinity traits class specialization for matrices.
		  */
		template<typename T> class VicinityTraits<CMatrixTemplate<T> >	{
		public:
			inline static void initialize(CMatrixTemplate<T> &mat,size_t N)	{
				mat.setSize(N,N,true);
			}
			inline static void insertInContainer(CMatrixTemplate<T> &mat,size_t r,size_t c,const T &t)	{
				mat.get_unsafe(r,c)=t;
			}
		};
		/**
		  * Vicinity traits class specialization for vectors. It ignores the spatial distribution, inserting elements at the end of the vector.
		  */
		template<typename T> class VicinityTraits<std::vector<T> >	{
		public:
			inline static void initialize(std::vector<T> &vec,size_t N)	{
				vec.reserve(N*N);
			}
			inline static void insertInContainer(std::vector<T> &vec,size_t,size_t,const T &t)	{
				vec.push_back(t);
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ * _ )
		  * ( * _ * )
		  * ( _ * _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,4>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<1>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,3);
				VicinityTraits<ReturnType>::insertInContainer(res,0,1,mat.get_unsafe(r-1,c));
				VicinityTraits<ReturnType>::insertInContainer(res,1,0,mat.get_unsafe(r,c-1));
				VicinityTraits<ReturnType>::insertInContainer(res,1,2,mat.get_unsafe(r,c+1));
				VicinityTraits<ReturnType>::insertInContainer(res,2,1,mat.get_unsafe(r+1,c));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ * _ )
		  * ( * * * )
		  * ( _ * _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,5>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<1>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,3);
				VicinityTraits<ReturnType>::insertInContainer(res,0,1,mat.get_unsafe(r-1,c));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,1,i+1,mat.get_unsafe(r,c+i));
				VicinityTraits<ReturnType>::insertInContainer(res,2,1,mat.get_unsafe(r+1,c));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( * * * )
		  * ( * _ * )
		  * ( * * * )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,8>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<1>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,3);
				for (int i=-1;i<=1;++i) for (int j=-1;j<=1;++j) if (i||j) VicinityTraits<ReturnType>::insertInContainer(res,i+1,j+1,mat.get_unsafe(r+i,c+j));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( * * * )
		  * ( * * * )
		  * ( * * * )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,9>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<1>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,3);
				for (int i=-1;i<=1;++i) for (int j=-1;j<=1;++j) VicinityTraits<ReturnType>::insertInContainer(res,i+1,j+1,mat.get_unsafe(r+i,c+j));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ _ * _ _ )
		  * ( _ * * * _ )
		  * ( * * _ * * )
		  * ( _ * * * _ )
		  * ( _ _ * _ _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,12>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				VicinityTraits<ReturnType>::insertInContainer(res,0,2,mat.get_unsafe(r-2,c));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,1,i+2,mat.get_unsafe(r-1,c+i));
				for (int i=-2;i<=2;++i) if (i) VicinityTraits<ReturnType>::insertInContainer(res,2,i+2,mat.get_unsafe(r,c+i));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,3,i+2,mat.get_unsafe(r+1,c+i));
				VicinityTraits<ReturnType>::insertInContainer(res,4,2,mat.get_unsafe(r+2,c));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ _ * _ _ )
		  * ( _ * * * _ )
		  * ( * * * * * )
		  * ( _ * * * _ )
		  * ( _ _ * _ _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,13>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				VicinityTraits<ReturnType>::insertInContainer(res,0,2,mat.get_unsafe(r-2,c));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,1,i+2,mat.get_unsafe(r-1,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,2,i+2,mat.get_unsafe(r,c+i));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,3,i+2,mat.get_unsafe(r+1,c+i));
				VicinityTraits<ReturnType>::insertInContainer(res,4,2,mat.get_unsafe(r+2,c));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ * * * _ )
		  * ( * * * * * )
		  * ( * * _ * * )
		  * ( * * * * * )
		  * ( _ * * * _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,20>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,0,i+2,mat.get_unsafe(r-2,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,1,i+2,mat.get_unsafe(r-1,c+i));
				for (int i=-2;i<=2;++i) if (i) VicinityTraits<ReturnType>::insertInContainer(res,2,i+2,mat.get_unsafe(r,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,3,i+2,mat.get_unsafe(r+1,c+i));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,4,i+2,mat.get_unsafe(r+2,c+i));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( _ * * * _ )
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( _ * * * _ )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,21>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,0,i+2,mat.get_unsafe(r-2,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,1,i+2,mat.get_unsafe(r-1,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,2,i+2,mat.get_unsafe(r,c+i));
				for (int i=-2;i<=2;++i) VicinityTraits<ReturnType>::insertInContainer(res,3,i+2,mat.get_unsafe(r+1,c+i));
				for (int i=-1;i<=1;++i) VicinityTraits<ReturnType>::insertInContainer(res,4,i+2,mat.get_unsafe(r+2,c+i));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( * * _ * * )
		  * ( * * * * * )
		  * ( * * * * * )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,24>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				for (int i=-2;i<=2;++i) for (int j=-2;j<=2;++j) if (i||j) VicinityTraits<ReturnType>::insertInContainer(res,i+2,j+2,mat.get_unsafe(r+i,c+j));
				return res;
			}
		};
		/**
		  * Template specialization for getVicinity.
		  *
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( * * * * * )
		  * ( * * * * * )
		  *
		  */
		template<typename MatrixType,typename T,typename ReturnType> struct getVicinity<MatrixType,T,ReturnType,25>	{
		public:
			static ReturnType get(size_t r,size_t c,const MatrixType &mat)	{
				mat.ASSERT_ENOUGHROOM<2>(r,c);
				ReturnType res;
				VicinityTraits<ReturnType>::initialize(res,5);
				for (int i=-2;i<=2;++i) for (int j=-2;j<=2;++j) VicinityTraits<ReturnType>::insertInContainer(res,i+2,j+2,mat.get_unsafe(r+i,c+j));
				return res;
			}
		};

		template<typename MatrixType> size_t rank(const MatrixType &m,typename MatrixType::value_type eps)	{
			size_t N=m.getRowCount();
			ASSERT_(m.getColCount()==N);
			MAT_TYPE_SAMESIZE_OF(MatrixType) tmp;
			tmp.assignMatrix(m);
			CArbitrarySubmatrixView<MatrixType> mat(tmp,0,N,0,N);
			size_t res=0;
			while (mat.getRowCount()!=0&&mat.getColCount()!=0)	{
				size_t row=0;
				if (std::abs(mat(0,0)<=eps))	{
					for (size_t i=1;i<mat.getColCount();++i) if (std::abs(mat(i,0))>eps)	{
						row=i;
						break;
					}
					if (row==0)	{
						mat.deleteColumn(0);
						continue;
					}
				}
				size_t pRow=mat.getProxyRow(row);
				for (size_t i=0;i<mat.getRowCount();++i)	{
					if (i==row) continue;
					size_t pRowAlt=mat.getProxyRow(i);
					typename MatrixType::value_type prop=mat.getWithRowProxied(pRowAlt,0)/mat.getWithRowProxied(pRow,0);
					for (size_t j=1;j<mat.getColCount();++j) mat.getWithRowProxied(pRowAlt,j)-=prop*mat.getWithRowProxied(pRow,j);
				}
				res++;
				mat.deleteColumn(0);
				mat.deleteRow(row);
			}
			return res;
		}

		template<typename JA> void pivotUntilIdentity(JointAccessor<JA> &joint)	{
			size_t N=joint.size();
			//First step: the lower submatrix is erased ("downwards" movement).
			for (size_t i=0;i<N-1;++i)	{
				//std::cout << "pivotUntilIdentity, i="<<i<< ", mat: "<< std::endl; joint.dumpToConsole(); std::cout << std::endl;
				joint.ensureSuitablePos(i);
				for (size_t j=i+1;j<N;++j) joint.substractRowAsNeeded(i,j);
				joint.unitarizeReducedRow(i);
			}
			joint.ensureAndUnitarizeLast();
			//std::cout << "pivotUntilIdentity, after last, mat: "<< std::endl; joint.dumpToConsole(); std::cout << std::endl;
			//Second step: the upper submatrix is erased ("upwards" movement, after the lower submatrix is cleared).
			for (size_t i=N-1;i>0;--i) for (int j=i-1;j>=0;--j) joint.substractWhenReduced(i,j);
		}

		/*! Matrix left divide: RES = A<sup>-1</sup> · C  (A must be a square matrix). \sa rightDivideSquare,fastLeftDivideSquare */
		template <typename MAT1,typename MAT2,typename MAT3> inline void leftDivideSquare(const MAT1 &C,const MAT2 &A,MAT3 &RES) {
			MAT_TYPE_SAMESIZE_OF(MAT2) tmp;
			tmp.assignMatrix(A);
			RES.assignMatrix(C);
			fastLeftDivideSquare(RES,tmp);
		}

		/*! Matrix right divide: RES = C · B<sup>-1</sup> (B must be a square matrix). \sa leftDivideSquare,fastRightDivideSquare */
		template <typename MAT1,typename MAT2,typename MAT3> inline void rightDivideSquare(const MAT1 &C,const MAT2 &B, MAT3 &RES) {
			MAT_TYPE_SAMESIZE_OF(MAT2) tmp;
			tmp.assignMatrix(B);
			RES.assignMatrix(C);
			fastRightDivideSquare(RES,tmp);
		}

		/*! Matrix left divide: B=A<sup>-1</sup>·C. A must be a square matrix, which is destroyed. C is also destroyed, and assigned to the looked for quotient. */
		template<typename MAT1,typename MAT2> inline void fastLeftDivideSquare(MAT1 &inout_CB,MAT2 &willBeDestroyed_A)	{
			JointHorizontalAccessor<MAT2,MAT1> jha=JointHorizontalAccessor<MAT2,MAT1>(willBeDestroyed_A,inout_CB);	//I don't like temporary variables, but this needs a scope.
			JointAccessor<JointHorizontalAccessor<MAT2,MAT1> > ja=JointAccessor<JointHorizontalAccessor<MAT2,MAT1> >(jha);
			pivotUntilIdentity(ja);
		}

		/*! Matrix left divide: A=C·B<sup>-1</sup>. B must be a square matrix, which is destroyed. C is also destroyed, and assigned to the looked for quotient. */
		template<typename MAT1,typename MAT2> inline void fastRightDivideSquare(MAT1 &inout_CA,MAT2 &willBeDestroyed_B)	{
			JointVerticalAccessor<MAT2,MAT1> jva=JointVerticalAccessor<MAT2,MAT1>(willBeDestroyed_B,inout_CA);	//I don't like temporary variables, but this needs a scope.
			JointAccessor<JointVerticalAccessor<MAT2,MAT1> > ja=JointAccessor<JointVerticalAccessor<MAT2,MAT1> >(jva);
			pivotUntilIdentity(ja);
		}

	} // end of detail namespace

	} // End of math namespace
} // End of mrpt namespace


#endif
