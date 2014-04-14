/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_MATH_H
#define  MRPT_MATH_H

#include <mrpt/utils/utils_defs.h>
//#include <mrpt/math/CMatrixTemplateNumeric.h>
//#include <mrpt/math/CMatrixFixedNumeric.h>
//#include <mrpt/math/CHistogram.h>

//#include <mrpt/math/ops_vectors.h>
//#include <mrpt/math/ops_matrices.h>

namespace mrpt
{
	/** This base provides a set of functions for maths stuff. \ingroup mrpt_base_grp
	 */
	namespace math
	{
		/** \addtogroup container_ops_grp
		  * @{ */

		/** Loads one row of a text file as a numerical std::vector.
		  * \return false on EOF or invalid format.
		  * The body of the function is implemented in MATH.cpp
			*/
		bool BASE_IMPEXP loadVector( utils::CFileStream &f, std::vector<int> &d);

		/** Loads one row of a text file as a numerical std::vector.
		  * \return false on EOF or invalid format.
		  * The body of the function is implemented in MATH.cpp
			*/
		bool BASE_IMPEXP loadVector( utils::CFileStream &f, std::vector<double> &d);


        /** Returns true if the number is NaN. */
        bool BASE_IMPEXP  isNaN(float  f) MRPT_NO_THROWS;

        /** Returns true if the number is NaN. */
        bool  BASE_IMPEXP isNaN(double f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity. */
        bool BASE_IMPEXP  isFinite(float f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity.  */
        bool  BASE_IMPEXP isFinite(double f) MRPT_NO_THROWS;

        void BASE_IMPEXP medianFilter( const std::vector<double> &inV, std::vector<double> &outV, const int &winSize, const int &numberOfSigmas = 2 );

#ifdef HAVE_LONG_DOUBLE
		/** Returns true if the number is NaN. */
        bool  BASE_IMPEXP isNaN(long double f) MRPT_NO_THROWS;

        /** Returns true if the number is non infinity. */
        bool BASE_IMPEXP  isFinite(long double f) MRPT_NO_THROWS;
#endif

		/** Generates an equidistant sequence of numbers given the first one, the last one and the desired number of points.
		  \sa sequence */
		template<typename T,typename VECTOR>
		void linspace(T first,T last, size_t count, VECTOR &out_vector)
		{
			if (count<2)
			{
				out_vector.assign(count,last);
				return;
			}
			else
			{
				out_vector.resize(count);
				const T incr = (last-first)/T(count-1);
				T c = first;
				for (size_t i=0;i<count;i++,c+=incr)
					out_vector[i] = c;
			}
		}

		/** Generates an equidistant sequence of numbers given the first one, the last one and the desired number of points.
		  \sa sequence */
		template<class T>
		inline Eigen::Matrix<T,Eigen::Dynamic,1> linspace(T first,T last, size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> ret;
			mrpt::math::linspace(first,last,count,ret);
			return ret;
		}

		/** Generates a sequence of values [first,first+STEP,first+2*STEP,...]   \sa linspace, sequenceStdVec */
		template<class T,T STEP>
		inline Eigen::Matrix<T,Eigen::Dynamic,1> sequence(T first,size_t length)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> ret(length);
			if (!length) return ret;
			size_t i=0;
			while (length--) { ret[i++]=first; first+=STEP; }
			return ret;
		}

		/** Generates a sequence of values [first,first+STEP,first+2*STEP,...]   \sa linspace, sequence */
		template<class T,T STEP>
		inline std::vector<T> sequenceStdVec(T first,size_t length)
		{
			std::vector<T> ret(length);
			if (!length) return ret;
			size_t i=0;
			while (length--) { ret[i++]=first; first+=STEP; }
			return ret;
		}

		/** Generates a vector of all ones of the given length. */
		template<class T> inline Eigen::Matrix<T,Eigen::Dynamic,1> ones(size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> v(count);
			v.setOnes();
			return v;
		}

		/** Generates a vector of all zeros of the given length. */
		template<class T> inline Eigen::Matrix<T,Eigen::Dynamic,1> zeros(size_t count)
		{
			Eigen::Matrix<T,Eigen::Dynamic,1> v(count);
			v.setZero();
			return v;
		}


		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapTo2PiInPlace(T &a)
		{
			bool was_neg = a<0;
			a = fmod(a, static_cast<T>(M_2PI) );
			if (was_neg) a+=static_cast<T>(M_2PI);
		}

		/** Modifies the given angle to translate it into the [0,2pi[ range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapTo2Pi(T a)
		{
			wrapTo2PiInPlace(a);
			return a;
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
		  */
		template <class T>
		inline T wrapToPi(T a)
		{
			return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
		}

		/** Modifies the given angle to translate it into the ]-pi,pi] range.
		  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
		  * \sa wrapToPi,wrapTo2Pi, unwrap2PiSequence
		  */
		template <class T>
		inline void wrapToPiInPlace(T &a)
		{
			a = wrapToPi(a);
		}


		/** Normalize a vector, such as its norm is the unity.
		  *  If the vector has a null norm, the output is a null vector.
		  */
		template<class VEC1,class VEC2>
		void normalize(const VEC1 &v, VEC2 &out_v)
		{
			typename VEC1::Scalar total=0;
			const size_t N = v.size();
			for (size_t i=0;i<N;i++)
				total += square(v[i]);
			total = std::sqrt(total);
			if (total)
			{
				out_v = v * (1.0/total);
			}
			else out_v.assign(v.size(),0);
		}

		/** Extract a column from a vector of vectors, and store it in another vector.
			*  - Input data can be: std::vector<vector_double>, std::deque<std::list<double> >, std::list<CArrayDouble<5> >, etc. etc.
			*  - Output is the sequence:  data[0][idx],data[1][idx],data[2][idx], etc..
			*
			*  For the sake of generality, this function does NOT check the limits in the number of column, unless it's implemented in the [] operator of each of the "rows".
			*/
		template <class VECTOR_OF_VECTORS, class VECTORLIKE>
		inline void extractColumnFromVectorOfVectors(const size_t colIndex, const VECTOR_OF_VECTORS &data, VECTORLIKE &out_column)
		{
			const size_t N = data.size();
			out_column.resize(N);
			for (size_t i=0;i<N;i++)
				out_column[i]=data[i][colIndex];
		}

		/** Computes the factorial of an integer number and returns it as a 64-bit integer number.
		  */
		uint64_t BASE_IMPEXP  factorial64(unsigned int n);

		/** Computes the factorial of an integer number and returns it as a double value (internally it uses logarithms for avoiding overflow).
		  */
		double BASE_IMPEXP  factorial(unsigned int n);

		/** Round up to the nearest power of two of a given number
		  */
		template <class T>
		T round2up(T val)
		{
			T n = 1;
			while (n < val)
			{
				n <<= 1;
				if (n<=1)
					THROW_EXCEPTION("Overflow!");
			}
			return n;
		}

		/** Round a decimal number up to the given 10'th power (eg, to 1000,100,10, and also fractions)
		  *  power10 means round up to: 1 -> 10, 2 -> 100, 3 -> 1000, ...  -1 -> 0.1, -2 -> 0.01, ...
		  */
		template <class T>
		T round_10power(T val, int power10)
		{
			long double F = ::pow((long double)10.0,-(long double)power10);
			long int t = round_long( val * F );
			return T(t/F);
		}

		/** Generates a string with the MATLAB commands required to plot an confidence interval (ellipse) for a 2D Gaussian ('float' version)..
		  *  \param cov22 The 2x2 covariance matrix
		  *  \param mean  The 2-length vector with the mean
		  *  \param stdCount How many "quantiles" to get into the area of the ellipse: 2: 95%, 3:99.97%,...
		  *  \param style A matlab style string, for colors, line styles,...
		  *  \param nEllipsePoints The number of points in the ellipse to generate
		  * \ingroup stats_grp
		  */
		std::string BASE_IMPEXP  MATLAB_plotCovariance2D(
			const CMatrixFloat  &cov22,
			const vector_float  &mean,
			const float         &stdCount,
			const std::string   &style = std::string("b"),
			const size_t        &nEllipsePoints = 30 );

		/** Generates a string with the MATLAB commands required to plot an confidence interval (ellipse) for a 2D Gaussian ('double' version).
		  *  \param cov22 The 2x2 covariance matrix
		  *  \param mean  The 2-length vector with the mean
		  *  \param stdCount How many "quantiles" to get into the area of the ellipse: 2: 95%, 3:99.97%,...
		  *  \param style A matlab style string, for colors, line styles,...
		  *  \param nEllipsePoints The number of points in the ellipse to generate
		  * \ingroup stats_grp
		  */
		std::string BASE_IMPEXP  MATLAB_plotCovariance2D(
			const CMatrixDouble  &cov22,
			const vector_double  &mean,
			const float         &stdCount,
			const std::string   &style = std::string("b"),
			const size_t        &nEllipsePoints = 30 );


		/** Efficiently compute the inverse of a 4x4 homogeneous matrix by only transposing the rotation 3x3 part and solving the translation with dot products.
		  *  This is a generic template which works with:
		  *    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class MATRIXLIKE1,class MATRIXLIKE2>
		void homogeneousMatrixInverse(const MATRIXLIKE1 &M, MATRIXLIKE2 &out_inverse_M)
		{
			MRPT_START
			ASSERT_( M.isSquare() && size(M,1)==4);

			/* Instead of performing a generic 4x4 matrix inversion, we only need to
			  transpose the rotation part, then replace the translation part by
			  three dot products. See, for example:
			 https://graphics.stanford.edu/courses/cs248-98-fall/Final/q4.html

				[ux vx wx tx] -1   [ux uy uz -dot(u,t)]
				[uy vy wy ty]      [vx vy vz -dot(v,t)]
				[uz vz wz tz]    = [wx wy wz -dot(w,t)]
				[ 0  0  0  1]      [ 0  0  0     1    ]
			*/

			out_inverse_M.setSize(4,4);

			// 3x3 rotation part:
			out_inverse_M.set_unsafe(0,0, M.get_unsafe(0,0));
			out_inverse_M.set_unsafe(0,1, M.get_unsafe(1,0));
			out_inverse_M.set_unsafe(0,2, M.get_unsafe(2,0));

			out_inverse_M.set_unsafe(1,0, M.get_unsafe(0,1));
			out_inverse_M.set_unsafe(1,1, M.get_unsafe(1,1));
			out_inverse_M.set_unsafe(1,2, M.get_unsafe(2,1));

			out_inverse_M.set_unsafe(2,0, M.get_unsafe(0,2));
			out_inverse_M.set_unsafe(2,1, M.get_unsafe(1,2));
			out_inverse_M.set_unsafe(2,2, M.get_unsafe(2,2));

			const double tx = -M.get_unsafe(0,3);
			const double ty = -M.get_unsafe(1,3);
			const double tz = -M.get_unsafe(2,3);

			const double tx_ = tx*M.get_unsafe(0,0)+ty*M.get_unsafe(1,0)+tz*M.get_unsafe(2,0);
			const double ty_ = tx*M.get_unsafe(0,1)+ty*M.get_unsafe(1,1)+tz*M.get_unsafe(2,1);
			const double tz_ = tx*M.get_unsafe(0,2)+ty*M.get_unsafe(1,2)+tz*M.get_unsafe(2,2);

			out_inverse_M.set_unsafe(0,3, tx_ );
			out_inverse_M.set_unsafe(1,3, ty_ );
			out_inverse_M.set_unsafe(2,3, tz_ );

			out_inverse_M.set_unsafe(3,0,  0);
			out_inverse_M.set_unsafe(3,1,  0);
			out_inverse_M.set_unsafe(3,2,  0);
			out_inverse_M.set_unsafe(3,3,  1);

			MRPT_END
		}
		//! \overload
		template <class IN_ROTMATRIX,class IN_XYZ, class OUT_ROTMATRIX, class OUT_XYZ>
		void homogeneousMatrixInverse(
			const IN_ROTMATRIX  & in_R,
			const IN_XYZ        & in_xyz,
			OUT_ROTMATRIX & out_R,
			OUT_XYZ       & out_xyz
			)
		{
			MRPT_START
			ASSERT_( in_R.isSquare() && size(in_R,1)==3 && in_xyz.size()==3)
			out_R.setSize(3,3);
			out_xyz.resize(3);

			// translation part:
			typedef typename IN_ROTMATRIX::Scalar T;
			const T tx = -in_xyz[0];
			const T ty = -in_xyz[1];
			const T tz = -in_xyz[2];

			out_xyz[0] = tx*in_R.get_unsafe(0,0)+ty*in_R.get_unsafe(1,0)+tz*in_R.get_unsafe(2,0);
			out_xyz[1] = tx*in_R.get_unsafe(0,1)+ty*in_R.get_unsafe(1,1)+tz*in_R.get_unsafe(2,1);
			out_xyz[2] = tx*in_R.get_unsafe(0,2)+ty*in_R.get_unsafe(1,2)+tz*in_R.get_unsafe(2,2);

			// 3x3 rotation part: transpose
			out_R = in_R.adjoint();

			MRPT_END
		}
		//! \overload
		template <class MATRIXLIKE>
		inline void homogeneousMatrixInverse(MATRIXLIKE &M)
		{
			ASSERTDEB_( M.isSquare() && size(M,1)==4);
			// translation:
			const double tx = -M(0,3);
			const double ty = -M(1,3);
			const double tz = -M(2,3);
			M(0,3) = tx*M(0,0)+ty*M(1,0)+tz*M(2,0);
			M(1,3) = tx*M(0,1)+ty*M(1,1)+tz*M(2,1);
			M(2,3) = tx*M(0,2)+ty*M(1,2)+tz*M(2,2);
			// 3x3 rotation part:
			std::swap( M(1,0),M(0,1) );
			std::swap( M(2,0),M(0,2) );
			std::swap( M(1,2),M(2,1) );
		}

		/** Assignment operator for initializing a std::vector from a C array (The vector will be automatically set to the correct size).
		  * \code
		  *	 vector_double  v;
		  *  const double numbers[] = { 1,2,3,5,6,7,8,9,10 };
		  *  loadVector( v, numbers );
		  * \endcode
		  * \note This operator performs the appropiate type castings, if required.
		  */
		template <typename T, typename At, size_t N>
		std::vector<T>& loadVector( std::vector<T> &v, At (&theArray)[N] )
		{
			MRPT_COMPILE_TIME_ASSERT(N!=0)
			v.resize(N);
			for (size_t i=0; i < N; i++)
				v[i] = static_cast<T>(theArray[i]);
			return v;
		}
		//! \overload
		template <typename Derived, typename At, size_t N>
		Eigen::EigenBase<Derived>& loadVector( Eigen::EigenBase<Derived> &v, At (&theArray)[N] )
		{
			MRPT_COMPILE_TIME_ASSERT(N!=0)
			v.derived().resize(N);
			for (size_t i=0; i < N; i++)
				(v.derived())[i] = static_cast<typename Derived::Scalar>(theArray[i]);
			return v;
		}

		/** Modify a sequence of angle values such as no consecutive values have a jump larger than PI in absolute value.
		  * \sa wrapToPi
		  */
		void unwrap2PiSequence(vector_double &x);

		/** A versatile template to build vectors on-the-fly in a style close to MATLAB's  v=[a b c d ...]
		  *  The first argument of the template is the vector length, and the second the type of the numbers.
		  *  Some examples:
		  *
		  *  \code
		  *    vector_double  = make_vector<4,double>(1.0,3.0,4.0,5.0);
		  *    vector_float   = make_vector<2,float>(-8.12, 3e4);
		  *  \endcode
		  */
		template <size_t N, typename T>
		std::vector<T> make_vector(const T val1, ...)
		{
			MRPT_COMPILE_TIME_ASSERT( N>0 )
			std::vector<T>	ret;
			ret.reserve(N);

			ret.push_back(val1);

			va_list args;
			va_start(args,val1);
			for (size_t i=1;i<N;i++)
				ret.push_back( va_arg(args,T) );

			va_end(args);
			return ret;
		}

		/**  @} */  // end of grouping container_ops_grp




		/** \defgroup mrpt_math_io Custom I/O for math containers
		  * \ingroup mrpt_base_grp */
		/** \addtogroup mrpt_math_io
		  * @{ */

		/** Saves to a plain-text file the nonzero entries of a Eigen sparse matrix, represented as a vector of triplets.
		  *  Output format is one line per entry with the format: "i j val", i:row, j:col, val:value.
		  * \tparam TRIPLET should be Eigen::Triplet<T>
		  */
		template <class TRIPLET>
		bool saveEigenSparseTripletsToFile(const std::string &sFile, std::vector<TRIPLET> &tri)
		{
#if defined(_MSC_VER) && (_MSC_VER>=1400) // Use a secure version in Visual Studio 2005+
			FILE *f;
			if (0!=::fopen_s(&f,sFile.c_str(),"wt")) f= NULL;
#else
			FILE *f= ::fopen(sFile.c_str(),"wt");
#endif

			if (!f) return false;

			for (size_t i=0;i<tri.size();i++)
				fprintf(f,"%u %u %e\n", 1+tri[i].row(), 1+tri[i].col(), tri[i].value() );

			fclose(f);
			return true;
		}

		/** @} */  // End of mrpt_math_io


	} // End of MATH namespace

} // End of namespace

#endif
