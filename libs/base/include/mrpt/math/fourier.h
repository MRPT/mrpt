/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_fourier_H
#define  mrpt_math_fourier_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

/*---------------------------------------------------------------
		Namespace
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{

		/** \addtogroup fourier_grp Fourier transform functions
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Computes the FFT of a 2^N-size vector of real numbers, and returns the Re+Im+Magnitude parts.
		  * \sa fft2_real
		  */
		void BASE_IMPEXP  fft_real(	CVectorFloat	&in_realData,
								CVectorFloat	&out_FFT_Re,
								CVectorFloat	&out_FFT_Im,
								CVectorFloat	&out_FFT_Mag );

		/** Compute the 2D Discrete Fourier Transform (DFT) of a real matrix, returning the real and imaginary parts separately.
		  * \param in_data The N_1xN_2 matrix.
		  * \param out_real The N_1xN_2 output matrix which will store the real values (user has not to initialize the size of this matrix).
		  * \param out_imag The N_1xN_2 output matrix which will store the imaginary values (user has not to initialize the size of this matrix).
		  * \sa fft_real, ifft2_read, fft2_complex
		  *  If the dimensions of the matrix are powers of two, the fast fourier transform (FFT) is used instead of the general algorithm.
		  */
		void BASE_IMPEXP  dft2_real(
			const CMatrixFloat &in_data,
			CMatrixFloat		&out_real,
			CMatrixFloat		&out_imag );

		/** Compute the 2D inverse Discrete Fourier Transform (DFT)
		  * \param in_real The N_1xN_2 input matrix with real values.
		  * \param in_imag The N_1xN_2 input matrix with imaginary values.
		  * \param out_data The N_1xN_2 output matrix (user has not to initialize the size of this matrix).
		  *  Note that the real and imaginary parts of the FFT will NOT be checked to assure that they represent the transformation
		  *    of purely real data.
		  *  If the dimensions of the matrix are powers of two, the fast fourier transform (FFT) is used instead of the general algorithm.
		  * \sa fft_real, fft2_real
		  */
		void BASE_IMPEXP  idft2_real(
			const CMatrixFloat	&in_real,
			const CMatrixFloat	&in_imag,
			CMatrixFloat		&out_data );

		/** Compute the 2D Discrete Fourier Transform (DFT) of a complex matrix, returning the real and imaginary parts separately.
		  * \param in_real The N_1xN_2 matrix with the real part.
		  * \param in_imag The N_1xN_2 matrix with the imaginary part.
		  * \param out_real The N_1xN_2 output matrix which will store the real values (user has not to initialize the size of this matrix).
		  * \param out_imag The N_1xN_2 output matrix which will store the imaginary values (user has not to initialize the size of this matrix).
		  *  If the dimensions of the matrix are powers of two, the fast fourier transform (FFT) is used instead of the general algorithm.
		  * \sa fft_real, idft2_complex,dft2_real
		  */
		void BASE_IMPEXP  dft2_complex(
			const CMatrixFloat &in_real,
			const CMatrixFloat &in_imag,
			CMatrixFloat		&out_real,
			CMatrixFloat		&out_imag);

		/** Compute the 2D inverse Discrete Fourier Transform (DFT).
		  * \param in_real The N_1xN_2 input matrix with real values, where both dimensions MUST BE powers of 2.
		  * \param in_imag The N_1xN_2 input matrix with imaginary values, where both dimensions MUST BE powers of 2.
		  * \param out_real The N_1xN_2 output matrix for real part (user has not to initialize the size of this matrix).
		  * \param out_imag The N_1xN_2 output matrix for imaginary part (user has not to initialize the size of this matrix).
		  * \sa fft_real, dft2_real,dft2_complex
		  *  If the dimensions of the matrix are powers of two, the fast fourier transform (FFT) is used instead of the general algorithm.
		  */
		void BASE_IMPEXP  idft2_complex(
			const CMatrixFloat	&in_real,
			const CMatrixFloat	&in_imag,
			CMatrixFloat		&out_real,
			CMatrixFloat		&out_imag );


		/** Correlation of two matrixes using 2D FFT
		  */
		void  BASE_IMPEXP  cross_correlation_FFT(
			const CMatrixFloat	&A,
			const CMatrixFloat	&B,
			CMatrixFloat		&out_corr );

		/** @} */

	} // End of MATH namespace

} // End of namespace

#endif
