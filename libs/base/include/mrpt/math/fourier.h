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
		void BASE_IMPEXP  fft_real(	vector_float	&in_realData,
								vector_float	&out_FFT_Re,
								vector_float	&out_FFT_Im,
								vector_float	&out_FFT_Mag );

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
