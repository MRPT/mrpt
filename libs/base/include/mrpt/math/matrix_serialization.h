/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CStream.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

/** \file matrix_serialization.h
  * This file implements matrix/vector text and binary serialization */
namespace mrpt
{
	namespace math
	{
		/** \addtogroup container_ops_grp
		  *  @{ */

		/** @name Operators for binary streaming of MRPT matrices
		    @{ */

		/** Read operator from a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in, CMatrixFixedNumeric<float,NROWS,NCOLS> & M) {
			CMatrix  aux;
			in.ReadObject(&aux);
			ASSERTMSG_(M.cols()==aux.cols() && M.rows()==aux.rows(), format("Size mismatch: deserialized is %ux%u, expected is %ux%u",(unsigned)aux.getRowCount(),(unsigned)aux.getColCount(),(unsigned)NROWS,(unsigned)NCOLS))
			M = aux;
			return in;
		}
		/** Read operator from a CStream. The format is compatible with that of CMatrix & CMatrixD */
		template <size_t NROWS,size_t NCOLS>
		mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in, CMatrixFixedNumeric<double,NROWS,NCOLS> & M) {
			CMatrixD  aux;
			in.ReadObject(&aux);
			ASSERTMSG_(M.cols()==aux.cols() && M.rows()==aux.rows(), format("Size mismatch: deserialized is %ux%u, expected is %ux%u",(unsigned)aux.getRowCount(),(unsigned)aux.getColCount(),(unsigned)NROWS,(unsigned)NCOLS))
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

		/** @} */  // end MRPT matrices stream operators


		/** @name Operators for text streaming of MRPT matrices
		    @{ */


		/** Dumps the matrix to a text ostream, adding a final "\n" to Eigen's default output. */
		template <typename T,size_t NROWS,size_t NCOLS>
		inline std::ostream & operator << (std::ostream & s, const CMatrixFixedNumeric<T,NROWS,NCOLS>& m)
		{
			Eigen::IOFormat  fmt; fmt.matSuffix="\n";
			return s << m.format(fmt);
		}

		/** Dumps the matrix to a text ostream, adding a final "\n" to Eigen's default output. */
		template<typename T>
		inline std::ostream & operator << (std::ostream & s, const CMatrixTemplateNumeric<T>& m)
		{
			Eigen::IOFormat  fmt; fmt.matSuffix="\n";
			return s << m.format(fmt);
		}

		/** @} */  // end MRPT matrices stream operators

		/**  @} */  // end of grouping
	} // End of math namespace
} // End of mrpt namespace
