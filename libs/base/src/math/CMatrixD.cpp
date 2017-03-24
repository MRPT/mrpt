/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixD, CSerializable, mrpt::math)

/** Constructor from a TPose2D, which generates a 3x1 matrix \f$ [x y \phi]^T \f$  */
CMatrixD::CMatrixD( const TPose2D &p) : CMatrixDouble(p) {}
/** Constructor from a TPose3D, which generates a 6x1 matrix \f$ [x y z yaw pitch roll]^T \f$  */
CMatrixD::CMatrixD( const TPose3D &p) : CMatrixDouble(p) {}
/** Constructor from a TPoint2D, which generates a 2x1 matrix \f$ [x y]^T \f$ */
CMatrixD::CMatrixD( const TPoint2D &p) : CMatrixDouble(p) {}
/** Constructor from a TPoint3D, which generates a 3x1 matrix \f$ [x y z]^T \f$ */
CMatrixD::CMatrixD( const TPoint3D &p) : CMatrixDouble(p) {}


/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMatrixD::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		// First, write the number of rows and columns:
		out << (uint32_t)rows() << (uint32_t)cols();

		if (rows()>0 && cols()>0)
			for (Index i=0;i<rows();i++)
				out.WriteBufferFixEndianness<Scalar>(&coeff(i,0),cols());
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CMatrixD::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t nRows,nCols;

			// First, write the number of rows and columns:
			in >> nRows >> nCols;

			setSize(nRows,nCols);

			if (nRows>0 && nCols>0)
				for (Index i=0;i<rows();i++)
					in.ReadBufferFixEndianness<Scalar>(&coeffRef(i,0),nCols);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

