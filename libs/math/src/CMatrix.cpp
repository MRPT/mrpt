/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::serialization;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrix, CSerializable, mrpt::math)

CMatrix::CMatrix(const TPose2D& p) : CMatrixFloat(p) {}
CMatrix::CMatrix(const TPose3D& p) : CMatrixFloat(p) {}
CMatrix::CMatrix(const TPoint2D& p) : CMatrixFloat(p) {}
CMatrix::CMatrix(const TPoint3D& p) : CMatrixFloat(p) {}

uint8_t CMatrix::serializeGetVersion() const { return 0; }
void CMatrix::serializeTo(mrpt::serialization::CArchive& out) const
{
	// First, write the number of rows and columns:
	out << (uint32_t)rows() << (uint32_t)cols();

	if (rows() > 0 && cols() > 0)
		for (Index i = 0; i < rows(); i++)
			out.WriteBufferFixEndianness<Scalar>(&coeff(i, 0), cols());
}

void CMatrix::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t nRows, nCols;

			// First, write the number of rows and columns:
			in >> nRows >> nCols;

			setSize(nRows, nCols);

			if (nRows > 0 && nCols > 0)
				for (Index i = 0; i < rows(); i++)
					in.ReadBufferFixEndianness<Scalar>(&coeffRef(i, 0), nCols);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
