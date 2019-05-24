/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixB.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixB, CSerializable, mrpt::math)

uint8_t CMatrixB::serializeGetVersion() const { return 0; }
void CMatrixB::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(sizeof(bool));

	// First, write the number of rows and columns:
	out.WriteAs<uint32_t>(rows());
	out.WriteAs<uint32_t>(cols());

	if (rows() > 0 && cols() > 0)
		out.WriteBuffer(&(*this)(0, 0), sizeof(bool) * cols() * rows());
}

void CMatrixB::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t size_bool;
			in >> size_bool;
			if (size_bool != sizeof(bool))
				THROW_EXCEPTION(
					"Error: size of 'bool' is different in serialized data!");

			uint32_t nRows, nCols;

			// First, write the number of rows and columns:
			in >> nRows >> nCols;

			setSize(nRows, nCols);

			if (nRows > 0 && nCols > 0)
				in.ReadBuffer(&(*this)(0, 0), sizeof(bool) * cols() * rows());
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
