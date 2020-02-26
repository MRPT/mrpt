/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixF.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::serialization;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixF, CSerializable, mrpt::math)

uint8_t CMatrixF::serializeGetVersion() const { return 0; }
void CMatrixF::serializeTo(mrpt::serialization::CArchive& out) const
{
	// First, write the number of rows and columns:
	out.WriteAs<uint32_t>(rows()).WriteAs<uint32_t>(cols());

	// Since mrpt-1.9.9, dynamic matrices are stored as a contiguous vector:
	if (rows() > 0 && cols() > 0)
		out.WriteBufferFixEndianness<value_type>(
			&(*this)(0, 0), cols() * rows());
}

void CMatrixF::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// Read the number of rows and columns:
			const uint32_t nRows = in.ReadAs<uint32_t>();
			const uint32_t nCols = in.ReadAs<uint32_t>();

			setSize(nRows, nCols);

			if (nRows > 0 && nCols > 0)
				in.ReadBufferFixEndianness<value_type>(
					&(*this)(0, 0), nRows * nCols);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/** Serialize CSerializable Object to CSchemeArchiveBase derived object*/
void CMatrixF::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["nrows"] = static_cast<uint32_t>(this->rows());
	out["ncols"] = static_cast<uint32_t>(this->cols());
	out["data"] = this->inMatlabFormat();
}
/** Serialize CSchemeArchiveBase derived object to CSerializable Object*/
void CMatrixF::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			this->fromMatlabStringFormat(static_cast<std::string>(in["data"]));
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}
