/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixD.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

using namespace mrpt;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMatrixD, CSerializable, mrpt::math)

uint8_t CMatrixD::serializeGetVersion() const { return 0; }
void CMatrixD::serializeTo(mrpt::serialization::CArchive& out) const
{
	// First, write the number of rows and columns:
	out << static_cast<uint32_t>(rows()) << static_cast<uint32_t>(cols());

	// Since mrpt-1.9.9, dynamic matrices are stored as a contiguous vector:
	if (rows() > 0 && cols() > 0)
		out.WriteBufferFixEndianness<value_type>(
			&(*this)(0, 0), cols() * rows());
}
void CMatrixD::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
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
				in.ReadBufferFixEndianness<value_type>(
					&(*this)(0, 0), nRows * nCols);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/** Serialize CSerializable Object to CSchemeArchiveBase derived object*/
void CMatrixD::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["nrows"] = static_cast<uint32_t>(this->rows());
	out["ncols"] = static_cast<uint32_t>(this->cols());
	out["data"] = this->inMatlabFormat();
}
/** Serialize CSchemeArchiveBase derived object to CSerializable Object*/
void CMatrixD::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
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