/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/lightweight_geom_data.h>
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
	out << (uint32_t)rows() << (uint32_t)cols();

	if (rows() > 0 && cols() > 0)
		for (Index i = 0; i < rows(); i++)
			out.WriteBufferFixEndianness<Scalar>(&coeff(i, 0), cols());
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
				for (Index i = 0; i < rows(); i++)
					in.ReadBufferFixEndianness<Scalar>(&coeffRef(i, 0), nCols);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/** Serialize CSerializable Object to CSchemeArchiveBase derived object*/
void CMatrixD::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["nrows"] = this->rows();
	out["ncols"] = this->cols();
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
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}
