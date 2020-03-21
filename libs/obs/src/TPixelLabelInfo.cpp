/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt::obs;
using plib = TPixelLabelInfoBase;

// Define here, not in the .h to allow the vtable to be created here.
TPixelLabelInfoBase::~TPixelLabelInfoBase() = default;

void plib::writeToStream(mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 1;  // for possible future changes.
	out << version;
	// 1st: Save number MAX_NUM_DIFFERENT_LABELS so we can reconstruct the
	// object in the class factory later on.
	out << BITFIELD_BYTES;
	// 2nd: data-specific serialization:
	this->internal_writeToStream(out);
}

template <unsigned int BYTES_REQUIRED_>
void TPixelLabelInfo<BYTES_REQUIRED_>::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	{
		uint32_t nR, nC;
		in >> nR >> nC;
		pixelLabels.resize(nR, nC);
		for (uint32_t c = 0; c < nC; c++)
			for (uint32_t r = 0; r < nR; r++) in >> pixelLabels.coeffRef(r, c);
	}
	in >> pixelLabelNames;
}
template <unsigned int BYTES_REQUIRED_>
void TPixelLabelInfo<BYTES_REQUIRED_>::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	{
		const auto nR = static_cast<uint32_t>(pixelLabels.rows());
		const auto nC = static_cast<uint32_t>(pixelLabels.cols());
		out << nR << nC;
		for (uint32_t c = 0; c < nC; c++)
			for (uint32_t r = 0; r < nR; r++) out << pixelLabels.coeff(r, c);
	}
	out << pixelLabelNames;
}

// Deserialization and class factory. All in one, ladies and gentlemen
TPixelLabelInfoBase* plib::readAndBuildFromStream(
	mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;

	switch (version)
	{
		case 1:
		{
			// 1st: Read NUM BYTES
			uint8_t bitfield_bytes;
			in >> bitfield_bytes;

			// Hand-made class factory. May be a good solution if there will be
			// not too many different classes:
			TPixelLabelInfoBase* new_obj = nullptr;
			switch (bitfield_bytes)
			{
				case 1:
					new_obj = new TPixelLabelInfo<1>();
					break;
				case 2:
					new_obj = new TPixelLabelInfo<2>();
					break;
				case 3:
				case 4:
					new_obj = new TPixelLabelInfo<4>();
					break;
				case 5:
				case 6:
				case 7:
				case 8:
					new_obj = new TPixelLabelInfo<8>();
					break;
				default:
					throw std::runtime_error(
						"Unknown type of pixelLabel inner class while "
						"deserializing!");
			};
			// 2nd: data-specific serialization:
			new_obj->internal_readFromStream(in);

			return new_obj;
		}
		break;

		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
			break;
	};
}

template <unsigned int BYTES_REQUIRED_>
void TPixelLabelInfo<BYTES_REQUIRED_>::Print(std::ostream& out) const
{
	{
		const auto nR = static_cast<uint32_t>(pixelLabels.rows());
		const auto nC = static_cast<uint32_t>(pixelLabels.cols());
		out << "Number of rows: " << nR << "\n";
		out << "Number of cols: " << nC << "\n";
		out << "Matrix of labels:\n";
		for (uint32_t c = 0; c < nC; c++)
		{
			for (uint32_t r = 0; r < nR; r++)
				out << pixelLabels.coeff(r, c) << " ";

			out << std::endl;
		}
	}
	out << std::endl;
	out << "Label indices and names: " << std::endl;
	std::map<uint32_t, std::string>::const_iterator it;
	for (it = pixelLabelNames.begin(); it != pixelLabelNames.end(); it++)
		out << it->first << " " << it->second << std::endl;
}

// Explicit instantiations:
template void TPixelLabelInfo<1>::Print(std::ostream& out) const;
template void TPixelLabelInfo<2>::Print(std::ostream& out) const;
template void TPixelLabelInfo<4>::Print(std::ostream& out) const;
template void TPixelLabelInfo<8>::Print(std::ostream& out) const;
