/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRGBD360, CObservation, mrpt::obs)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservationRGBD360::CObservationRGBD360() : sensorPose() {}
/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CObservationRGBD360::~CObservationRGBD360() {}

uint8_t CObservationRGBD360::serializeGetVersion() const { return 1; }
void CObservationRGBD360::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The data
	out << maxRange << sensorPose;

	out << hasRangeImage;
	if (hasRangeImage)
		for (const auto& ri : rangeImages)
		{
			out.WriteAs<uint32_t>(ri.rows());
			out.WriteAs<uint32_t>(ri.cols());
			if (ri.size() == 0) continue;
			out.WriteBufferFixEndianness<uint16_t>(ri.data(), ri.size());
		}
	out << hasIntensityImage;
	if (hasIntensityImage)
		for (const auto& intensityImage : intensityImages)
			out << intensityImage;
	//		out << hasConfidenceImage; if (hasConfidenceImage) out <<
	// confidenceImage;
	for (auto t : timestamps)
		out << t;
	//
	out << stdError;
	out << timestamp;
	out << sensorLabel;

	out << m_points3D_external_stored << m_points3D_external_file;
	out << m_rangeImage_external_stored << m_rangeImage_external_file;
}

void CObservationRGBD360::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			THROW_EXCEPTION(
				"Import from serialization version 0 not implemented!");
			break;
		case 1:
		{
			in >> maxRange >> sensorPose;
			in >> hasRangeImage;
			if (hasRangeImage)
				for (auto& ri : rangeImages)
				{
					const auto rows = in.ReadAs<uint32_t>();
					const auto cols = in.ReadAs<uint32_t>();
					ri.setSize(rows, cols);
					in.ReadBufferFixEndianness<uint16_t>(ri.data(), ri.size());
				}

			in >> hasIntensityImage;
			if (hasIntensityImage)
				for (auto& intensityImage : intensityImages)
					in >> intensityImage;

			for (auto& t : timestamps)
				in >> t;
			in >> stdError;
			in >> timestamp;
			in >> sensorLabel;

			in >> m_points3D_external_stored >> m_points3D_external_file;
			in >> m_rangeImage_external_stored >> m_rangeImage_external_file;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory
// pooling to speed-up the memory allocation.
void CObservationRGBD360::rangeImage_setSize(
	const int H, const int W, const unsigned sensor_id)
{
	// Fall-back to normal method:
	rangeImages[sensor_id].setSize(H, W);
}

void CObservationRGBD360::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
}
