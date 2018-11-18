/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
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

CObservationRGBD360::~CObservationRGBD360()
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	mempool_donate_xyz_buffers(*this);
	mempool_donate_range_matrix(*this);
#endif
}

uint8_t CObservationRGBD360::serializeGetVersion() const { return 0; }
void CObservationRGBD360::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The data
	out << maxRange << sensorPose;

	//		out << hasPoints3D;
	//		if (hasPoints3D)
	//		{
	//			uint32_t N = points3D_x.size();
	//			out << N;
	//			if (N)
	//			{
	//				out.WriteBufferFixEndianness( &points3D_x[0], N );
	//				out.WriteBufferFixEndianness( &points3D_y[0], N );
	//				out.WriteBufferFixEndianness( &points3D_z[0], N );
	//			}
	//		}
	//
	out << hasRangeImage;
	if (hasRangeImage)
		for (const auto& rangeImage : rangeImages) out << rangeImage;
	out << hasIntensityImage;
	if (hasIntensityImage)
		for (const auto& intensityImage : intensityImages)
			out << intensityImage;
	//		out << hasConfidenceImage; if (hasConfidenceImage) out <<
	// confidenceImage;
	for (auto t : timestamps) out << t;
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
		{
			in >> maxRange >> sensorPose;
			in >> hasRangeImage;
			if (hasRangeImage)
				for (auto& rangeImage : rangeImages)
				{
#ifdef COBS3DRANGE_USE_MEMPOOL
					// We should call "rangeImage_setSize()" to exploit the
					// mempool:
					this->rangeImage_setSize(240, 320, i);
#endif
					in >> rangeImage;
				}

			in >> hasIntensityImage;
			if (hasIntensityImage)
				for (auto& intensityImage : intensityImages)
					in >> intensityImage;

			//      in >> hasConfidenceImage;
			//      if (hasConfidenceImage)
			//        in >> confidenceImage;

			//      in >> cameraParams;

			for (auto& t : timestamps) in >> t;
			in >> stdError;
			in >> timestamp;
			in >> sensorLabel;

			in >> m_points3D_external_stored >> m_points3D_external_file;
			in >> m_rangeImage_external_stored >> m_rangeImage_external_file;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory
// pooling to speed-up the memory allocation.
void CObservationRGBD360::rangeImage_setSize(
	const int H, const int W, const unsigned sensor_id)
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	// Request memory from the memory pool:
	TMyRangesMemPool* pool = TMyRangesMemPool::getInstance();
	if (pool)
	{
		CObservationRGBD360_Ranges_MemPoolParams mem_params;
		mem_params.H = H;
		mem_params.W = W;

		CObservationRGBD360_Ranges_MemPoolData* mem_block =
			pool->request_memory(mem_params);

		if (mem_block)
		{  // Take the memory via swaps:
			rangeImage.swap(mem_block->rangeImage);
			delete mem_block;
			return;
		}
	}
// otherwise, continue with the normal method:
#endif
	// Fall-back to normal method:
	rangeImages[sensor_id].setSize(H, W);
}

void CObservationRGBD360::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
}
