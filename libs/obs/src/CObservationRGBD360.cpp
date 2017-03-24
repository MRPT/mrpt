/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/poses/CPosePDF.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTimeLogger.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRGBD360, CObservation, mrpt::obs)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservationRGBD360::CObservationRGBD360( ) :
	m_points3D_external_stored(false),
	m_rangeImage_external_stored(false),
//	hasPoints3D(false),
	hasRangeImage(false),
//	range_is_depth(true),
	hasIntensityImage(false),
//	hasConfidenceImage(false),
//	cameraParams(),
	maxRange( 10.0f ),
	sensorPose(),
	stdError( 0.01f )
{
}

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

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRGBD360::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
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
		out << hasRangeImage; if (hasRangeImage) for (unsigned i=0; i < NUM_SENSORS; i++) out << rangeImages[i];
		out << hasIntensityImage; if (hasIntensityImage) for (unsigned i=0; i < NUM_SENSORS; i++) out << intensityImages[i];
//		out << hasConfidenceImage; if (hasConfidenceImage) out << confidenceImage;
		for (unsigned i=0; i < NUM_SENSORS; i++) out << timestamps[i];
//
		out << stdError;
		out << timestamp;
		out << sensorLabel;

		out << m_points3D_external_stored << m_points3D_external_file;
		out << m_rangeImage_external_stored << m_rangeImage_external_file;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRGBD360::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> maxRange >> sensorPose;

      in >> hasRangeImage;
      if (hasRangeImage) for (unsigned i=0; i < NUM_SENSORS; i++)
      {
#ifdef COBS3DRANGE_USE_MEMPOOL
        // We should call "rangeImage_setSize()" to exploit the mempool:
        this->rangeImage_setSize(240,320,i);
#endif
        in >> rangeImages[i];
      }

      in >> hasIntensityImage;
      if (hasIntensityImage) for (unsigned i=0; i < NUM_SENSORS; i++)
        in >>intensityImages[i];

//      in >> hasConfidenceImage;
//      if (hasConfidenceImage)
//        in >> confidenceImage;

//      in >> cameraParams;

      for (unsigned i=0; i < NUM_SENSORS; i++) in >> timestamps[i];
      in >> stdError;
      in >> timestamp;
      in >> sensorLabel;

      in >> m_points3D_external_stored >> m_points3D_external_file;
      in >> m_rangeImage_external_stored >> m_rangeImage_external_file;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

// Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.
void CObservationRGBD360::rangeImage_setSize(const int H, const int W, const unsigned sensor_id)
{
#ifdef COBS3DRANGE_USE_MEMPOOL
	// Request memory from the memory pool:
	TMyRangesMemPool *pool = TMyRangesMemPool::getInstance();
	if (pool)
	{
		CObservationRGBD360_Ranges_MemPoolParams mem_params;
		mem_params.H = H;
		mem_params.W = W;

		CObservationRGBD360_Ranges_MemPoolData *mem_block = pool->request_memory(mem_params);

		if (mem_block)
		{	// Take the memory via swaps:
			rangeImage.swap(mem_block->rangeImage);
			delete mem_block;
			return;
		}
	}
	// otherwise, continue with the normal method:
#endif
	// Fall-back to normal method:
	rangeImages[sensor_id].setSize(H,W);
}

void CObservationRGBD360::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

}

