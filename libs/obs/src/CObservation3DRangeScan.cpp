/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers



#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>

#include <mrpt/math/utils.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DRangeScan, CObservation,mrpt::slam)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservation3DRangeScan::CObservation3DRangeScan( ) :
	hasPoints3D(false),
	hasRangeImage(false),
	hasIntensityImage(false),
	hasConfidenceImage(false),
	maxRange( 5.0f ),
	sensorPose(),
	stdError( 0.01f )
{
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CObservation3DRangeScan::~CObservation3DRangeScan()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		// The data
		out << maxRange << sensorPose;

		// Old in v0:
		// uint32_t	N = scan_x.size();
		// out << N;
		//	out.WriteBuffer( &scan_x[0], sizeof(scan_x[0])*N );
		//	out.WriteBuffer( &scan_y[0], sizeof(scan_y[0])*N );
		//	out.WriteBuffer( &scan_z[0], sizeof(scan_z[0])*N );
		//	out.WriteBuffer( &validRange[0],sizeof(validRange[0])*N );

		out << hasPoints3D;
		if (hasPoints3D)
		{
			uint32_t N = points3D_x.size();
			out << N;
			if (N)
			{
				out.WriteBuffer( &points3D_x[0], sizeof(points3D_x[0])*N );
				out.WriteBuffer( &points3D_y[0], sizeof(points3D_y[0])*N );
				out.WriteBuffer( &points3D_z[0], sizeof(points3D_z[0])*N );
			}
		}

		out << hasRangeImage; if (hasRangeImage) out << rangeImage;
		out << hasIntensityImage; if (hasIntensityImage)  out << intensityImage;
		out << hasConfidenceImage; if (hasConfidenceImage) out << confidenceImage;

		out << stdError;
		out << timestamp;
		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t		N;

			in >> maxRange >> sensorPose;

			if (version>0)
				in >> hasPoints3D;
			else hasPoints3D = true;

			if (hasPoints3D)
			{
				in >> N;
				points3D_x.resize(N);
				points3D_y.resize(N);
				points3D_z.resize(N);

				if (N)
				{
					in.ReadBuffer( &points3D_x[0], sizeof(points3D_x[0])*N);
					in.ReadBuffer( &points3D_y[0], sizeof(points3D_x[0])*N);
					in.ReadBuffer( &points3D_z[0], sizeof(points3D_x[0])*N);

					if (version==0)
					{
						vector<char> validRange(N);  // for v0.
						in.ReadBuffer( &validRange[0], sizeof(validRange[0])*N );
					}
				}
			}
			else
			{
				points3D_x.clear();
				points3D_y.clear();
				points3D_z.clear();
			}

			if (version>=1)
			{
				in >> hasRangeImage;
				if (hasRangeImage)
					in >> rangeImage;

				in >> hasIntensityImage;
				if (hasIntensityImage)
					in >>intensityImage;

				in >> hasConfidenceImage;
				if (hasConfidenceImage)
					in >> confidenceImage;
			}

			in >> stdError;
			in >> timestamp;
			in >> sensorLabel;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

void CObservation3DRangeScan::swap(CObservation3DRangeScan &o)
{
	CObservation::swap(o);

	std::swap(hasPoints3D,o.hasPoints3D);
	points3D_x.swap(o.points3D_x);
	points3D_y.swap(o.points3D_y);
	points3D_z.swap(o.points3D_z);

	std::swap(hasRangeImage,o.hasRangeImage);
	rangeImage.swap(o.rangeImage);

	std::swap(hasIntensityImage,o.hasIntensityImage);
	intensityImage.swap(o.intensityImage);

	std::swap(hasConfidenceImage,o.hasConfidenceImage);
	confidenceImage.swap(o.confidenceImage);

	std::swap(maxRange, o.maxRange);
	std::swap(sensorPose, o.sensorPose);
	std::swap(stdError, o.stdError);
}

void CObservation3DRangeScan::unload()
{
	intensityImage.unload();
	confidenceImage.unload();
}
