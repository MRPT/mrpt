/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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



#include <mrpt/slam/CObservationImage.h>

#include <iostream>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationImage, CObservation,mrpt::slam)

/** Constructor
 */
CObservationImage::CObservationImage( void *iplImage  ) :
	cameraPose(),
	image( iplImage )
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationImage::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 4;
	else
	{
		// The data
		out << cameraPose << cameraParams << image
		    << timestamp
		    << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationImage::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		{
			in >> cameraPose;

			if (version>=4)
			{
				in >> cameraParams;
			}
			else
			{
				CMatrix	 intrinsicParams, distortionParams;
				in >> distortionParams >> intrinsicParams;

				if (size(distortionParams,1)==1 && size(distortionParams,2)==5)
				{
					const CMatrixDouble15 p = distortionParams.cast<double>();
					cameraParams.setDistortionParamsVector(p);
				}
				else 	cameraParams.dist.assign(0);

				cameraParams.intrinsicParams = intrinsicParams.block(0,0,3,3).cast<double>();
			}

			in >> image;

			if (version>=1)
				in >> timestamp;

			if (version>=2)
			{
				if (version<4)
					in >> cameraParams.focalLengthMeters ;
			}
			else
				cameraParams.focalLengthMeters = 0.002;

			if (version>=3)
					in >> sensorLabel;
			else	sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


/*---------------------------------------------------------------
						getRectifiedImage
 ---------------------------------------------------------------*/
void  CObservationImage::getRectifiedImage( CImage &out_img ) const
{
	image.rectifyImage(out_img, cameraParams.intrinsicParams, cameraParams.getDistortionParamsAsVector() );
}
