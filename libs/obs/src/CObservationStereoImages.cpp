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

#include <mrpt/slam/CObservationStereoImages.h>
//#include <mrpt/slam/CLandmarksMap.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationStereoImages, CObservation,mrpt::slam)

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CObservationStereoImages::CObservationStereoImages( void *iplImageLeft,void *iplImageRight, void *iplImageDisparity,bool ownMemory ) :
	imageLeft( UNINITIALIZED_IMAGE ),
	imageRight( UNINITIALIZED_IMAGE ),
	imageDisparity( UNINITIALIZED_IMAGE ),
	hasImageDisparity( iplImageDisparity!=NULL ),
	hasImageRight( iplImageRight!=NULL )
{
	if (iplImageLeft)
		ownMemory ? imageLeft.setFromIplImage(iplImageLeft) : imageLeft.loadFromIplImage(iplImageLeft);
	if (iplImageRight)
		ownMemory ? imageRight.setFromIplImage(iplImageRight) : imageRight.loadFromIplImage(iplImageRight);
	if (iplImageDisparity)
		ownMemory ? imageDisparity.setFromIplImage(iplImageDisparity) : imageDisparity.loadFromIplImage(iplImageDisparity);
}

/*---------------------------------------------------------------
					Default Constructor
 ---------------------------------------------------------------*/
CObservationStereoImages::CObservationStereoImages( ) :
	hasImageDisparity(false),
	hasImageRight(true)
{
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CObservationStereoImages::~CObservationStereoImages(  )
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationStereoImages::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 6 ;
	else
	{
		// The data
		out << cameraPose << leftCamera << rightCamera
			<< imageLeft;

		out << hasImageDisparity << hasImageRight;

		if (hasImageRight)
			out << imageRight;

		if (hasImageDisparity)
			out << imageDisparity;

		out << timestamp;
		out << rightCameraPose;
		out << sensorLabel;

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationStereoImages::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 6:
		{
			in >> cameraPose >> leftCamera >> rightCamera
				>> imageLeft;

			in >> hasImageDisparity >> hasImageRight;

			if (hasImageRight)
				in >> imageRight;

			if (hasImageDisparity)
				in >> imageDisparity;

			in >> timestamp;
			in >> rightCameraPose;
			in >> sensorLabel;
		}
		break;

	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		{
			// This, for backwards compatibility before version 6:
			hasImageRight = true;
			hasImageDisparity = false;


			if( version < 5 )
			{
				CPose3D aux;
				in >> aux;
				cameraPose = CPose3DQuat( aux );
			}

			if( version >= 5 )
			{
				in >> cameraPose >> leftCamera >> rightCamera;
			}
			else
			{
				CMatrix intParams;
				in >> intParams;																// Get the intrinsic params
				leftCamera.intrinsicParams = CMatrixDouble33(intParams);				// Set them to both cameras
				rightCamera.intrinsicParams = CMatrixDouble33(intParams);				// ... distortion parameters are set to zero
			}

			in >> imageLeft >> imageRight;														// For all the versions

			if(version >= 1) in >> timestamp; else timestamp = INVALID_TIMESTAMP;				// For version 1 to 5
			if(version >= 2)
			{
				if(version < 5)
				{
					CPose3D aux;
					in >> aux;
					rightCameraPose = CPose3DQuat( aux );
			}
			else
					in >> rightCameraPose;
			}
			else
				rightCameraPose = CPose3DQuat( 0.10f, 0, 0, mrpt::math::CQuaternionDouble(1,0,0,0)  );	// For version 1 to 5

			if(version >= 3 && version < 5)														// For versions 3 & 4
			{
				double foc;
				in >> foc;																		// Get the focal length in meters
				leftCamera.focalLengthMeters = rightCamera.focalLengthMeters = foc;				// ... and set it to both cameras
			}
			else
				if( version < 3 )
					leftCamera.focalLengthMeters = rightCamera.focalLengthMeters = 0.002;		// For version 0, 1 & 2 (from version 5, this parameter is included in the TCamera objects)

			if(version >= 4) in >> sensorLabel; else sensorLabel = "";							// For version 1 to 5

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


// Do an efficient swap of all data members of this object with "o".
void CObservationStereoImages::swap( CObservationStereoImages &o)
{
	CObservation::swap(o);

	imageLeft.swap(o.imageLeft);
	imageRight.swap(o.imageRight);
	imageDisparity.swap(o.imageDisparity);

	std::swap(hasImageDisparity, o.hasImageDisparity);
	std::swap(hasImageRight, o.hasImageRight);

	std::swap(leftCamera,o.leftCamera);
	std::swap(rightCamera, o.rightCamera);

	std::swap(cameraPose, o.cameraPose);
	std::swap(rightCameraPose, o.rightCameraPose);
}
