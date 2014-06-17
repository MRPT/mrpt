/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::slam;
using namespace mrpt::math;
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

/** Populates a TStereoCamera structure with the parameters in \a leftCamera, \a rightCamera and \a rightCameraPose */
void CObservationStereoImages::getStereoCameraParams(mrpt::utils::TStereoCamera &out_params) const
{
	out_params.leftCamera  = this->leftCamera;
	out_params.rightCamera = this->rightCamera;
	out_params.rightCameraPose = this->rightCameraPose;
}

/** Sets \a leftCamera, \a rightCamera and \a rightCameraPose from a TStereoCamera structure */
void CObservationStereoImages::setStereoCameraParams(const mrpt::utils::TStereoCamera &in_params)
{
	this->leftCamera      = in_params.leftCamera;
	this->rightCamera     = in_params.rightCamera;
	this->rightCameraPose = in_params.rightCameraPose;
}

/** This method only checks whether ALL the distortion parameters in \a leftCamera are set to zero, which is
  * the convention in MRPT to denote that this pair of stereo images has been rectified.
  */
bool CObservationStereoImages::areImagesRectified() const
{
	return (leftCamera.dist.array()==0).all();
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
