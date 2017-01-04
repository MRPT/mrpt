/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers
#include <mrpt/obs/CObservationStereoImagesFeatures.h>

#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationStereoImagesFeatures, CObservation,mrpt::obs)

 CObservationStereoImagesFeatures::CObservationStereoImagesFeatures( ) :
	cameraLeft(),
	cameraRight(),
	rightCameraPose(),
	cameraPoseOnRobot()
	{}

CObservationStereoImagesFeatures::CObservationStereoImagesFeatures(
	const CMatrixDouble33 &iPLeft, const CMatrixDouble33 &iPRight,
	const CArrayDouble<5> &dPLeft, const CArrayDouble<5> &dPRight,
	const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot )
{
	cameraLeft.intrinsicParams	= iPLeft;
	cameraLeft.dist				= dPLeft;

	cameraRight.intrinsicParams	= iPRight;
	cameraRight.dist			= dPRight;

	rightCameraPose				= rCPose;
	cameraPoseOnRobot			= cPORobot;
}

CObservationStereoImagesFeatures::CObservationStereoImagesFeatures(
	const TCamera &cLeft, const TCamera &cRight,
	const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot )
{
	cameraLeft	= cLeft;
	cameraRight = cRight;

	rightCameraPose			= rCPose;
	cameraPoseOnRobot		= cPORobot;
}


CObservationStereoImagesFeatures::~CObservationStereoImagesFeatures( )
{}

void  CObservationStereoImagesFeatures::saveFeaturesToTextFile( const std::string &filename )
{
	CFileOutputStream	file( filename );

	vector<TStereoImageFeatures>::iterator it;
	for( it = theFeatures.begin(); it != theFeatures.end(); ++it )
		file << format("%u %.2f %.2f %.2f %.2f\n", it->ID, it->pixels.first.x, it->pixels.first.y, it->pixels.second.x, it->pixels.second.y );

	file.close();
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationStereoImagesFeatures::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0 ;
	else
	{
		// The data
		out << cameraLeft;
		out << cameraRight;
		out << rightCameraPose << cameraPoseOnRobot;
		out << (uint32_t)theFeatures.size();	// Write the number of items within the feature list
		for( unsigned int i = 0; i < theFeatures.size(); ++i )
		{
			out << theFeatures[i].pixels.first.x << theFeatures[i].pixels.first.y;
			out << theFeatures[i].pixels.second.x << theFeatures[i].pixels.second.y;
			out << (uint32_t)theFeatures[i].ID;
		}
		out << sensorLabel << timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationStereoImagesFeatures::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t nL, nR;
			in >> cameraLeft;
			in >> cameraRight;
			in >> rightCameraPose >> cameraPoseOnRobot;
			in >> nL;
			theFeatures.resize( nL );
			for( unsigned int i = 0; i < theFeatures.size(); ++i )
			{
				in >> theFeatures[i].pixels.first.x >> theFeatures[i].pixels.first.y;
				in >> theFeatures[i].pixels.second.x >> theFeatures[i].pixels.second.y;
				in >> nR;
				theFeatures[i].ID = (unsigned int)nR;
			}
			in >> sensorLabel >> timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationStereoImagesFeatures::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << cameraPoseOnRobot.getHomogeneousMatrixVal()
	<< cameraPoseOnRobot << endl;

	o << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:\n";
	o << rightCameraPose.getHomogeneousMatrixVal()
	<< rightCameraPose << endl;

	o << "Intrinsic parameters matrix for the LEFT camera:"<< endl;
	CMatrixDouble33 aux = cameraLeft.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the LEFT camera:"<< endl << "[ ";
	for( unsigned int i = 0; i < 5; ++i )
		o << cameraLeft.dist[i] << " ";
	o << "]" << endl;

	o << "Intrinsic parameters matrix for the RIGHT camera:"<< endl;
	aux = cameraRight.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the RIGHT camera:"<< endl << "[ ";
	for( unsigned int i = 0; i < 5; ++i )
		o << cameraRight.dist[i] << " ";
	o << "]"<< endl;

	o << endl << format(" Image size: %ux%u pixels\n", (unsigned int)cameraLeft.ncols, (unsigned int)cameraLeft.nrows );
	o << endl << format(" Number of features in images: %u\n", (unsigned int)theFeatures.size() );


}


