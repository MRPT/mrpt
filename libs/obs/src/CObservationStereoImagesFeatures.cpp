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
#include <mrpt/slam/CObservationStereoImagesFeatures.h>

#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationStereoImagesFeatures, CObservation,mrpt::slam)

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
void  CObservationStereoImagesFeatures::writeToStream(CStream &out, int *version) const
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
void  CObservationStereoImagesFeatures::readFromStream(CStream &in, int version)
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

