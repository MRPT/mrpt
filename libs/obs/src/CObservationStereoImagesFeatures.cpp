/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
#include <mrpt/obs/CObservationStereoImagesFeatures.h>
#include <mrpt/serialization/CArchive.h>
#include <fstream>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(
	CObservationStereoImagesFeatures, CObservation, mrpt::obs)

CObservationStereoImagesFeatures::CObservationStereoImagesFeatures(
	const TCamera& cLeft, const TCamera& cRight, const CPose3DQuat& rCPose,
	const CPose3DQuat& cPORobot)
{
	cameraLeft = cLeft;
	cameraRight = cRight;

	rightCameraPose = rCPose;
	cameraPoseOnRobot = cPORobot;
}

void CObservationStereoImagesFeatures::saveFeaturesToTextFile(
	const std::string& filename)
{
	std::ofstream file(filename);
	ASSERT_(file.is_open());

	vector<TStereoImageFeatures>::iterator it;
	for (it = theFeatures.begin(); it != theFeatures.end(); ++it)
		file << format(
			"%u %.2f %.2f %.2f %.2f\n", it->ID, it->pixels.first.x,
			it->pixels.first.y, it->pixels.second.x, it->pixels.second.y);
}

uint8_t CObservationStereoImagesFeatures::serializeGetVersion() const
{
	return 0;
}
void CObservationStereoImagesFeatures::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	// The data
	out << cameraLeft;
	out << cameraRight;
	out << rightCameraPose << cameraPoseOnRobot;
	out << (uint32_t)theFeatures.size();  // Write the number of items
	// within the feature list
	for (const auto& theFeature : theFeatures)
	{
		out << theFeature.pixels.first.x << theFeature.pixels.first.y;
		out << theFeature.pixels.second.x << theFeature.pixels.second.y;
		out << (uint32_t)theFeature.ID;
	}
	out << sensorLabel << timestamp;
}

void CObservationStereoImagesFeatures::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t nL, nR;
			in >> cameraLeft;
			in >> cameraRight;
			in >> rightCameraPose >> cameraPoseOnRobot;
			in >> nL;
			theFeatures.resize(nL);
			for (auto& theFeature : theFeatures)
			{
				in >> theFeature.pixels.first.x >> theFeature.pixels.first.y;
				in >> theFeature.pixels.second.x >> theFeature.pixels.second.y;
				in >> nR;
				theFeature.ID = (unsigned int)nR;
			}
			in >> sensorLabel >> timestamp;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationStereoImagesFeatures::getDescriptionAsText(
	std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot "
		 "base:\n";
	o << cameraPoseOnRobot.getHomogeneousMatrixVal<CMatrixDouble44>()
	  << cameraPoseOnRobot << endl;

	o << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT "
		 "camera reference system:\n";
	o << rightCameraPose.getHomogeneousMatrixVal<CMatrixDouble44>()
	  << rightCameraPose << endl;

	o << "Intrinsic parameters matrix for the LEFT camera:" << endl;
	CMatrixDouble33 aux = cameraLeft.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the LEFT camera:" << endl << "[ ";
	for (unsigned int i = 0; i < 5; ++i) o << cameraLeft.dist[i] << " ";
	o << "]" << endl;

	o << "Intrinsic parameters matrix for the RIGHT camera:" << endl;
	aux = cameraRight.intrinsicParams;
	o << aux.inMatlabFormat() << endl << aux << endl;

	o << "Distortion parameters vector for the RIGHT camera:" << endl << "[ ";
	for (unsigned int i = 0; i < 5; ++i) o << cameraRight.dist[i] << " ";
	o << "]" << endl;

	o << endl
	  << format(
			 " Image size: %ux%u pixels\n", (unsigned int)cameraLeft.ncols,
			 (unsigned int)cameraLeft.nrows);
	o << endl
	  << format(
			 " Number of features in images: %u\n",
			 (unsigned int)theFeatures.size());
}
