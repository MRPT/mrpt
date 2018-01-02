/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/math/geometry.h>
#include <mrpt/io/CFileOutputStream.h>

#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/system/os.h>
#include <mrpt/random.h>

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/COpenGLScene.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace std;
using mrpt::maps::internal::TSequenceLandmarks;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CLandmarksMap,landmarksMap", mrpt::maps::CLandmarksMap)

CLandmarksMap::TMapDefinition::TMapDefinition() {}
void CLandmarksMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation =
		sectionNamePrefix + string("_creationOpts");
	this->initialBeacons.clear();
	const unsigned int nBeacons = source.read_int(sSectCreation, "nBeacons", 0);
	for (unsigned int q = 1; q <= nBeacons; q++)
	{
		TPairIdBeacon newPair;
		newPair.second =
			source.read_int(sSectCreation, format("beacon_%03u_ID", q), 0);

		newPair.first.x =
			source.read_float(sSectCreation, format("beacon_%03u_x", q), 0);
		newPair.first.y =
			source.read_float(sSectCreation, format("beacon_%03u_y", q), 0);
		newPair.first.z =
			source.read_float(sSectCreation, format("beacon_%03u_z", q), 0);

		this->initialBeacons.push_back(newPair);
	}

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CLandmarksMap::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	out.printf(
		"number of initial beacons                = %u\n",
		(int)initialBeacons.size());

	out.printf("      ID         (X,Y,Z)\n");
	out.printf("--------------------------------------------------------\n");
	for (std::deque<TPairIdBeacon>::const_iterator p = initialBeacons.begin();
		 p != initialBeacons.end(); ++p)
		out.printf(
			"      %03u         (%8.03f,%8.03f,%8.03f)\n", p->second,
			p->first.x, p->first.y, p->first.z);

	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out.printf(
		"\n----------- [CLandmarksMap::TLikelihoodOptions] ------------ \n\n");

	out.printf(
		"rangeScan2D_decimation                  = %i\n",
		rangeScan2D_decimation);
	out.printf(
		"SIFTs_sigma_euclidean_dist              = %f\n",
		SIFTs_sigma_euclidean_dist);
	out.printf(
		"SIFTs_sigma_descriptor_dist             = %f\n",
		SIFTs_sigma_descriptor_dist);
	out.printf(
		"SIFTs_mahaDist_std                      = %f\n", SIFTs_mahaDist_std);
	out.printf(
		"SIFTs_decimation                        = %i\n", SIFTs_decimation);
	out.printf(
		"SIFTnullCorrespondenceDistance          = %f\n",
		SIFTnullCorrespondenceDistance);
	out.printf(
		"beaconRangesStd                         = %f\n", beaconRangesStd);
	out.printf(
		"beaconRangesUseObservationStd           = %c\n",
		beaconRangesUseObservationStd ? 'Y' : 'N');
	out.printf(
		"extRobotPoseStd                         = %f\n", extRobotPoseStd);

	out.printf(
		"GPSOrigin:LATITUDE                      = %f\n", GPSOrigin.latitude);
	out.printf(
		"GPSOrigin:LONGITUDE                     = %f\n", GPSOrigin.longitude);
	out.printf(
		"GPSOrigin:ALTITUDE                      = %f\n", GPSOrigin.altitude);
	out.printf("GPSOrigin:Rotation_Angle                = %f\n", GPSOrigin.ang);
	out.printf(
		"GPSOrigin:x_shift                       = %f\n", GPSOrigin.x_shift);
	out.printf(
		"GPSOrigin:y_shift                       = %f\n", GPSOrigin.y_shift);
	out.printf(
		"GPSOrigin:min_sat                       = %i\n", GPSOrigin.min_sat);

	out.printf("GPS_sigma                               = %f (m)\n", GPS_sigma);

	SIFT_feat_options.dumpToTextStream(out);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CLandmarksMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	rangeScan2D_decimation = iniFile.read_int(
		section.c_str(), "rangeScan2D_decimation", rangeScan2D_decimation);
	SIFTs_sigma_euclidean_dist = iniFile.read_double(
		section.c_str(), "SIFTs_sigma_euclidean_dist",
		SIFTs_sigma_euclidean_dist);
	SIFTs_sigma_descriptor_dist = iniFile.read_double(
		section.c_str(), "SIFTs_sigma_descriptor_dist",
		SIFTs_sigma_descriptor_dist);
	SIFTs_mahaDist_std = iniFile.read_float(
		section.c_str(), "SIFTs_mahaDist_std", SIFTs_mahaDist_std);
	SIFTs_decimation =
		iniFile.read_int(section.c_str(), "SIFTs_decimation", SIFTs_decimation);
	SIFTnullCorrespondenceDistance = iniFile.read_float(
		section.c_str(), "SIFTnullCorrespondenceDistance",
		SIFTnullCorrespondenceDistance);

	GPSOrigin.latitude = iniFile.read_double(
		section.c_str(), "GPSOriginLatitude", GPSOrigin.latitude);
	GPSOrigin.longitude = iniFile.read_double(
		section.c_str(), "GPSOriginLongitude", GPSOrigin.longitude);
	GPSOrigin.altitude = iniFile.read_double(
		section.c_str(), "GPSOriginAltitude", GPSOrigin.altitude);
	GPSOrigin.ang =
		iniFile.read_double(section.c_str(), "GPSOriginAngle", GPSOrigin.ang) *
		M_PI / 180;
	GPSOrigin.x_shift = iniFile.read_double(
		section.c_str(), "GPSOriginXshift", GPSOrigin.x_shift);
	GPSOrigin.y_shift = iniFile.read_double(
		section.c_str(), "GPSOriginYshift", GPSOrigin.y_shift);
	GPSOrigin.min_sat =
		iniFile.read_int(section.c_str(), "GPSOriginMinSat", GPSOrigin.min_sat);

	GPS_sigma = iniFile.read_float(section.c_str(), "GPSSigma", GPS_sigma);

	beaconRangesStd =
		iniFile.read_float(section.c_str(), "beaconRangesStd", beaconRangesStd);
	beaconRangesUseObservationStd = iniFile.read_bool(
		section.c_str(), "beaconRangesUseObservationStd",
		beaconRangesUseObservationStd);

	extRobotPoseStd =
		iniFile.read_float(section.c_str(), "extRobotPoseStd", extRobotPoseStd);

	SIFT_feat_options.loadFromConfigFile(iniFile, section);
}

/*---------------------------------------------------------------
					TFuseOptions
  ---------------------------------------------------------------*/
CLandmarksMap::TFuseOptions::TFuseOptions()
	: minTimesSeen(2), ellapsedTime(4.0f)
{
}

/*---------------------------------------------------------------
					 isEmpty
  ---------------------------------------------------------------*/
bool CLandmarksMap::isEmpty() const { return size() == 0; }
/*---------------------------------------------------------------
					 simulateBeaconReadings
  ---------------------------------------------------------------*/
void CLandmarksMap::simulateBeaconReadings(
	const CPose3D& in_robotPose, const CPoint3D& in_sensorLocationOnRobot,
	mrpt::obs::CObservationBeaconRanges& out_Observations) const
{
	TSequenceLandmarks::const_iterator it;
	mrpt::obs::CObservationBeaconRanges::TMeasurement newMeas;
	CPoint3D point3D, beacon3D;
	CPointPDFGaussian beaconPDF;

	// Compute the 3D position of the sensor:
	point3D = in_robotPose + in_sensorLocationOnRobot;

	// Clear output data:
	out_Observations.sensedData.clear();
	out_Observations.timestamp = mrpt::system::getCurrentTime();

	// For each BEACON landmark in the map:
	for (it = landmarks.begin(); it != landmarks.end(); it++)
	{
		if (it->getType() == featBeacon)
		{
			// Get the 3D position of the beacon (just the mean value):
			it->getPose(beaconPDF);
			beacon3D = beaconPDF.mean;

			float range = point3D.distanceTo(beacon3D);

			// Add noise:
			range += out_Observations.stdError *
					 getRandomGenerator().drawGaussian1D_normalized();
			range = max(0.0f, range);

			if (range >= out_Observations.minSensorDistance &&
				range <= out_Observations.maxSensorDistance)
			{
				// Fill out:
				newMeas.beaconID = it->ID;
				newMeas.sensorLocationOnRobot = in_sensorLocationOnRobot;
				newMeas.sensedDistance = range;

				// Insert:
				out_Observations.sensedData.push_back(newMeas);
			}
		}  // end if beacon
	}  // end for it
	// Done!
}

/*---------------------------------------------------------------
					 saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CLandmarksMap::saveMetricMapRepresentationToFile(
	const std::string& filNamePrefix) const
{
	MRPT_START

	// Matlab:
	std::string fil1(filNamePrefix + std::string("_3D.m"));
	saveToMATLABScript3D(fil1);

	// 3D Scene:
	opengl::COpenGLScene scene;
	mrpt::opengl::CSetOfObjects::Ptr obj3D =
		mrpt::make_aligned_shared<mrpt::opengl::CSetOfObjects>();
	getAs3DObject(obj3D);

	opengl::CGridPlaneXY::Ptr objGround =
		mrpt::make_aligned_shared<opengl::CGridPlaneXY>(
			-100, 100, -100, 100, 0, 1);

	scene.insert(obj3D);
	scene.insert(objGround);

	std::string fil2(filNamePrefix + std::string("_3D.3Dscene"));
	CFileOutputStream f(fil2.c_str());
	f << scene;

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
  ---------------------------------------------------------------*/
void CLandmarksMap::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	// TODO: Generate patchs in 3D, etc...

	// Save 3D ellipsoids
	CPointPDFGaussian pointGauss;
	for (TCustomSequenceLandmarks::const_iterator it = landmarks.begin();
		 it != landmarks.end(); ++it)
	{
		opengl::CEllipsoid::Ptr ellip =
			mrpt::make_aligned_shared<opengl::CEllipsoid>();

		it->getPose(pointGauss);

		ellip->setPose(pointGauss.mean);
		ellip->setCovMatrix(pointGauss.cov);
		ellip->enableDrawSolid3D(false);
		ellip->setQuantiles(3.0);
		ellip->set3DsegmentsCount(10);
		ellip->setColor(0, 0, 1);
		ellip->setName(
			mrpt::format("LM.ID=%u", static_cast<unsigned int>(it->ID)));
		ellip->enableShowName(true);

		outObj->insert(ellip);
	}
}
/**** FAMD ****/
mrpt::maps::CLandmark::TLandmarkID CLandmarksMap::getMapMaxID()
{
	return _mapMaxID;
}
/**** END FAMD ****/

const CLandmark* CLandmarksMap::TCustomSequenceLandmarks::getByID(
	CLandmark::TLandmarkID ID) const
{
	for (size_t indx = 0; indx < m_landmarks.size(); indx++)
	{
		if (m_landmarks[indx].ID == ID) return &m_landmarks[indx];
	}
	return nullptr;
}

// CLandmark* 	CLandmarksMap::TCustomSequenceLandmarks::getByID(
// CLandmark::TLandmarkID ID )
//{
//	for(size_t indx = 0; indx < m_landmarks.size(); indx++)
//	{
//		if( m_landmarks[indx].ID == ID )
//			return &m_landmarks[indx];
//	}
//	return nullptr;
//}

const CLandmark* CLandmarksMap::TCustomSequenceLandmarks::getByBeaconID(
	unsigned int ID) const
{
	for (size_t indx = 0; indx < m_landmarks.size(); indx++)
	{
		if (m_landmarks[indx].ID == ID) return &m_landmarks[indx];
	}
	return nullptr;
}

/*---------------------------------------------------------------
   Computes the ratio in [0,1] of correspondences between "this" and the
 "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the
 maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching
 with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen
 from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2
 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance
 between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
 ----------------------------------------------------------------*/
float CLandmarksMap::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap2,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_START

	// Compare to a similar map only:
	const CLandmarksMap* otherMap = nullptr;

	if (otherMap2->GetRuntimeClass() == CLASS_ID(CLandmarksMap))
		otherMap = static_cast<const CLandmarksMap*>(otherMap2);

	if (!otherMap) return 0;

	TCustomSequenceLandmarks::const_iterator itThis, itOther;
	std::deque<CPointPDFGaussian::Ptr> poses3DThis, poses3DOther;
	std::deque<CPointPDFGaussian::Ptr>::iterator itPoseThis, itPoseOther;
	CPointPDFGaussian tempPose;
	size_t nThis = landmarks.size();
	size_t nOther = otherMap->landmarks.size();
	size_t otherLandmarkWithCorrespondence = 0;

	// Checks:
	if (!nThis) return 0;
	if (!nOther) return 0;

	// The transformation:
	CMatrixDouble44 pose3DMatrix;
	otherMapPose.getHomogeneousMatrix(pose3DMatrix);
	float Tx = pose3DMatrix.get_unsafe(0, 3);
	float Ty = pose3DMatrix.get_unsafe(1, 3);
	float Tz = pose3DMatrix.get_unsafe(2, 3);

	// ---------------------------------------------------------------------------------------------------------------
	// Is there any "contact" between the spheres that contain all the points
	// from each map after translating them??
	// (Note that we can avoid computing the rotation of all the points if this
	// test fail, with a great speed up!)
	// ---------------------------------------------------------------------------------------------------------------
	if (sqrt(square(Tx) + square(Ty) + square(Tz)) >=
		(landmarks.getLargestDistanceFromOrigin() +
		 otherMap->landmarks.getLargestDistanceFromOrigin() + 1.0f))
		return 0;  // There is no contact!

	// Prepare:
	poses3DOther.resize(nOther);
	for (size_t i = 0; i < nOther; i++)
		poses3DOther[i] = mrpt::make_aligned_shared<CPointPDFGaussian>();

	poses3DThis.resize(nThis);
	for (size_t i = 0; i < nThis; i++)
		poses3DThis[i] = mrpt::make_aligned_shared<CPointPDFGaussian>();

	// Save 3D poses of the other map with transformed coordinates:
	for (itOther = otherMap->landmarks.begin(),
		itPoseOther = poses3DOther.begin();
		 itOther != otherMap->landmarks.end(); itOther++, itPoseOther++)
	{
		itOther->getPose(**itPoseOther);
		(*itPoseOther)->changeCoordinatesReference(otherMapPose);
	}

	// And the 3D poses of "this" map:
	for (itThis = landmarks.begin(), itPoseThis = poses3DThis.begin();
		 itThis != landmarks.end(); itThis++, itPoseThis++)
	{
		itThis->getPose(**itPoseThis);
	}

	// Check whether each "other"'s LM has a correspondence or not:
	for (itOther = otherMap->landmarks.begin(),
		itPoseOther = poses3DOther.begin();
		 itOther != otherMap->landmarks.end(); itOther++, itPoseOther++)
	{
		for (itThis = landmarks.begin(), itPoseThis = poses3DThis.begin();
			 itThis != landmarks.end(); itThis++, itPoseThis++)
		{
			CMatrixDouble COV =
				CMatrixDouble((*itPoseThis)->cov + (*itPoseOther)->cov);
			TPoint3D D(
				(*itPoseThis)->mean.x() - (*itPoseOther)->mean.x(),
				(*itPoseThis)->mean.y() - (*itPoseOther)->mean.y(),
				(*itPoseThis)->mean.z() - (*itPoseOther)->mean.z());
			CMatrixDouble d(1, 3);
			d(0, 0) = D.x;
			d(0, 1) = D.y;
			d(0, 2) = D.z;

			float distMaha = sqrt(
				d.multiply_HCHt_scalar(
					COV.inv()));  //(d*COV.inv()*(~d))(0,0) );

			if (distMaha < params.maxMahaDistForCorr)
			{
				// Now test the SIFT descriptors:
				if (!itThis->features.empty() && !itOther->features.empty() &&
					itThis->features[0]->descriptors.SIFT.size() ==
						itOther->features[0]->descriptors.SIFT.size())
				{
					unsigned long descrDist = 0;
					std::vector<unsigned char>::const_iterator it1, it2;
					for (it1 = itThis->features[0]->descriptors.SIFT.begin(),
						it2 = itOther->features[0]->descriptors.SIFT.begin();
						 it1 != itThis->features[0]->descriptors.SIFT.end();
						 it1++, it2++)
						descrDist += square(*it1 - *it2);

					float descrDist_f =
						sqrt(static_cast<float>(descrDist)) /
						itThis->features[0]->descriptors.SIFT.size();

					if (descrDist_f < 1.5f)
					{
						otherLandmarkWithCorrespondence++;
						break;  // Go to the next "other" LM
					}
				}
			}
		}  // for each in "this"

	}  // for each in "other"

	return static_cast<float>(otherLandmarkWithCorrespondence) / nOther;

	MRPT_END
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void CLandmarksMap::auxParticleFilterCleanUp()
{
	// std::cout << "mEDD:" << std::endl;
	// std::cout << "-----------------------" << std::endl;
	// std::map<std::pair<mrpt::maps::CLandmark::TLandmarkID,
	// mrpt::maps::CLandmark::TLandmarkID>, unsigned long>::iterator itmEDD;
	// for(itmEDD = CLandmarksMap::_mEDD.begin(); itmEDD !=
	// CLandmarksMap::_mEDD.end(); itmEDD++)
	//	std::cout << "(" << itmEDD->first.first << "," << itmEDD->first.second
	//<< ")"  << ": " << itmEDD->second << std::endl;

	CLandmarksMap::_mEDD.clear();
	CLandmarksMap::_maxIDUpdated = false;
	// TODO: Paco...
}

/*---------------------------------------------------------------
					simulateRangeBearingReadings
 ---------------------------------------------------------------*/
void CLandmarksMap::simulateRangeBearingReadings(
	const CPose3D& in_robotPose, const CPose3D& in_sensorLocationOnRobot,
	CObservationBearingRange& out_Observations, bool sensorDetectsIDs,
	const float in_stdRange, const float in_stdYaw, const float in_stdPitch,
	std::vector<size_t>* out_real_associations, const double spurious_count_mean,
	const double spurious_count_std) const
{
	TSequenceLandmarks::const_iterator it;
	size_t idx;
	mrpt::obs::CObservationBearingRange::TMeasurement newMeas;
	CPoint3D beacon3D;
	CPointPDFGaussian beaconPDF;

	if (out_real_associations) out_real_associations->clear();

	// Compute the 3D position of the sensor:
	const CPose3D point3D = in_robotPose + CPose3D(in_sensorLocationOnRobot);

	// Clear output data:
	out_Observations.validCovariances = false;
	out_Observations.sensor_std_range = in_stdRange;
	out_Observations.sensor_std_yaw = in_stdYaw;
	out_Observations.sensor_std_pitch = in_stdPitch;

	out_Observations.sensedData.clear();
	out_Observations.timestamp = mrpt::system::getCurrentTime();
	out_Observations.sensorLocationOnRobot = in_sensorLocationOnRobot;

	// For each BEACON landmark in the map:
	// ------------------------------------------
	for (idx = 0, it = landmarks.begin(); it != landmarks.end(); ++it, ++idx)
	{
		// Get the 3D position of the beacon (just the mean value):
		it->getPose(beaconPDF);
		beacon3D = CPoint3D(beaconPDF.mean);

		// Compute yaw,pitch,range:
		double range, yaw, pitch;
		point3D.sphericalCoordinates(beacon3D, range, yaw, pitch);

		// Add noises:
		range += in_stdRange * getRandomGenerator().drawGaussian1D_normalized();
		yaw += in_stdYaw * getRandomGenerator().drawGaussian1D_normalized();
		pitch += in_stdPitch * getRandomGenerator().drawGaussian1D_normalized();

		yaw = math::wrapToPi(yaw);
		range = max(0.0, range);

		if (range >= out_Observations.minSensorDistance &&
			range <= out_Observations.maxSensorDistance &&
			fabs(yaw) <= 0.5f * out_Observations.fieldOfView_yaw &&
			fabs(pitch) <= 0.5f * out_Observations.fieldOfView_pitch)
		{
			// Fill out:
			if (sensorDetectsIDs)
				newMeas.landmarkID = it->ID;
			else
				newMeas.landmarkID = INVALID_LANDMARK_ID;
			newMeas.range = range;
			newMeas.yaw = yaw;
			newMeas.pitch = pitch;

			// Insert:
			out_Observations.sensedData.push_back(newMeas);

			if (out_real_associations)
				out_real_associations->push_back(idx);  // Real indices.
		}
	}  // end for it

	const double fSpurious =
		getRandomGenerator().drawGaussian1D(spurious_count_mean, spurious_count_std);
	size_t nSpurious = 0;
	if (spurious_count_std != 0 || spurious_count_mean != 0)
		nSpurious = static_cast<size_t>(
			mrpt::round_long(std::max(0.0, fSpurious)));

	// For each spurios reading to generate:
	// ------------------------------------------
	for (size_t i = 0; i < nSpurious; i++)
	{
		// Make up yaw,pitch,range out from thin air:
		// (the conditionals on yaw & pitch are to generate 2D observations if
		// we are in 2D, which we learn from a null std.dev.)
		const double range = getRandomGenerator().drawUniform(
			out_Observations.minSensorDistance,
			out_Observations.maxSensorDistance);
		const double yaw = (out_Observations.sensor_std_yaw == 0)
							   ? 0
							   : getRandomGenerator().drawUniform(
									 -0.5f * out_Observations.fieldOfView_yaw,
									 0.5f * out_Observations.fieldOfView_yaw);
		const double pitch =
			(out_Observations.sensor_std_pitch == 0)
				? 0
				: getRandomGenerator().drawUniform(
					  -0.5f * out_Observations.fieldOfView_pitch,
					  0.5f * out_Observations.fieldOfView_pitch);

		// Fill out:
		newMeas.landmarkID =
			INVALID_LANDMARK_ID;  // Always invalid ID since it's spurious
		newMeas.range = range;
		newMeas.yaw = yaw;
		newMeas.pitch = pitch;

		// Insert:
		out_Observations.sensedData.push_back(newMeas);

		if (out_real_associations)
			out_real_associations->push_back(
				std::string::npos);  // Real index: spurious
	}  // end for it

	// Done!
}
