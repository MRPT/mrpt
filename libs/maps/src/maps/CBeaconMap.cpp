/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/random.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/math/geometry.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/math/data_utils.h>  // averageLogLikelihood()
#include <mrpt/system/os.h>
#include <mrpt/serialization/CArchive.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::system;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CBeaconMap,beaconMap", mrpt::maps::CBeaconMap)

CBeaconMap::TMapDefinition::TMapDefinition() {}
void CBeaconMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	// const std::string sSectCreation =
	// sectionNamePrefix+string("_creationOpts");
	// MRPT_LOAD_CONFIG_VAR(resolution, float,   source,sSectCreation);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CBeaconMap::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	// LOADABLEOPTS_DUMP_VAR(resolution     , float);

	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out.printf(
		"\n----------- [CBeaconMap::TInsertionOptions] ------------ \n\n");

	out.printf(
		"insertAsMonteCarlo                      = %c\n",
		insertAsMonteCarlo ? 'Y' : 'N');
	out.printf(
		"minElevation_deg                        = %.03f\n", minElevation_deg);
	out.printf(
		"maxElevation_deg                        = %.03f\n", maxElevation_deg);
	out.printf(
		"MC_numSamplesPerMeter                   = %d\n",
		MC_numSamplesPerMeter);
	out.printf(
		"MC_maxStdToGauss                        = %.03f\n", MC_maxStdToGauss);
	out.printf(
		"MC_thresholdNegligible                  = %.03f\n",
		MC_thresholdNegligible);
	out.printf(
		"MC_performResampling                    = %c\n",
		MC_performResampling ? 'Y' : 'N');
	out.printf(
		"MC_afterResamplingNoise                 = %.03f\n",
		MC_afterResamplingNoise);
	out.printf(
		"SOG_thresholdNegligible                 = %.03f\n",
		SOG_thresholdNegligible);
	out.printf(
		"SOG_maxDistBetweenGaussians             = %.03f\n",
		SOG_maxDistBetweenGaussians);
	out.printf(
		"SOG_separationConstant                  = %.03f\n",
		SOG_separationConstant);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CBeaconMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
	MRPT_LOAD_CONFIG_VAR(insertAsMonteCarlo, bool, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(maxElevation_deg, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(minElevation_deg, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_numSamplesPerMeter, int, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_maxStdToGauss, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		MC_thresholdNegligible, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(MC_performResampling, bool, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		MC_afterResamplingNoise, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_thresholdNegligible, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_maxDistBetweenGaussians, float, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		SOG_separationConstant, float, iniFile, section.c_str());
}

/*---------------------------------------------------------------
					 isEmpty
  ---------------------------------------------------------------*/
bool CBeaconMap::isEmpty() const { return size() == 0; }
/*---------------------------------------------------------------
					 simulateBeaconReadings
  ---------------------------------------------------------------*/
void CBeaconMap::simulateBeaconReadings(
	const CPose3D& in_robotPose, const CPoint3D& in_sensorLocationOnRobot,
	CObservationBeaconRanges& out_Observations) const
{
	TSequenceBeacons::const_iterator it;
	mrpt::obs::CObservationBeaconRanges::TMeasurement newMeas;
	CPoint3D point3D, beacon3D;
	CPointPDFGaussian beaconPDF;

	// Compute the 3D position of the sensor:
	point3D = in_robotPose + in_sensorLocationOnRobot;

	// Clear output data:
	out_Observations.sensedData.clear();

	// For each BEACON landmark in the map:
	for (it = m_beacons.begin(); it != m_beacons.end(); ++it)
	{
		it->getMean(beacon3D);

		float range = point3D.distanceTo(beacon3D);

		if (range < out_Observations.maxSensorDistance &&
			range > out_Observations.minSensorDistance)
		{
			// Add noise:
			range +=
				getRandomGenerator().drawGaussian1D(0, out_Observations.stdError);

			// Fill out:
			newMeas.beaconID = it->m_ID;
			newMeas.sensorLocationOnRobot = in_sensorLocationOnRobot;
			newMeas.sensedDistance = range;

			// Insert:
			out_Observations.sensedData.push_back(newMeas);
		}
	}  // end for it
	// Done!
}

/*---------------------------------------------------------------
					 saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CBeaconMap::saveMetricMapRepresentationToFile(
	const string& filNamePrefix) const
{
	MRPT_START

	// Matlab:
	string fil1(filNamePrefix + string("_3D.m"));
	saveToMATLABScript3D(fil1);

	// 3D Scene:
	opengl::COpenGLScene scene;
	opengl::CSetOfObjects::Ptr obj3D =
		mrpt::make_aligned_shared<opengl::CSetOfObjects>();

	getAs3DObject(obj3D);
	opengl::CGridPlaneXY::Ptr objGround =
		mrpt::make_aligned_shared<opengl::CGridPlaneXY>(
			-100, 100, -100, 100, 0, 1);

	scene.insert(obj3D);
	scene.insert(objGround);

	string fil2(filNamePrefix + string("_3D.3Dscene"));
	CFileOutputStream f(fil2.c_str());
	f << scene;

	// Textual representation:
	string fil3(filNamePrefix + string("_covs.txt"));
	saveToTextFile(fil3);

	// Total number of particles / modes:
	string fil4(filNamePrefix + string("_population.txt"));
	{
		FILE* f = os::fopen(fil4.c_str(), "wt");
		if (f)
		{
			size_t nParts = 0, nGaussians = 0;

			for (TSequenceBeacons::const_iterator it = m_beacons.begin();
				 it != m_beacons.end(); ++it)
			{
				switch (it->m_typePDF)
				{
					case CBeacon::pdfMonteCarlo:
						nParts += it->m_locationMC.size();
						break;
					case CBeacon::pdfSOG:
						nGaussians += it->m_locationSOG.size();
						break;
					case CBeacon::pdfGauss:
						nGaussians++;
						break;
				};
			}

			fprintf(
				f, "%u %u", static_cast<unsigned>(nParts),
				static_cast<unsigned>(nGaussians));
			os::fclose(f);
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
  ---------------------------------------------------------------*/
void CBeaconMap::getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START

	if (!genericMapParams.enableSaveAs3DObject) return;

	// ------------------------------------------------
	//  Add the XYZ corner for the current area:
	// ------------------------------------------------
	outObj->insert(opengl::stock_objects::CornerXYZ());

	// Save 3D ellipsoids or whatever representation:
	for (const_iterator it = m_beacons.begin(); it != m_beacons.end(); ++it)
		it->getAs3DObject(outObj);

	MRPT_END
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
float CBeaconMap::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap2,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_START

	// Compare to a similar map only:
	const CBeaconMap* otherMap = nullptr;

	if (otherMap2->GetRuntimeClass() == CLASS_ID(CBeaconMap))
		otherMap = static_cast<const CBeaconMap*>(otherMap2);

	if (!otherMap) return 0;

	TMatchingPairList matchList;
	vector<bool> otherCorrespondences;
	float out_corrsRatio;

	CBeaconMap modMap;

	modMap.changeCoordinatesReference(otherMapPose, otherMap);

	computeMatchingWith3DLandmarks(
		&modMap, matchList, out_corrsRatio, otherCorrespondences);

	return out_corrsRatio;

	MRPT_END
}

/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
const CBeacon* CBeaconMap::getBeaconByID(CBeacon::TBeaconID id) const
{
	for (const_iterator it = m_beacons.begin(); it != m_beacons.end(); ++it)
		if (it->m_ID == id) return &(*it);
	return nullptr;
}

/*---------------------------------------------------------------
					getBeaconByID
 ---------------------------------------------------------------*/
CBeacon* CBeaconMap::getBeaconByID(CBeacon::TBeaconID id)
{
	for (iterator it = m_beacons.begin(); it != m_beacons.end(); ++it)
		if (it->m_ID == id) return &(*it);
	return nullptr;
}

/*---------------------------------------------------------------
					saveToTextFile
- VX VY VZ: Variances of each dimension (C11, C22, C33)
- DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
- C12, C13, C23: Cross covariances
 ---------------------------------------------------------------*/
void CBeaconMap::saveToTextFile(const string& fil) const
{
	MRPT_START
	FILE* f = os::fopen(fil.c_str(), "wt");
	ASSERT_(f != nullptr);

	CPoint3D p;
	CMatrixDouble33 C;

	for (const_iterator it = m_beacons.begin(); it != m_beacons.end(); ++it)
	{
		it->getCovarianceAndMean(C, p);

		float D3 = C.det();
		float D2 = C(0, 0) * C(1, 1) - square(C(0, 1));
		os::fprintf(
			f, "%i %f %f %f %e %e %e %e %e %e %e %e\n",
			static_cast<int>(it->m_ID), p.x(), p.y(), p.z(), C(0, 0), C(1, 1),
			C(2, 2), D2, D3, C(0, 1), C(1, 2), C(1, 2));
	}

	os::fclose(f);
	MRPT_END
}
