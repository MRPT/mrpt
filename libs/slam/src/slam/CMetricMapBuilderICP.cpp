/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers
//
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/img/CEnhancedMetaFile.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/system/CTicTac.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::system;

CMetricMapBuilderICP::CMetricMapBuilderICP()
	: ICP_options(m_min_verbosity_level)
{
	this->setLoggerName("CMetricMapBuilderICP");
	this->initialize(CSimpleMap());
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CMetricMapBuilderICP::~CMetricMapBuilderICP()
{
	// Ensure, we have exit all critical zones:
	enterCriticalSection();
	leaveCriticalSection();

	// Save current map to current file:
	setCurrentMapFile("");
}

/*---------------------------------------------------------------
							Options
  ---------------------------------------------------------------*/
CMetricMapBuilderICP::TConfigParams::TConfigParams(
	mrpt::system::VerbosityLevel& parent_verbosity_level)
	: matchAgainstTheGrid(false),
	  insertionLinDistance(1.0),
	  insertionAngDistance(30.0_deg),
	  localizationLinDistance(0.20),
	  localizationAngDistance(30.0_deg),
	  minICPgoodnessToAccept(0.40),
	  verbosity_level(parent_verbosity_level),
	  mapInitializers()
{
}

CMetricMapBuilderICP::TConfigParams&
	CMetricMapBuilderICP::TConfigParams::operator=(
		const CMetricMapBuilderICP::TConfigParams& other)
{
	matchAgainstTheGrid = other.matchAgainstTheGrid;
	insertionLinDistance = other.insertionLinDistance;
	insertionAngDistance = other.insertionAngDistance;
	localizationLinDistance = other.localizationLinDistance;
	localizationAngDistance = other.localizationAngDistance;
	minICPgoodnessToAccept = other.minICPgoodnessToAccept;
	//	We can't copy a reference type
	//	verbosity_level         = other.verbosity_level;
	mapInitializers = other.mapInitializers;
	return *this;
}

void CMetricMapBuilderICP::TConfigParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_LOAD_CONFIG_VAR(matchAgainstTheGrid, bool, source, section)
	MRPT_LOAD_CONFIG_VAR(insertionLinDistance, double, source, section)
	MRPT_LOAD_CONFIG_VAR_DEGREES(insertionAngDistance, source, section)
	MRPT_LOAD_CONFIG_VAR(localizationLinDistance, double, source, section)
	MRPT_LOAD_CONFIG_VAR_DEGREES(localizationAngDistance, source, section)
	verbosity_level = source.read_enum<mrpt::system::VerbosityLevel>(
		section, "verbosity_level", verbosity_level);

	MRPT_LOAD_CONFIG_VAR(minICPgoodnessToAccept, double, source, section)

	mapInitializers.loadFromConfigFile(source, section);
}

void CMetricMapBuilderICP::TConfigParams::dumpToTextStream(
	std::ostream& out) const
{
	out << "\n----------- [CMetricMapBuilderICP::TConfigParams] ------------ "
		   "\n\n";

	out << mrpt::format(
		"insertionLinDistance                    = %f m\n",
		insertionLinDistance);
	out << mrpt::format(
		"insertionAngDistance                    = %f deg\n",
		RAD2DEG(insertionAngDistance));
	out << mrpt::format(
		"localizationLinDistance                 = %f m\n",
		localizationLinDistance);
	out << mrpt::format(
		"localizationAngDistance                 = %f deg\n",
		RAD2DEG(localizationAngDistance));
	out << mrpt::format(
		"verbosity_level                         = %s\n",
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::value2name(
			verbosity_level)
			.c_str());

	out << "  Now showing 'mapsInitializers':\n";
	mapInitializers.dumpToTextStream(out);
}

/*---------------------------------------------------------------
						processObservation
 This is the new entry point of the algorithm (the old one
  was processActionObservation, which now is a wrapper to
  this method).
  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::processObservation(const CObservation::Ptr& obs)
{
	auto lck = mrpt::lockHelper(critZoneChangingMap);

	MRPT_START

	if (metricMap.countMapsByClass<mrpt::maps::CPointsMap>() == 0 &&
		metricMap.countMapsByClass<mrpt::maps::COccupancyGridMap2D>() == 0)
		throw std::runtime_error(
			"Neither grid maps nor points map: Have you called initialize() "
			"after setting ICP_options.mapInitializers?");

	ASSERT_(obs);

	// Is it an odometry observation??
	if (IS_CLASS(*obs, CObservationOdometry))
	{
		MRPT_LOG_DEBUG("processObservation(): obs is CObservationOdometry");
		m_there_has_been_an_odometry = true;

		const CObservationOdometry::Ptr odo =
			std::dynamic_pointer_cast<CObservationOdometry>(obs);
		ASSERT_(odo->timestamp != INVALID_TIMESTAMP);

		CPose2D pose_before;
		bool pose_before_valid = m_lastPoseEst.getLatestRobotPose(pose_before);

		// Move our estimation:
		m_lastPoseEst.processUpdateNewOdometry(
			odo->odometry.asTPose(), odo->timestamp, odo->hasVelocities,
			odo->velocityLocal);

		if (pose_before_valid)
		{
			// Accumulate movement:
			CPose2D pose_after;
			if (m_lastPoseEst.getLatestRobotPose(pose_after))
				this->accumulateRobotDisplacementCounters(pose_after);
			MRPT_LOG_DEBUG_STREAM(
				"processObservation(): obs is CObservationOdometry, new "
				"post_after="
				<< pose_after);
		}
	}  // end it's odometry
	else
	{
		// Current robot pose given the timestamp of the observation (this can
		// include a small extrapolation
		//  using the latest known robot velocities):
		TPose2D initialEstimatedRobotPose(0, 0, 0);
		{
			mrpt::math::TTwist2D robotVelLocal, robotVelGlobal;
			if (obs->timestamp != INVALID_TIMESTAMP)
			{
				MRPT_LOG_DEBUG(
					"processObservation(): extrapolating pose from latest pose "
					"and new observation timestamp...");
				if (!m_lastPoseEst.getCurrentEstimate(
						initialEstimatedRobotPose, robotVelLocal,
						robotVelGlobal, obs->timestamp))
				{  // couldn't had a good extrapolation estimate... we'll have
					// to live with the latest pose:
					m_lastPoseEst.getLatestRobotPose(initialEstimatedRobotPose);
					MRPT_LOG_THROTTLE_WARN(
						10.0 /*seconds*/,
						"processObservation(): new pose extrapolation failed, "
						"using last pose as is.");
				}
			}
			else
			{
				MRPT_LOG_WARN(
					"processObservation(): invalid observation timestamp.");
				m_lastPoseEst.getLatestRobotPose(initialEstimatedRobotPose);
			}
		}

		// To know the total path length:
		CPose2D previousKnownRobotPose;
		m_lastPoseEst.getLatestRobotPose(previousKnownRobotPose);

		// Increment (this may only include the effects of extrapolation with
		// velocity...):
		this->accumulateRobotDisplacementCounters(
			previousKnownRobotPose);  // initialEstimatedRobotPose-previousKnownRobotPose);

		// We'll skip ICP-based localization for this observation only if:
		//  - We had some odometry since the last pose correction
		//  (m_there_has_been_an_odometry=true).
		//  - AND, the traversed distance is small enough:
		const bool we_skip_ICP_pose_correction = m_there_has_been_an_odometry &&
			m_distSinceLastICP.lin < std::min(
										 ICP_options.localizationLinDistance,
										 ICP_options.insertionLinDistance) &&
			m_distSinceLastICP.ang < std::min(
										 ICP_options.localizationAngDistance,
										 ICP_options.insertionAngDistance);

		MRPT_LOG_DEBUG_STREAM(
			"processObservation(): skipping ICP pose correction due to small "
			"odometric displacement? : "
			<< (we_skip_ICP_pose_correction ? "YES" : "NO"));

		CICP::TReturnInfo icpReturn;
		bool can_do_icp = false;

		// Select the map to match with ....
		CMetricMap* matchWith = nullptr;
		if (auto pGrid = metricMap.mapByClass<COccupancyGridMap2D>();
			ICP_options.matchAgainstTheGrid && pGrid)
		{
			matchWith = static_cast<CMetricMap*>(pGrid.get());
			MRPT_LOG_DEBUG("processObservation(): matching against gridmap.");
		}
		else
		{
			auto pPts = metricMap.mapByClass<CPointsMap>();
			ASSERTMSG_(pPts, "No points map in multi-metric map.");

			matchWith = static_cast<CMetricMap*>(pPts.get());
			MRPT_LOG_DEBUG("processObservation(): matching against point map.");
		}
		ASSERT_(matchWith != nullptr);

		if (!we_skip_ICP_pose_correction)
		{
			m_there_has_been_an_odometry = false;

			// --------------------------------------------------------------------------------------
			// Any other observation:
			//  1) If the observation generates points in a point map,
			//  do ICP 2) In any case, insert the observation if the
			//  minimum distance has been satisfaced.
			// --------------------------------------------------------------------------------------
			CSimplePointsMap sensedPoints;
			sensedPoints.insertionOptions.minDistBetweenLaserPoints = 0.02f;
			sensedPoints.insertionOptions.also_interpolate = false;

			// Create points representation of the observation:
			// Insert only those planar range scans in the altitude of
			// the grid map:
			if (ICP_options.matchAgainstTheGrid &&
				0 != metricMap.countMapsByClass<COccupancyGridMap2D>() &&
				metricMap.mapByClass<COccupancyGridMap2D>(0)
					->insertionOptions.useMapAltitude)
			{
				// Use grid altitude:
				if (IS_CLASS(*obs, CObservation2DRangeScan))
				{
					CObservation2DRangeScan::Ptr obsLaser =
						std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
					if (std::abs(
							metricMap.mapByClass<COccupancyGridMap2D>(0)
								->insertionOptions.mapAltitude -
							obsLaser->sensorPose.z()) < 0.01)
						can_do_icp = sensedPoints.insertObservationPtr(obs);
				}
			}
			else
			{
				// Do not use grid altitude:
				can_do_icp = sensedPoints.insertObservationPtr(obs);
			}

			if (IS_DERIVED(*matchWith, CPointsMap) &&
				static_cast<CPointsMap*>(matchWith)->empty())
				can_do_icp = false;	 // The reference map is empty!

			if (can_do_icp)
			{
				// We DO HAVE points with this observation:
				// Execute ICP over the current points map and the
				// sensed points:
				// ----------------------------------------------------------------------
				CICP ICP;
				ICP.options = ICP_params;

				// a first gross estimation of map 2 relative to map 1.
				const auto firstGuess =
					mrpt::poses::CPose2D(initialEstimatedRobotPose);

				CPosePDF::Ptr pestPose = ICP.Align(
					matchWith,	// Map 1
					&sensedPoints,	// Map 2
					firstGuess, icpReturn);

				if (icpReturn.goodness > ICP_options.minICPgoodnessToAccept)
				{
					// save estimation:
					CPosePDFGaussian pEst2D;
					pEst2D.copyFrom(*pestPose);

					m_lastPoseEst.processUpdateNewPoseLocalization(
						pEst2D.mean.asTPose(), obs->timestamp);
					m_lastPoseEst_cov = pEst2D.cov;

					m_distSinceLastICP.updatePose(pEst2D.mean);

					// Debug output to console:
					MRPT_LOG_INFO_STREAM(
						"processObservation: previousPose="
						<< previousKnownRobotPose << "-> currentPose="
						<< pEst2D.getMeanVal() << std::endl);
					MRPT_LOG_INFO(format(
						"[CMetricMapBuilderICP]   Fit:%.1f%% Itr:%i In "
						"%.02fms \n",
						icpReturn.goodness * 100, icpReturn.nIterations,
						1000 * icpReturn.executionTime));
				}
				else
				{
					MRPT_LOG_WARN_STREAM(
						"Ignoring ICP of low quality: " << icpReturn.goodness *
							100 << std::endl);
				}

				// Compute the transversed length:
				CPose2D currentKnownRobotPose;
				m_lastPoseEst.getLatestRobotPose(currentKnownRobotPose);

				this->accumulateRobotDisplacementCounters(
					currentKnownRobotPose);	 // currentKnownRobotPose -
				// previousKnownRobotPose);

			}  // end we can do ICP.
			else
			{
				MRPT_LOG_WARN_STREAM(
					"Cannot do ICP: empty pointmap or not suitable "
					"gridmap...\n");
			}

		}  // else, we do ICP pose correction

		// ----------------------------------------------------------
		//				CRITERION TO DECIDE MAP UPDATE:
		//   A distance large-enough from the last update for each
		//   sensor, AND
		//    either: (i) this was a good match or (ii) this is the
		//    first time for this sensor.
		// ----------------------------------------------------------
		const bool firstTimeForThisSensor =
			m_distSinceLastInsertion.find(obs->sensorLabel) ==
			m_distSinceLastInsertion.end();
		bool update = firstTimeForThisSensor ||
			((!can_do_icp ||
			  icpReturn.goodness > ICP_options.minICPgoodnessToAccept) &&
			 (m_distSinceLastInsertion[obs->sensorLabel].lin >=
				  ICP_options.insertionLinDistance ||
			  m_distSinceLastInsertion[obs->sensorLabel].ang >=
				  ICP_options.insertionAngDistance));

		// Used any "options.alwaysInsertByClass" ??
		if (options.alwaysInsertByClass.contains(obs->GetRuntimeClass()))
			update = true;

		// We need to always insert ALL the observations at the
		// beginning until the first one
		//  that actually insert some points into the map used as a
		//  reference, since otherwise we'll not be able to do ICP
		//  against an empty map!!
		if (matchWith && matchWith->isEmpty()) update = true;

		MRPT_LOG_DEBUG_STREAM(
			"update map: " << (update ? "YES" : "NO")
						   << " options.enableMapUpdating: "
						   << (options.enableMapUpdating ? "YES" : "NO"));

		if (options.enableMapUpdating && update)
		{
			CTicTac tictac;

			tictac.Tic();

			// Insert the observation:
			CPose2D currentKnownRobotPose;
			m_lastPoseEst.getLatestRobotPose(currentKnownRobotPose);

			// Create new entry:
			m_distSinceLastInsertion[obs->sensorLabel].last_update =
				currentKnownRobotPose.asTPose();

			// Reset distance counters:
			resetRobotDisplacementCounters(currentKnownRobotPose);
			// m_distSinceLastInsertion[obs->sensorLabel].updatePose(currentKnownRobotPose);

			MRPT_LOG_INFO(mrpt::format(
				"Updating map from pose %s\n",
				currentKnownRobotPose.asString().c_str()));

			CPose3D estimatedPose3D(currentKnownRobotPose);
			const bool anymap_update =
				metricMap.insertObservationPtr(obs, estimatedPose3D);
			if (!anymap_update)
				MRPT_LOG_WARN_STREAM(
					"**No map was updated** after inserting an "
					"observation of "
					"type `"
					<< obs->GetRuntimeClass()->className << "`");

			// Add to the vector of "poses"-"SFs" pairs:
			CPosePDFGaussian posePDF(currentKnownRobotPose);
			CPose3DPDF::Ptr pose3D =
				CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(posePDF));

			CSensoryFrame::Ptr sf = std::make_shared<CSensoryFrame>();
			sf->insert(obs);

			SF_Poses_seq.insert(pose3D, sf);

			MRPT_LOG_INFO_STREAM(
				"Map updated OK. Done in "
				<< mrpt::system::formatTimeInterval(tictac.Tac()) << std::endl);
		}

	}  // end other observation

	// Robot path history:
	{
		TPose2D p;
		if (m_lastPoseEst.getLatestRobotPose(p)) m_estRobotPath.push_back(p);
	}

	MRPT_END

}  // end processObservation

/*---------------------------------------------------------------

						processActionObservation

  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::processActionObservation(
	CActionCollection& action, CSensoryFrame& in_SF)
{
	// 1) process action:
	CActionRobotMovement2D::Ptr movEstimation =
		action.getBestMovementEstimation();
	if (movEstimation)
	{
		m_auxAccumOdometry.composeFrom(
			m_auxAccumOdometry, movEstimation->poseChange->getMeanVal());

		CObservationOdometry::Ptr obs =
			std::make_shared<CObservationOdometry>();
		obs->timestamp = movEstimation->timestamp;
		obs->odometry = m_auxAccumOdometry;
		this->processObservation(obs);
	}

	// 2) Process observations one by one:
	for (auto& i : in_SF)
		this->processObservation(i);
}

/*---------------------------------------------------------------
						setCurrentMapFile
  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::setCurrentMapFile(const char* mapFile)
{
	// Save current map to current file:
	if (currentMapFile.size()) saveCurrentMapToFile(currentMapFile.c_str());

	// Sets new current map file:
	currentMapFile = mapFile;

	// Load map from file or create an empty one:
	if (currentMapFile.size()) loadCurrentMapFromFile(mapFile);
}

/*---------------------------------------------------------------
						getCurrentPoseEstimation
  ---------------------------------------------------------------*/
CPose3DPDF::Ptr CMetricMapBuilderICP::getCurrentPoseEstimation() const
{
	CPosePDFGaussian pdf2D;
	m_lastPoseEst.getLatestRobotPose(pdf2D.mean);
	pdf2D.cov = m_lastPoseEst_cov;

	CPose3DPDFGaussian::Ptr pdf3D = std::make_shared<CPose3DPDFGaussian>();
	pdf3D->copyFrom(pdf2D);
	return pdf3D;
}

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::initialize(
	const CSimpleMap& initialMap, const CPosePDF* x0)
{
	MRPT_START

	// Reset vars:
	m_estRobotPath.clear();
	m_auxAccumOdometry = CPose2D(0, 0, 0);

	m_distSinceLastICP.lin = m_distSinceLastICP.ang = 0;
	m_distSinceLastInsertion.clear();

	m_there_has_been_an_odometry = false;

	// Init path & map:
	auto lck = mrpt::lockHelper(critZoneChangingMap);

	// Create metric maps:
	metricMap.setListOfMaps(ICP_options.mapInitializers);

	// copy map:
	SF_Poses_seq = initialMap;

	// Load estimated pose from given PDF:
	m_lastPoseEst.reset();

	if (x0)
		m_lastPoseEst.processUpdateNewPoseLocalization(
			x0->getMeanVal().asTPose(), mrpt::Clock::now());

	for (size_t i = 0; i < SF_Poses_seq.size(); i++)
	{
		// Get the SF and its pose:
		const auto& kf = SF_Poses_seq.get(i);

		const CPose3D estimatedPose3D = kf.pose->getMeanVal();

		// Insert observations into the map:
		kf.sf->insertObservationsInto(metricMap, estimatedPose3D);
	}

	MRPT_END
}

void CMetricMapBuilderICP::getCurrentMapPoints(
	std::vector<float>& x, std::vector<float>& y)
{
	auto lck = mrpt::lockHelper(critZoneChangingMap);

	auto pPts = metricMap.mapByClass<CPointsMap>(0);

	ASSERT_(pPts);
	pPts->getAllPoints(x, y);
}

/*---------------------------------------------------------------
				getCurrentlyBuiltMap
  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::getCurrentlyBuiltMap(CSimpleMap& out_map) const
{
	out_map = SF_Poses_seq;
}

const CMultiMetricMap& CMetricMapBuilderICP::getCurrentlyBuiltMetricMap() const
{
	return metricMap;
}

/*---------------------------------------------------------------
			getCurrentlyBuiltMapSize
  ---------------------------------------------------------------*/
unsigned int CMetricMapBuilderICP::getCurrentlyBuiltMapSize()
{
	return SF_Poses_seq.size();
}

/*---------------------------------------------------------------
				saveCurrentEstimationToImage
  ---------------------------------------------------------------*/
void CMetricMapBuilderICP::saveCurrentEstimationToImage(
	const std::string& file, bool formatEMF_BMP)
{
	MRPT_START

	CImage img;
	const size_t nPoses = m_estRobotPath.size();

	if (!formatEMF_BMP) THROW_EXCEPTION("Not implemented yet for BMP!");

	// grid map as bitmap:
	auto pGrid = metricMap.mapByClass<COccupancyGridMap2D>();
	if (pGrid) pGrid->getAsImage(img);

	// Draw paths (using vectorial plots!) over the EMF file:
	// -------------------------------------------------
	CEnhancedMetaFile EMF(file, 1000);

	EMF.drawImage(0, 0, img);

	unsigned int imgHeight = img.getHeight();

	// Path hypothesis:
	// ----------------------------------
	int x1, x2, y1, y2;

	// First point: (0,0)
	x2 = pGrid->x2idx(0.0f);
	y2 = pGrid->y2idx(0.0f);

	// Draw path in the bitmap:
	for (size_t j = 0; j < nPoses; j++)
	{
		// For next segment
		x1 = x2;
		y1 = y2;

		// Coordinates -> pixels
		x2 = pGrid->x2idx(m_estRobotPath[j].x);
		y2 = pGrid->y2idx(m_estRobotPath[j].y);

		// Draw line:
		EMF.line(
			x1, imgHeight - 1 - y1, x2, imgHeight - 1 - y2, TColor::black());
	}

	MRPT_END
}

void CMetricMapBuilderICP::accumulateRobotDisplacementCounters(
	const CPose2D& new_pose)
{
	m_distSinceLastICP.updateDistances(new_pose);
	for (auto& m : m_distSinceLastInsertion)
		m.second.updateDistances(new_pose);
}

void CMetricMapBuilderICP::resetRobotDisplacementCounters(
	const CPose2D& new_pose)
{
	m_distSinceLastICP.updatePose(new_pose);
	for (auto& m : m_distSinceLastInsertion)
		m.second.updatePose(new_pose);
}

void CMetricMapBuilderICP::TDist::updateDistances(const mrpt::poses::CPose2D& p)
{
	const auto Ap = p - mrpt::poses::CPose2D(this->last_update);
	lin = Ap.norm();
	ang = std::abs(Ap.phi());
}

void CMetricMapBuilderICP::TDist::updatePose(const mrpt::poses::CPose2D& p)
{
	this->last_update = p.asTPose();
	lin = 0;
	ang = 0;
}
