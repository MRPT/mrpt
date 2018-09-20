/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CRobot2DPoseEstimator.h>

namespace mrpt::slam
{
/** A class for very simple 2D SLAM based on ICP. This is a non-probabilistic
 *pose tracking algorithm.
 *   Map are stored as in files as binary dumps of "mrpt::maps::CSimpleMap"
 *objects. The methods are
 *	 thread-safe.
 * \ingroup metric_slam_grp
 */
class CMetricMapBuilderICP : public mrpt::slam::CMetricMapBuilder
{
   public:
	/** Default constructor - Upon construction, you can set the parameters in
	 * ICP_options, then call "initialize".
	 */
	CMetricMapBuilderICP();

	/** Destructor:
	 */
	~CMetricMapBuilderICP() override;

	/** Algorithm configuration params
	 */
	struct TConfigParams : public mrpt::config::CLoadableOptions
	{
		/** Initializer */
		TConfigParams(mrpt::system::VerbosityLevel& parent_verbosity_level);
		TConfigParams& operator=(const TConfigParams& other);  // Copy
		// assignment

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs
		/** (default:false) Match against the occupancy grid or the points map?
		 * The former is quicker but less precise. */
		bool matchAgainstTheGrid;

		/** Minimum robot linear (m) displacement for a new observation to be
		 * inserted in the map. */
		double insertionLinDistance;
		/** Minimum robot angular (rad, deg when loaded from the .ini)
		 * displacement for a new observation to be inserted in the map. */
		double insertionAngDistance;
		/** Minimum robot linear (m) displacement for a new observation to be
		 * used to do ICP-based localization (otherwise, dead-reckon with
		 * odometry). */
		double localizationLinDistance;
		/** Minimum robot angular (rad, deg when loaded from the .ini)
		 * displacement for a new observation to be used to do ICP-based
		 * localization (otherwise, dead-reckon with odometry). */
		double localizationAngDistance;

		/** Minimum ICP goodness (0,1) to accept the resulting corrected
		 * position (default: 0.40) */
		double minICPgoodnessToAccept;

		mrpt::system::VerbosityLevel& verbosity_level;

		/** What maps to create (at least one points map and/or a grid map are
		 * needed).
		 *  For the expected format in the .ini file when loaded with
		 * loadFromConfigFile(), see documentation of
		 * TSetOfMetricMapInitializers.
		 */
		mrpt::maps::TSetOfMetricMapInitializers mapInitializers;
	};

	/** Options for the ICP-SLAM application \sa ICP_params */
	TConfigParams ICP_options;
	/** Options for the ICP algorithm itself \sa ICP_options */
	CICP::TConfigParams ICP_params;

	/** Initialize the method, starting with a known location PDF "x0"(if
	 * supplied, set to nullptr to left unmodified) and a given fixed, past map.
	 *  This method MUST be called if using the default constructor, after
	 * loading the configuration into ICP_options. In particular,
	 * TConfigParams::mapInitializers
	 */
	void initialize(
		const mrpt::maps::CSimpleMap& initialMap = mrpt::maps::CSimpleMap(),
		const mrpt::poses::CPosePDF* x0 = nullptr) override;

	/** Returns a copy of the current best pose estimation as a pose PDF.
	 */
	mrpt::poses::CPose3DPDF::Ptr getCurrentPoseEstimation() const override;

	/** Sets the "current map file", thus that map will be loaded if it exists
	 * or a new one will be created if it does not, and the updated map will be
	 * save to that file when destroying the object.
	 */
	void setCurrentMapFile(const char* mapFile);

	/** Appends a new action and observations to update this map: See the
	 *description of the class at the top of this page to see a more complete
	 *description.
	 *  \param action The estimation of the incremental pose change in the robot
	 *pose.
	 *	\param in_SF The set of observations that robot senses at the new pose.
	 * See params in CMetricMapBuilder::options and
	 *CMetricMapBuilderICP::ICP_options
	 * \sa processObservation
	 */
	void processActionObservation(
		mrpt::obs::CActionCollection& action,
		mrpt::obs::CSensoryFrame& in_SF) override;

	/**  The main method of this class: Process one odometry or sensor
	 observation.
		The new entry point of the algorithm (the old one  was
	 processActionObservation, which now is a wrapper to
	  this method).
	 * See params in CMetricMapBuilder::options and
	 CMetricMapBuilderICP::ICP_options
	  */
	void processObservation(const mrpt::obs::CObservation::Ptr& obs);

	/** Fills "out_map" with the set of "poses"-"sensory-frames", thus the so
	 * far built map */
	void getCurrentlyBuiltMap(mrpt::maps::CSimpleMap& out_map) const override;

	/** Returns the 2D points of current local map */
	void getCurrentMapPoints(std::vector<float>& x, std::vector<float>& y);

	const mrpt::maps::CMultiMetricMap* getCurrentlyBuiltMetricMap()
		const override;

	/** Returns just how many sensory-frames are stored in the currently build
	 * map */
	unsigned int getCurrentlyBuiltMapSize() override;

	/** A useful method for debugging: the current map (and/or poses) estimation
	 * is dumped to an image file.
	 * \param file The output file name
	 * \param formatEMF_BMP Output format = true:EMF, false:BMP
	 */
	void saveCurrentEstimationToImage(
		const std::string& file, bool formatEMF_BMP = true) override;

   private:
	/** The set of observations that leads to current map: */
	mrpt::maps::CSimpleMap SF_Poses_seq;

	/** The metric map representation as a points map: */
	mrpt::maps::CMultiMetricMap metricMap;

	/** Current map file. */
	std::string currentMapFile;

	/** The pose estimation by the alignment algorithm (ICP). */
	/** Last pose estimation (Mean) */
	mrpt::poses::CRobot2DPoseEstimator m_lastPoseEst;
	/** Last pose estimation (covariance) */
	mrpt::math::CMatrixDouble33 m_lastPoseEst_cov;

	/** The estimated robot path:
	 */
	std::deque<mrpt::math::TPose2D> m_estRobotPath;
	mrpt::poses::CPose2D m_auxAccumOdometry;

	/** Traveled distances from last map update / ICP-based localization. */
	struct TDist
	{
		TDist() = default;
		double lin{0};  // meters
		double ang{0};  // degrees
		mrpt::poses::CPose2D last_update;

		void updateDistances(const mrpt::poses::CPose2D& p);
		void updatePose(const mrpt::poses::CPose2D& p);
	};
	TDist m_distSinceLastICP;
	/** Indexed by sensor label. */
	mrpt::aligned_std_map<std::string, TDist> m_distSinceLastInsertion;
	bool m_there_has_been_an_odometry;

	void accumulateRobotDisplacementCounters(
		const mrpt::poses::CPose2D& new_pose);
	void resetRobotDisplacementCounters(const mrpt::poses::CPose2D& new_pose);
};

}  // namespace mrpt::slam
