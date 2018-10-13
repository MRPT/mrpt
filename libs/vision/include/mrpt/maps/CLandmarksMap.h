/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
namespace maps
{
namespace internal
{
using TSequenceLandmarks = std::vector<CLandmark>;
}

/** A class for storing a map of 3D probabilistic landmarks.
 * <br>
 *  Currently these types of landmarks are defined: (see mrpt::maps::CLandmark)
 *		- For "visual landmarks" from images: features with associated
 descriptors.
 *		- For laser scanners: each of the range measuremnts, as "occupancy"
 landmarks.
 *		- For grid maps: "Panoramic descriptor" feature points.
 *		- For range-only localization and SLAM: Beacons. It is also supported
 the simulation of expected beacon-to-sensor readings, observation
 likelihood,...
 * <br>
 * <b>How to load landmarks from observations:</b><br>
 *  When invoking CLandmarksMap::insertObservation(), the values in
 CLandmarksMap::insertionOptions will
 *     determinate the kind of landmarks that will be extracted and fused into
 the map. Supported feature
 *     extraction processes are listed next:
 *
  <table>
  <tr> <td><b>Observation class:</b></td> <td><b>Generated Landmarks:</b></td>
 <td><b>Comments:</b></td> </tr>
  <tr> <td>CObservationImage</td> <td>vlSIFT</td> <td>1) A SIFT feature is
 created for each SIFT detected in the image,
		   <br>2) Correspondences between SIFTs features and existing ones are
 finded by computeMatchingWith3DLandmarks,
		   <br>3) The corresponding feaures are fused, and the new ones added,
 with an initial uncertainty according to insertionOptions</td> </tr>
  <tr> <td>CObservationStereoImages</td> <td>vlSIFT</td> <td> Each image is
 separately procesed by the method for CObservationImage observations </td>
 </tr>
  <tr> <td>CObservationStereoImages</td> <td>vlColor</td> <td> TODO... </td>
 </tr>
  <tr> <td>CObservation2DRangeScan</td> <td>glOccupancy</td> <td> A landmark is
 added for each range in the scan, with its appropiate covariance matrix derived
 from the jacobians matrixes. </td> </tr>
  </table>
 *
 * \sa CMetricMap
 * \ingroup mrpt_vision_grp
 */
class CLandmarksMap : public mrpt::maps::CMetricMap
{
	DEFINE_SERIALIZABLE(CLandmarksMap)

   private:
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

   public:
	/** Computes the (logarithmic) likelihood that a given observation was taken
	 *from a given pose in the world being modeled with this map.
	 *
	 *  In the current implementation, this method behaves in a different way
	 *according to the nature of
	 *   the observation's class:
	 *		- "mrpt::obs::CObservation2DRangeScan": This calls
	 *"computeLikelihood_RSLC_2007".
	 *		- "mrpt::obs::CObservationStereoImages": This calls
	 *"computeLikelihood_SIFT_LandmarkMap".
	 * <table>
	 * <tr> <td><b>Observation class:</b></td> <td><b>Generated
	 *Landmarks:</b></td> <td><b>Comments:</b></td> </tr>
	 * <tr> <td>CObservationImage</td> <td>vlSIFT</td> <td>1) A SIFT feature is
	 *created for each SIFT detected in the image,
	 *          <br>2) Correspondences between SIFTs features and existing ones
	 *are found by computeMatchingWith3DLandmarks,
	 *		   <br>3) The corresponding feaures are fused, and the new ones
	 * added,
	 *with an initial uncertainty according to insertionOptions</td> </tr>
	 * <tr> <td>CObservationStereoImages</td> <td>vlSIFT</td> <td> Each image is
	 *separately procesed by the method for CObservationImage observations </td>
	 *</tr>
	 * <tr> <td>CObservationStereoImages</td> <td>vlColor</td> <td> TODO...
	 *</td> </tr>
	 * <tr> <td>CObservation2DRangeScan</td> <td>glOccupancy</td> <td> A
	 *landmark is added for each range in the scan, with its appropiate
	 *covariance matrix derived from the jacobians matrixes. </td> </tr>
	 * </table>
	 *
	 * \param takenFrom The robot's pose the observation is supposed to be taken
	 *from.
	 * \param obs The observation.
	 * \return This method returns a likelihood value > 0.
	 *
	 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
	 */
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

	/** The color of landmark ellipsoids in CLandmarksMap::getAs3DObject */
	static mrpt::img::TColorf COLOR_LANDMARKS_IN_3DSCENES;

	using landmark_type = mrpt::maps::CLandmark;

	/** The list of landmarks: the wrapper class is just for maintaining the
	 * KD-Tree representation
	 */
	struct TCustomSequenceLandmarks
	{
	   private:
		/** The actual list */
		internal::TSequenceLandmarks m_landmarks;

		/** A grid-map with the set of landmarks falling into each cell.
		 *  \todo Use the KD-tree instead?
		 */
		mrpt::containers::CDynamicGrid<std::vector<int32_t>> m_grid;

		/** Auxiliary variables used in "getLargestDistanceFromOrigin"
		 * \sa getLargestDistanceFromOrigin
		 */
		mutable float m_largestDistanceFromOrigin{};

		/** Auxiliary variables used in "getLargestDistanceFromOrigin"
		 * \sa getLargestDistanceFromOrigin
		 */
		mutable bool m_largestDistanceFromOriginIsUpdated{false};

	   public:
		/** Default constructor
		 */
		TCustomSequenceLandmarks();

		using iterator = internal::TSequenceLandmarks::iterator;
		inline iterator begin() { return m_landmarks.begin(); };
		inline iterator end() { return m_landmarks.end(); };
		void clear();
		inline size_t size() const { return m_landmarks.size(); };
		using const_iterator = internal::TSequenceLandmarks::const_iterator;
		inline const_iterator begin() const { return m_landmarks.begin(); };
		inline const_iterator end() const { return m_landmarks.end(); };
		/** The object is copied, thus the original copy passed as a parameter
		 * can be released.
		 */
		void push_back(const CLandmark& lm);
		CLandmark* get(unsigned int indx);
		const CLandmark* get(unsigned int indx) const;
		void isToBeModified(unsigned int indx);
		void hasBeenModified(unsigned int indx);
		void hasBeenModifiedAll();
		void erase(unsigned int indx);

		mrpt::containers::CDynamicGrid<std::vector<int32_t>>* getGrid()
		{
			return &m_grid;
		}
		/** Returns the landmark with a given landmrk ID, or nullptr if not
		 * found
		 */
		const CLandmark* getByID(CLandmark::TLandmarkID ID) const;

		/** Returns the landmark with a given beacon ID, or nullptr if not found
		 */
		const CLandmark* getByBeaconID(unsigned int ID) const;

		/** This method returns the largest distance from the origin to any of
		 * the points, such as a sphere centered at the origin with this radius
		 * cover ALL the points in the map (the results are buffered, such as,
		 * if the map is not modified, the second call will be much faster than
		 * the first one).
		 */
		float getLargestDistanceFromOrigin() const;

	} landmarks;

	/** Constructor
	 */
	CLandmarksMap();

	/** Virtual destructor.
	 */
	~CLandmarksMap() override;

	/**** FAMD ***/
	/** Map of the Euclidean Distance between the descriptors of two SIFT-based
	 * landmarks
	 */
	static std::map<
		std::pair<
			mrpt::maps::CLandmark::TLandmarkID,
			mrpt::maps::CLandmark::TLandmarkID>,
		double>
		_mEDD;
	static mrpt::maps::CLandmark::TLandmarkID _mapMaxID;
	static bool _maxIDUpdated;

	mrpt::maps::CLandmark::TLandmarkID getMapMaxID();
	/**** END FAMD *****/

	// See docs in base class
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;

	/** With this struct options are provided to the observation insertion
	 * process.
	 */
	struct TInsertionOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		/** Initilization of default parameters
		 */
		TInsertionOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** If set to true (default), the insertion of a CObservationImage in
		 * the map will insert SIFT 3D features.
		 */
		bool insert_SIFTs_from_monocular_images{true};

		/** If set to true (default), the insertion of a
		 * CObservationStereoImages in the map will insert SIFT 3D features.
		 */
		bool insert_SIFTs_from_stereo_images{true};

		/** If set to true (default), inserting a CObservation2DRangeScan in the
		 * map will generate landmarks for each range.
		 */
		bool insert_Landmarks_from_range_scans{true};

		/** [For SIFT landmarks only] The ratio between the best and second best
		 * descriptor distances to set as correspondence (Default=0.4)
		 */
		float SiftCorrRatioThreshold{0.4f};

		/** [For SIFT landmarks only] The minimum likelihood value of a match to
		 * set as correspondence (Default=0.5)
		 */
		float SiftLikelihoodThreshold{0.5f};

		/****************************************** FAMD
		 * ******************************************/
		/** [For SIFT landmarks only] The minimum Euclidean Descriptor Distance
		 * value of a match to set as correspondence (Default=200)
		 */
		float SiftEDDThreshold{200.0f};

		/** [For SIFT landmarks only] Method to compute 3D matching (Default = 0
		 * (Our method))
		 * 0: Our method -> Euclidean Distance between Descriptors and 3D
		 * position
		 * 1: Sim, Elinas, Griffin, Little -> Euclidean Distance between
		 * Descriptors
		 */
		unsigned int SIFTMatching3DMethod{0};

		/** [For SIFT landmarks only] Method to compute the likelihood (Default
		 * = 0 (Our method))
		 * 0: Our method -> Euclidean Distance between Descriptors and 3D
		 * position
		 * 1: Sim, Elinas, Griffin, Little -> 3D position
		 */
		unsigned int SIFTLikelihoodMethod{0};

		/****************************************** END FAMD
		 * ******************************************/

		/** [For SIFT landmarks only] The distance (in meters) of the mean value
		 * of landmarks, for the initial position PDF (Default = 3m)
		 */
		float SIFTsLoadDistanceOfTheMean{3.0f};

		/** [For SIFT landmarks only] The width (in meters, standard deviation)
		 * of the ellipsoid in the axis perpendicular to the main directiom
		 * (Default = 0.05f)
		 */
		float SIFTsLoadEllipsoidWidth{0.05f};

		/** [For SIFT landmarks only] The standard deviation (in pixels) for the
		 * SIFTs detector (This is used for the Jacobbian to project stereo
		 * images to 3D)
		 */
		float SIFTs_stdXY{2.0f}, SIFTs_stdDisparity{1.0f};

		/** Number of points to extract in the image
		 */
		int SIFTs_numberOfKLTKeypoints{60};

		/** Maximum depth of 3D landmarks when loading a landmarks map from a
		 * stereo image observation.
		 */
		float SIFTs_stereo_maxDepth{15.0f};

		/** Maximum distance (in pixels) from a point to a certain epipolar line
		 * to be considered a potential match.
		 */
		float SIFTs_epipolar_TH{1.5f};

		/** Indicates if the images (as well as the SIFT detected features)
		 * should be shown in a window.
		 */
		bool PLOT_IMAGES{false};

		/** Parameters of the SIFT feature detector/descriptors while inserting
		 * images in the landmark map.
		 *  \note There exists another \a SIFT_feat_options field in the \a
		 * likelihoodOptions member.
		 *  \note All parameters of this field can be loaded from a config
		 * file. See mrpt::vision::CFeatureExtraction::TOptions for the names of
		 * the expected fields.
		 */
		mrpt::vision::CFeatureExtraction::TOptions SIFT_feat_options;

	} insertionOptions;

	/** With this struct options are provided to the likelihood computations.
	 */
	struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		TLikelihoodOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** @name Parameters for: 2D LIDAR scans
		 * @{ */
		/** The number of rays from a 2D range scan will be decimated by this
		 * factor (default = 1, no decimation) */
		unsigned int rangeScan2D_decimation{20};
		/** @} */

		/** @name Parameters for: images
		 * @{ */
		double SIFTs_sigma_euclidean_dist{0.30f};
		double SIFTs_sigma_descriptor_dist{100.0f};
		float SIFTs_mahaDist_std{4.0f};
		float SIFTnullCorrespondenceDistance{4.0f};
		/** Considers 1 out of "SIFTs_decimation" visual landmarks in the
		 * observation during the likelihood computation. */
		int SIFTs_decimation{1};
		/** Parameters of the SIFT feature detector/descriptors while inserting
		 * images in the landmark map.
		 *  \note There exists another \a SIFT_feat_options field in the \a
		 * insertionOptions member.
		 *  \note All parameters of this field can be loaded from a config
		 * file. See mrpt::vision::CFeatureExtraction::TOptions for the names of
		 * the expected fields. */
		mrpt::vision::CFeatureExtraction::TOptions SIFT_feat_options;
		/** @} */

		/** @name Parameters for: Range-only observation
		 * @{ */
		/** The standard deviation used for Beacon ranges likelihood
		 * (default=0.08) [meters] \sa beaconRangesUseObservationStd */
		float beaconRangesStd{0.08f};
		/** (Default: false) If true, `beaconRangesStd` is ignored and each
		 * individual `CObservationBeaconRanges::stdError` field is used
		 * instead. */
		bool beaconRangesUseObservationStd{false};
		/** @} */

		/** @name Parameters for: External robot poses observation
		 * @{ */
		/** The standard deviation used for external robot poses likelihood
		 * (default=0.05) [meters] */
		float extRobotPoseStd{0.05f};
		/** @} */

		/** @name Parameters for: GPS readings
		 * @{ */

		/** This struct store de GPS longitude, latitude (in degrees ) and
		 * altitude (in meters) for the first GPS observation
		 * compose with de sensor position on the robot */
		struct TGPSOrigin
		{
		   public:
			TGPSOrigin();
			/** degrees */
			double longitude{-4.47763833333333};
			/** degrees */
			double latitude{36.71559000000000};
			/** meters */
			double altitude{42.3};
			/** These 3 params allow rotating and shifting GPS coordinates with
			 * other 2D maps (e.g. gridmaps).
			 * - ang : Map rotation [deg]
			 * - x_shift, y_shift: (x,y) offset [m] */
			double ang{0}, x_shift{0}, y_shift{0};
			/** Minimum number of sats to take into account the data */
			unsigned int min_sat{4};
		} GPSOrigin;

		/** A constant "sigma" for GPS localization data (in meters) */
		float GPS_sigma{1.0f};
		/** @} */

	} likelihoodOptions;

	/** This struct stores extra results from invoking insertObservation
	 */
	struct TInsertionResults
	{
		/** The number of SIFT detected in left and right images respectively
		 */

		unsigned int nSiftL, nSiftR;

	} insertionResults;

	/** With this struct options are provided to the fusion process.
	 */
	struct TFuseOptions
	{
		/** Initialization
		 */
		TFuseOptions();

		/** Required number of times of a landmark to be seen not to be removed,
		 * in "ellapsedTime" seconds.
		 */
		unsigned int minTimesSeen{2};

		/** See "minTimesSeen"
		 */
		float ellapsedTime{4.0f};

	} fuseOptions;

	/** Save to a text file.
	 *  In line "i" there are the (x,y,z) mean values of the i'th landmark +
	 * type of landmark + # times seen + timestamp + RGB/descriptor + ID
	 *
	 *   Returns false if any error occured, true elsewere.
	 */
	bool saveToTextFile(std::string file);

	/** Save to a MATLAB script which displays 2D error ellipses for the map
	 *(top-view, projection on the XY plane).
	 *	\param file		The name of the file to save the script to.
	 *  \param style	The MATLAB-like string for the style of the lines (see
	 *'help plot' in MATLAB for posibilities)
	 *  \param stdCount The ellipsoids will be drawn from the center to
	 *"stdCount" times the "standard deviations". (default is 2std = 95%
	 *confidence intervals)
	 *
	 *  \return Returns false if any error occured, true elsewere.
	 */
	bool saveToMATLABScript2D(
		std::string file, const char* style = "b", float stdCount = 2.0f);

	/** Save to a MATLAB script which displays 3D error ellipses for the map.
	 *	\param file		The name of the file to save the script to.
	 *  \param style	The MATLAB-like string for the style of the lines (see
	 *'help plot' in MATLAB for posibilities)
	 *  \param stdCount The ellipsoids will be drawn from the center to a given
	 *confidence interval in [0,1], e.g. 2 sigmas=0.95 (default is 2std = 0.95
	 *confidence intervals)
	 *
	 *  \return Returns false if any error occured, true elsewere.
	 */
	bool saveToMATLABScript3D(
		std::string file, const char* style = "b",
		float confInterval = 0.95f) const;

	/** Returns the stored landmarks count.
	 */
	size_t size() const;

	/** Computes the (logarithmic) likelihood function for a sensed observation
	  "o" according to "this" map.
	  *   This is the implementation of the algorithm reported in the paper:
			<em>J.L. Blanco, J. Gonzalez, and J.A. Fernandez-Madrigal, "A
	  Consensus-based Approach for Estimating the Observation Likelihood of
	  Accurate Range Sensors", in IEEE International Conference on Robotics and
	  Automation (ICRA), Rome (Italy), Apr 10-14, 2007</em>
	  */
	double computeLikelihood_RSLC_2007(
		const CLandmarksMap* s, const mrpt::poses::CPose2D& sensorPose);

	/** Loads into this landmarks map the SIFT features extracted from an image
	 * observation (Previous contents of map will be erased)
	 *  The robot is assumed to be at the origin of the map.
	 *  Some options may be applicable from "insertionOptions"
	 * (insertionOptions.SIFTsLoadDistanceOfTheMean)
	 *
	 *  \param feat_options Optionally, you can pass here parameters for
	 * changing the default SIFT detector settings.
	 */
	void loadSiftFeaturesFromImageObservation(
		const mrpt::obs::CObservationImage& obs,
		const mrpt::vision::CFeatureExtraction::TOptions& feat_options =
			mrpt::vision::CFeatureExtraction::TOptions(mrpt::vision::featSIFT));

	/** Loads into this landmarks map the SIFT features extracted from an
	 * observation consisting of a pair of stereo-image (Previous contents of
	 * map will be erased)
	 *  The robot is assumed to be at the origin of the map.
	 *  Some options may be applicable from "insertionOptions"
	 *
	 *  \param feat_options Optionally, you can pass here parameters for
	 * changing the default SIFT detector settings.
	 */
	void loadSiftFeaturesFromStereoImageObservation(
		const mrpt::obs::CObservationStereoImages& obs,
		mrpt::maps::CLandmark::TLandmarkID fID,
		const mrpt::vision::CFeatureExtraction::TOptions& feat_options =
			mrpt::vision::CFeatureExtraction::TOptions(mrpt::vision::featSIFT));

	/** Loads into this landmarks-map a set of occupancy features according to a
	 * 2D range scan (Previous contents of map will be erased)
	 *  \param obs	The observation
	 *  \param robotPose	The robot pose in the map (Default = the origin)
	 *  Some options may be applicable from "insertionOptions"
	 */
	void loadOccupancyFeaturesFrom2DRangeScan(
		const mrpt::obs::CObservation2DRangeScan& obs,
		const mrpt::poses::CPose3D* robotPose = nullptr,
		unsigned int downSampleFactor = 1);

	// See docs in base class
	void computeMatchingWith2D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose2D& otherMapPose,
		float maxDistForCorrespondence, float maxAngularDistForCorrespondence,
		const mrpt::poses::CPose2D& angularDistPivotPoint,
		mrpt::tfest::TMatchingPairList& correspondences,
		float& correspondencesRatio, float* sumSqrDist = nullptr,
		bool onlyKeepTheClosest = false, bool onlyUniqueRobust = false) const;

	/** Perform a search for correspondences between "this" and another
	 * lansmarks map:
	 *  In this class, the matching is established solely based on the
	 * landmarks' IDs.
	 * \param otherMap [IN] The other map.
	 * \param correspondences [OUT] The matched pairs between maps.
	 * \param correspondencesRatio [OUT] This is NumberOfMatchings /
	 * NumberOfLandmarksInTheAnotherMap
	 * \param otherCorrespondences [OUT] Will be returned with a vector
	 * containing "true" for the indexes of the other map's landmarks with a
	 * correspondence.
	 */
	void computeMatchingWith3DLandmarks(
		const mrpt::maps::CLandmarksMap* otherMap,
		mrpt::tfest::TMatchingPairList& correspondences,
		float& correspondencesRatio,
		std::vector<bool>& otherCorrespondences) const;

	/** Changes the reference system of the map to a given 3D pose.
	 */
	void changeCoordinatesReference(const mrpt::poses::CPose3D& newOrg);

	/** Changes the reference system of the map "otherMap" and save the result
	 * in "this" map.
	 */
	void changeCoordinatesReference(
		const mrpt::poses::CPose3D& newOrg,
		const mrpt::maps::CLandmarksMap* otherMap);

	/** Fuses the contents of another map with this one, updating "this" object
	 * with the result.
	 *  This process involves fusing corresponding landmarks, then adding the
	 * new ones.
	 *  \param other The other landmarkmap, whose landmarks will be inserted
	 * into "this"
	 *  \param justInsertAllOfThem If set to "true", all the landmarks in
	 * "other" will be inserted into "this" without checking for possible
	 * correspondences (may appear duplicates ones, etc...)
	 */
	void fuseWith(CLandmarksMap& other, bool justInsertAllOfThem = false);

	/** Returns the (logarithmic) likelihood of a set of landmarks "map" given
	 * "this" map.
	 *  See paper: JJAA 2006
	 */
	double computeLikelihood_SIFT_LandmarkMap(
		CLandmarksMap* map,
		mrpt::tfest::TMatchingPairList* correspondences = nullptr,
		std::vector<bool>* otherCorrespondences = nullptr);

	/** Returns true if the map is empty/no observation has been inserted.
	 */
	bool isEmpty() const override;

	/** Simulates a noisy reading toward each of the beacons in the landmarks
	 * map, if any.
	 * \param in_robotPose This robot pose is used to simulate the ranges to
	 * each beacon.
	 * \param in_sensorLocationOnRobot The 3D position of the sensor on the
	 * robot
	 * \param out_Observations The results will be stored here. NOTICE that the
	 * fields
	 * "CObservationBeaconRanges::minSensorDistance","CObservationBeaconRanges::maxSensorDistance"
	 * and "CObservationBeaconRanges::stdError" MUST BE FILLED OUT before
	 * calling this function.
	 * An observation will be generated for each beacon in the map, but notice
	 * that some of them may be missed if out of the sensor maximum range.
	 */
	void simulateBeaconReadings(
		const mrpt::poses::CPose3D& in_robotPose,
		const mrpt::poses::CPoint3D& in_sensorLocationOnRobot,
		mrpt::obs::CObservationBeaconRanges& out_Observations) const;

	/** Simulates a noisy bearing-range observation of all the beacons
	 * (landamrks with type glBeacon) in the landmarks map, if any.
	 * \param[in]  robotPose The robot pose.
	 * \param[in]  sensorLocationOnRobot The 3D position of the sensor on the
	 * robot
	 * \param[out] observations The results will be stored here.
	 * \param[in]  sensorDetectsIDs If this is set to false, all the landmarks
	 * will be sensed with INVALID_LANDMARK_ID as ID.
	 * \param[in]  stdRange The sigma of the sensor noise in range (meters).
	 * \param[in]  stdYaw The sigma of the sensor noise in yaw (radians).
	 * \param[in]  stdPitch The sigma of the sensor noise in pitch (radians).
	 * \param[out] real_associations If it's not a nullptr pointer, this will
	 * contain at the return the real indices of the landmarks in the map in the
	 * same order than they appear in out_Observations. Useful when
	 * sensorDetectsIDs=false. Spurious readings are assigned a
	 * std::string::npos (=-1) index.
	 * \param[in]  spurious_count_mean The mean number of spurious measurements
	 * (uniformly distributed in range & angle) to generate. The number of
	 * spurious is generated by rounding a random Gaussin number. If both this
	 * mean and the std are zero (the default) no spurious readings are
	 * generated.
	 * \param[in]  spurious_count_std  Read spurious_count_mean above.
	 *
	 * \note The fields
	 * "CObservationBearingRange::fieldOfView_*","CObservationBearingRange::maxSensorDistance"
	 * and "CObservationBearingRange::minSensorDistance" MUST BE FILLED OUT
	 * before calling this function.
	 * \note At output, the observation will have
	 * CObservationBearingRange::validCovariances set to "false" and the 3
	 * sensor_std_* members correctly set to their values.
	 * An observation will be generated for each beacon in the map, but notice
	 * that some of them may be missed if out of the sensor maximum range or
	 * field of view-
	 */
	void simulateRangeBearingReadings(
		const mrpt::poses::CPose3D& robotPose,
		const mrpt::poses::CPose3D& sensorLocationOnRobot,
		mrpt::obs::CObservationBearingRange& observations,
		bool sensorDetectsIDs = true, const float stdRange = 0.01f,
		const float stdYaw = mrpt::DEG2RAD(0.1f),
		const float stdPitch = mrpt::DEG2RAD(0.1f),
		std::vector<size_t>* real_associations = nullptr,
		const double spurious_count_mean = 0,
		const double spurious_count_std = 0) const;

	/** This virtual method saves the map to a file "filNamePrefix"+<
	 *some_file_extension >, as an image or in any other applicable way (Notice
	 *that other methods to save the map may be implemented in classes
	 *implementing this virtual interface).
	 *  In the case of this class, these files are generated:
	 *		- "filNamePrefix"+"_3D.m": A script for MATLAB for drawing landmarks
	 *as
	 *3D ellipses.
	 *		- "filNamePrefix"+"_3D.3DScene": A 3D scene with a "ground plane
	 *grid"
	 *and the set of ellipsoids in 3D.
	 */
	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	/** Returns a 3D object representing the map.
	 * \sa COLOR_LANDMARKS_IN_3DSCENES
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	// See base docs
	void auxParticleFilterCleanUp() override;

	MAP_DEFINITION_START(CLandmarksMap)
	using TPairIdBeacon = std::pair<mrpt::math::TPoint3D, unsigned int>;
	/** Initial contents of the map, especified by a set of 3D Beacons with
	 * associated IDs */
	std::deque<TPairIdBeacon> initialBeacons;
	mrpt::maps::CLandmarksMap::TInsertionOptions insertionOpts;
	mrpt::maps::CLandmarksMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CLandmarksMap)

};  // End of class def.

}  // namespace maps
}  // namespace mrpt
