/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CBeaconMap_H
#define CBeaconMap_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CBeacon.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CBeaconMap, CMetricMap ,MAPS_IMPEXP )

	/** A class for storing a map of 3D probabilistic beacons, using a Montecarlo, Gaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).
	 * <br>
	 *  The individual beacons are defined as mrpt::maps::CBeacon objects.
	 * <br>
	 *  When invoking CBeaconMap::insertObservation(), landmarks will be extracted and fused into the map.
	 *   The only currently supported observation type is mrpt::obs::CObservationBeaconRanges.
	 *   See insertionOptions and likelihoodOptions for parameters used when creating and fusing beacon landmarks.
	 * <br>
	 *   Use "TInsertionOptions::insertAsMonteCarlo" to select between 2 different behaviors:
	 *		- Initial PDF of beacons: MonteCarlo, after convergence, pass to Gaussians; or
	 *		- Initial PDF of beacons: SOG, after convergence, a single Gaussian.
	 *
	 *   Refer to the papers: []
	 *
	  * \ingroup mrpt_maps_grp
	 * \sa CMetricMap
	 */
	class MAPS_IMPEXP CBeaconMap : public mrpt::maps::CMetricMap
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CBeaconMap )

	public:
		typedef std::deque<CBeacon>					TSequenceBeacons;
		typedef std::deque<CBeacon>::iterator			iterator;
		typedef std::deque<CBeacon>::const_iterator	const_iterator;

	protected:
		TSequenceBeacons		m_beacons;  //!< The individual beacons

		// See docs in base class
		virtual void  internal_clear() MRPT_OVERRIDE;
		virtual bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL ) MRPT_OVERRIDE;
		double	 internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

	public:
		/** Constructor */
		CBeaconMap();

		void resize(const size_t N); //!< Resize the number of SOG modes

		/** Access to individual beacons */
		const CBeacon& operator [](size_t i) const {
			ASSERT_(i<m_beacons.size())
			return  m_beacons[i];
		}
		/** Access to individual beacons */
		const CBeacon& get(size_t i) const{
			ASSERT_(i<m_beacons.size())
			return  m_beacons[i];
		}
		/** Access to individual beacons */
		CBeacon& operator [](size_t i) {
			ASSERT_(i<m_beacons.size())
			return  m_beacons[i];
		}
		/** Access to individual beacons */
		CBeacon& get(size_t i)  {
			ASSERT_(i<m_beacons.size())
			return  m_beacons[i];
		}

		iterator begin() { return m_beacons.begin(); }
		const_iterator begin() const { return m_beacons.begin(); }
		iterator end() { return m_beacons.end(); }
		const_iterator end() const { return m_beacons.end(); }

		/** Inserts a copy of the given mode into the SOG */
		void push_back(const CBeacon& m) {
			m_beacons.push_back( m );
		}

		// See docs in base class
		float compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const MRPT_OVERRIDE;

		 /** With this struct options are provided to the likelihood computations */
		 struct MAPS_IMPEXP TLikelihoodOptions : public utils::CLoadableOptions
		 {
		 public:
			/** Initilization of default parameters
			 */
			 TLikelihoodOptions();

			 void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			 void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			 /** The standard deviation used for Beacon ranges likelihood (default=0.08m).
			   */
			 float			rangeStd;
		 } likelihoodOptions;

		 /** This struct contains data for choosing the method by which new beacons are inserted in the map.
		  */
		 struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		 {
		 public:
			/** Initilization of default parameters */
			 TInsertionOptions();
			 void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			 void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** Insert a new beacon as a set of montecarlo samples (default=true), or, if false, as a sum of gaussians (see mrpt::maps::CBeacon).
			  * \sa MC_performResampling
			  */
			bool	insertAsMonteCarlo;

			/** Minimum and maximum elevation angles (in degrees) for inserting new beacons at the first observation: the default values (both 0), makes the beacons to be in the same horizontal plane that the sensors, that is, 2D SLAM - the min/max values are -90/90.
			  */
			float	maxElevation_deg,minElevation_deg;

			/** Number of particles per meter of range, i.e. per meter of the "radius of the ring".
			  */
			unsigned int	MC_numSamplesPerMeter;

			/** The threshold for the maximum std (X,Y,and Z) before colapsing the particles into a Gaussian PDF (default=0,4).
			  */
			float			MC_maxStdToGauss;

			/** Threshold for the maximum difference from the maximun (log) weight in the set of samples for erasing a given sample (default=5).
			  */
			float			MC_thresholdNegligible;

			/** If set to false (default), the samples will be generated the first time a beacon is observed, and their weights just updated subsequently - if set to "true", fewer samples will be required since the particles will be resamples when necessary, and a small "noise" will be added to avoid depletion.
			  */
			bool			MC_performResampling;

			/** The std.dev. of the Gaussian noise to be added to each sample after resampling, only if MC_performResampling=true.
			  */
			float			MC_afterResamplingNoise;

			/** Threshold for the maximum difference from the maximun (log) weight in the SOG for erasing a given mode (default=20).
			  */
			float			SOG_thresholdNegligible;

			/** A parameter for initializing 2D/3D SOGs
			  */
			float			SOG_maxDistBetweenGaussians;

			/** Constant used to compute the std. dev. int the tangent direction when creating the Gaussians.
			  */
			float			SOG_separationConstant;

		 } insertionOptions;

		/** Save to a MATLAB script which displays 3D error ellipses for the map.
		 *	\param file		The name of the file to save the script to.
		 *  \param style	The MATLAB-like string for the style of the lines (see 'help plot' in MATLAB for posibilities)
		 *  \param stdCount The ellipsoids will be drawn from the center to a given confidence interval in [0,1], e.g. 2 sigmas=0.95 (default is 2std = 0.95 confidence intervals)
		 *
		 *  \return Returns false if any error occured, true elsewere.
		 */
		bool  saveToMATLABScript3D(
			const std::string	&file,
			const char	*style="b",
			float		confInterval = 0.95f ) const;


		/** Returns the stored landmarks count.
		 */
		size_t  size() const;

		// See docs in base class
		virtual void  determineMatching2D(
			const mrpt::maps::CMetricMap      * otherMap,
			const mrpt::poses::CPose2D         & otherMapPose,
			mrpt::utils::TMatchingPairList     & correspondences,
			const TMatchingParams & params,
			TMatchingExtraResults & extraResults ) const MRPT_OVERRIDE;

		/** Perform a search for correspondences between "this" and another lansmarks map:
		  *  Firsly, the landmarks' descriptor is used to find correspondences, then inconsistent ones removed by
		  *    looking at their 3D poses.
		  * \param otherMap [IN] The other map.
		  * \param correspondences [OUT] The matched pairs between maps.
		  * \param correspondencesRatio [OUT] This is NumberOfMatchings / NumberOfLandmarksInTheAnotherMap
		  * \param otherCorrespondences [OUT] Will be returned with a vector containing "true" for the indexes of the other map's landmarks with a correspondence.
		  */
		void  computeMatchingWith3DLandmarks(
			const mrpt::maps::CBeaconMap					*otherMap,
			mrpt::utils::TMatchingPairList						&correspondences,
			float									&correspondencesRatio,
			std::vector<bool>						&otherCorrespondences) const;

		/** Changes the reference system of the map to a given 3D pose.
		  */
		void  changeCoordinatesReference( const mrpt::poses::CPose3D &newOrg );

		/** Changes the reference system of the map "otherMap" and save the result in "this" map.
		  */
		void  changeCoordinatesReference( const mrpt::poses::CPose3D &newOrg, const mrpt::maps::CBeaconMap *otherMap );


		/** Returns true if the map is empty/no observation has been inserted.
		   */
		bool isEmpty() const MRPT_OVERRIDE;

		/** Simulates a reading toward each of the beacons in the landmarks map, if any.
		  * \param in_robotPose This robot pose is used to simulate the ranges to each beacon.
		  * \param in_sensorLocationOnRobot The 3D position of the sensor on the robot
		  * \param out_Observations The results will be stored here. NOTICE that the fields "CObservationBeaconRanges::minSensorDistance","CObservationBeaconRanges::maxSensorDistance" and "CObservationBeaconRanges::stdError" MUST BE FILLED OUT before calling this function.
		  * An observation will be generated for each beacon in the map, but notice that some of them may be missed if out of the sensor maximum range.
		  */
		void  simulateBeaconReadings(
			const mrpt::poses::CPose3D					&in_robotPose,
			const mrpt::poses::CPoint3D					&in_sensorLocationOnRobot,
			mrpt::obs::CObservationBeaconRanges		&out_Observations ) const;

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
		  *  In the case of this class, these files are generated:
		  *		- "filNamePrefix"+"_3D.m": A script for MATLAB for drawing landmarks as 3D ellipses.
		  *		- "filNamePrefix"+"_3D.3DScene": A 3D scene with a "ground plane grid" and the set of ellipsoids in 3D.
		  *		- "filNamePrefix"+"_covs.m": A textual representation (see saveToTextFile)
		  */
		void  saveMetricMapRepresentationToFile(const std::string	&filNamePrefix ) const MRPT_OVERRIDE;

		/** Save a text file with a row per beacon, containing this 11 elements:
		  *  - X Y Z: Mean values
		  *  - VX VY VZ: Variances of each dimension (C11, C22, C33)
		  *  - DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
		  *  - C12, C13, C23: Cross covariances
		  */
		void saveToTextFile(const std::string &fil) const;

		void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj) const MRPT_OVERRIDE; //!< Returns a 3D object representing the map.

		const CBeacon * getBeaconByID( CBeacon::TBeaconID  id ) const; //!< Returns a pointer to the beacon with the given ID, or NULL if it does not exist.
		CBeacon * getBeaconByID( CBeacon::TBeaconID  id ); 		//!< Returns a pointer to the beacon with the given ID, or NULL if it does not exist.


		MAP_DEFINITION_START(CBeaconMap,MAPS_IMPEXP)
			mrpt::maps::CBeaconMap::TInsertionOptions	insertionOpts;	//!< Observations insertion options
			mrpt::maps::CBeaconMap::TLikelihoodOptions	likelihoodOpts;	//!< Probabilistic observation likelihood options
		MAP_DEFINITION_END(CBeaconMap,MAPS_IMPEXP)

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CBeaconMap, CMetricMap ,MAPS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
