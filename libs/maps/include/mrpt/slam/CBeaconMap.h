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
#ifndef CBeaconMap_H
#define CBeaconMap_H

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CBeacon.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::math;

	class CObservationBeaconRanges;


	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CBeaconMap, CMetricMap ,MAPS_IMPEXP )

	/** A class for storing a map of 3D probabilistic beacons, using a Montecarlo, Gaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).
	 * <br>
	 *  The individual beacons are defined as mrpt::slam::CBeacon objects.
	 * <br>
	 *  When invoking CBeaconMap::insertObservation(), landmarks will be extracted and fused into the map.
	 *   The only currently supported observation type is mrpt::slam::CObservationBeaconRanges.
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
	class MAPS_IMPEXP CBeaconMap : public CMetricMap
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CBeaconMap )

	public:
		typedef std::deque<CBeacon>					TSequenceBeacons;
		typedef std::deque<CBeacon>::iterator			iterator;
		typedef std::deque<CBeacon>::const_iterator	const_iterator;

	protected:
		/** The individual beacons */
		TSequenceBeacons		m_beacons;

		/** Clear the map, erasing all landmarks.
		 */
		virtual void  internal_clear();

		 /** Insert the observation information into this map. This method must be implemented
		  *    in derived classes.
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  * \sa CObservation::insertObservationInto
		  */
		 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

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

		/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
		 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
		 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
		 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
		 *
		 * \return The matching ratio [0,1]
		 * \sa computeMatchingWith2D
		 */
		float  compute3DMatchingRatio(
				const CMetricMap	*otherMap,
				const CPose3D		&otherMapPose,
				float				minDistForCorr = 0.10f,
				float				minMahaDistForCorr = 2.0f
				) const;

		 /** With this struct options are provided to the likelihood computations.
		  */
		 struct MAPS_IMPEXP TLikelihoodOptions : public utils::CLoadableOptions
		 {
		 public:
			/** Initilization of default parameters
			 */
			 TLikelihoodOptions();

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string &section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

			 /** The standard deviation used for Beacon ranges likelihood (default=0.08m).
			   */
			 float			rangeStd;

		 } likelihoodOptions;

		 /** This struct contains data for choosing the method by which new beacons are inserted in the map.
		  */
		 struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		 {
		 public:
			/** Initilization of default parameters
			 */
			 TInsertionOptions();

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string &section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

			/** Insert a new beacon as a set of montecarlo samples (default=true), or, if false, as a sum of gaussians (see mrpt::slam::CBeacon).
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
			std::string	file,
			const char	*style="b",
			float		confInterval = 0.95f ) const;


		/** Returns the stored landmarks count.
		 */
		size_t  size() const;


		/** Computes the (logarithmic) likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 *  In the current implementation, this method behaves in a different way according to the nature of
		 *   the observation's class:
		 *		- "mrpt::slam::CObservation2DRangeScan": This calls "computeLikelihood_RSLC_2007".
		 *		- "mrpt::slam::CObservationStereoImages": This calls "computeLikelihood_SIFT_LandmarkMap".
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood value > 0.
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

		/** Computes the matchings between this and another 2D points map.
		   This includes finding:
				- The set of points pairs in each map
				- The mean squared distance between corresponding pairs.
		   This method is the most time critical one into the ICP algorithm.

		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
		 * \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
		 * \param  maxAngularDistForCorrespondence [IN] In radians: The aim is to allow larger distances to more distant correspondences.
		 * \param  angularDistPivotPoint      [IN] The point used to calculate distances from in both maps.
		 * \param  correspondences			  [OUT] The detected matchings pairs.
		 * \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
		 * \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
		 * \param  covariance				  [OUT] The resulting matching covariance 3x3 matrix, or NULL if undesired.
		 * \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
		 *
		 * \sa compute3DMatchingRatio
		 */
		void  computeMatchingWith2D(
				const CMetricMap     *otherMap,
				const CPose2D        &otherMapPose,
				float                maxDistForCorrespondence,
				float                maxAngularDistForCorrespondence,
				const CPose2D        &angularDistPivotPoint,
				TMatchingPairList    &correspondences,
				float                &correspondencesRatio,
				float                *sumSqrDist	= NULL,
				bool                  onlyKeepTheClosest = false,
				bool                  onlyUniqueRobust = false,
				const size_t          decimation_other_map_points = 1,
				const size_t          offset_other_map_points = 0 ) const;

		/** Perform a search for correspondences between "this" and another lansmarks map:
		  *  Firsly, the landmarks' descriptor is used to find correspondences, then inconsistent ones removed by
		  *    looking at their 3D poses.
		  * \param otherMap [IN] The other map.
		  * \param correspondences [OUT] The matched pairs between maps.
		  * \param correspondencesRatio [OUT] This is NumberOfMatchings / NumberOfLandmarksInTheAnotherMap
		  * \param otherCorrespondences [OUT] Will be returned with a vector containing "true" for the indexes of the other map's landmarks with a correspondence.
		  */
		void  computeMatchingWith3DLandmarks(
				const mrpt::slam::CBeaconMap					*otherMap,
				TMatchingPairList						&correspondences,
				float									&correspondencesRatio,
				std::vector<bool>						&otherCorrespondences) const;

		/** Changes the reference system of the map to a given 3D pose.
		  */
		void  changeCoordinatesReference( const CPose3D &newOrg );

		/** Changes the reference system of the map "otherMap" and save the result in "this" map.
		  */
		void  changeCoordinatesReference( const CPose3D &newOrg, const mrpt::slam::CBeaconMap *otherMap );


		/** Returns true if the map is empty/no observation has been inserted.
		   */
		bool  isEmpty() const;

		/** Simulates a reading toward each of the beacons in the landmarks map, if any.
		  * \param in_robotPose This robot pose is used to simulate the ranges to each beacon.
		  * \param in_sensorLocationOnRobot The 3D position of the sensor on the robot
		  * \param out_Observations The results will be stored here. NOTICE that the fields "CObservationBeaconRanges::minSensorDistance","CObservationBeaconRanges::maxSensorDistance" and "CObservationBeaconRanges::stdError" MUST BE FILLED OUT before calling this function.
		  * An observation will be generated for each beacon in the map, but notice that some of them may be missed if out of the sensor maximum range.
		  */
		void  simulateBeaconReadings(
            const CPose3D					&in_robotPose,
			const CPoint3D					&in_sensorLocationOnRobot,
			CObservationBeaconRanges		&out_Observations ) const;

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
		  *  In the case of this class, these files are generated:
		  *		- "filNamePrefix"+"_3D.m": A script for MATLAB for drawing landmarks as 3D ellipses.
		  *		- "filNamePrefix"+"_3D.3DScene": A 3D scene with a "ground plane grid" and the set of ellipsoids in 3D.
		  *		- "filNamePrefix"+"_covs.m": A textual representation (see saveToTextFile)
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix ) const;

		/** Save a text file with a row per beacon, containing this 11 elements:
		  *  - X Y Z: Mean values
		  *  - VX VY VZ: Variances of each dimension (C11, C22, C33)
		  *  - DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
		  *  - C12, C13, C23: Cross covariances
		  */
		void saveToTextFile(const std::string &fil) const;

		/** Returns a 3D object representing the map.
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Returns a pointer to the beacon with the given ID, or NULL if it does not exist.
		  */
		const CBeacon * getBeaconByID( CBeacon::TBeaconID  id ) const;

		/** Returns a pointer to the beacon with the given ID, or NULL if it does not exist.
		  */
		CBeacon * getBeaconByID( CBeacon::TBeaconID  id );

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
