/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMetricMap_H
#define CMetricMap_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/utils/CObservable.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/slam/CMetricMapEvents.h>
#include <mrpt/obs/obs_frwds.h>
#include <deque>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::utils;

		/** Parameters for the determination of matchings between point clouds, etc. \sa CMetricMap::determineMatching2D, CMetricMap::determineMatching3D */
		struct OBS_IMPEXP TMatchingParams
		{
			float  maxDistForCorrespondence;          //!< Maximum linear distance between two points to be paired (meters)
			float  maxAngularDistForCorrespondence;   //!< Allowed "angular error" (in radians): this permits larger pairing threshold distances to more distant points.
			bool   onlyKeepTheClosest;  //!< If set to true (default), only the closest correspondence will be returned. If false all are returned.
			bool   onlyUniqueRobust;    //!< Additional consistency filter: "onlyKeepTheClosest" allows one correspondence for each "local map" point, but many of them may have as corresponding pair the same "global point", which this flag avoids.
			size_t decimation_other_map_points; //!< (Default=1) Only consider 1 out of this number of points from the "other" map.
			size_t offset_other_map_points;  //!< Index of the first point in the "other" map to start checking for correspondences (Default=0)
			mrpt::math::TPoint3D angularDistPivotPoint; //!< The point used to calculate angular distances: e.g. the coordinates of the sensor for a 2D laser scanner.

			/** Ctor: default values */
			TMatchingParams() :
				maxDistForCorrespondence(0.50f),
				maxAngularDistForCorrespondence(.0f),
				onlyKeepTheClosest(true),
				onlyUniqueRobust(false),
				decimation_other_map_points(1),
				offset_other_map_points(0),
				angularDistPivotPoint(0,0,0)
			{}
		};

		/** Additional results from the determination of matchings between point clouds, etc., apart from the pairings themselves \sa CMetricMap::determineMatching2D, CMetricMap::determineMatching3D */
		struct OBS_IMPEXP TMatchingExtraResults
		{
			float correspondencesRatio; //!< The ratio [0,1] of points in otherMap with at least one correspondence.
			float sumSqrDist;           //!< The sum of all matched points squared distances.If undesired, set to NULL, as default.

			TMatchingExtraResults() : correspondencesRatio(0),sumSqrDist(0)
			{}
		};

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CMetricMap, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a virtual base class for all metric maps storage classes.
		 *  In this class virtual methods are provided to allow the insertion
		 *  of any type of "CObservation" objects into the metric map, thus
		 *  updating the map (doesn't matter if it is a 2D/3D grid, a point map, etc.).
		 *
		 *  Observations don't include any information about the
		 *  robot pose, just the raw observation and information about
		 *  the sensor pose relative to the robot mobile base coordinates origin.
		 *
		 *  Note that all metric maps implement this mrpt::utils::CObservable interface,
		 *   emitting the following events:
		 *	  - mrpt::slam::mrptEventMetricMapClear: Upon call of the ::clear() method.
		 *    - mrpt::slam::mrptEventMetricMapInsert: Upon insertion of an observation that effectively modifies the map (e.g. inserting an image into a grid map will NOT raise an event, inserting a laser scan will).
		 *
		 * \sa CObservation, CSensoryFrame, CMultiMetricMap
	 	 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP CMetricMap :
			public mrpt::utils::CSerializable,
			public mrpt::utils::CObservable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_SERIALIZABLE( CMetricMap )

		private:
			/** Internal method called by clear() */
			virtual void  internal_clear() = 0;

			/** Hook for each time a "internal_insertObservation" returns "true"
			  * This is called automatically from insertObservation() when internal_insertObservation returns true.
			  */
			virtual void OnPostSuccesfulInsertObs(const CObservation *)
			{
				// Default: do nothing
			}

			/** Internal method called by insertObservation() */
			virtual bool  internal_insertObservation(
				const CObservation *obs,
				const CPose3D *robotPose = NULL ) = 0;

		public:
			/** Erase all the contents of the map */
			void  clear();

			/** Returns true if the map is empty/no observation has been inserted.
			  */
			virtual bool  isEmpty() const = 0;

			/** Load the map contents from a CSimpleMap object, erasing all previous content of the map.
			 *  This is automaticed invoking "insertObservation" for each observation at the mean 3D robot pose as
			 *   given by the "poses::CPosePDF" in the CSimpleMap object.
			 *
			 * \sa insertObservation, CSimpleMap
			 * \exception std::exception Some internal steps in invoked methods can raise exceptions on invalid parameters, etc...
			 */
			void  loadFromProbabilisticPosesAndObservations( const CSimpleMap &Map );

			/** Load the map contents from a CSimpleMap object, erasing all previous content of the map.
			 *  This is automaticed invoking "insertObservation" for each observation at the mean 3D robot pose as
			 *   given by the "poses::CPosePDF" in the CSimpleMap object.
			 *
			 * \sa insertObservation, CSimpleMap
			 * \exception std::exception Some internal steps in invoked methods can raise exceptions on invalid parameters, etc...
			 */
			inline void  loadFromSimpleMap( const CSimpleMap &Map ) {  loadFromProbabilisticPosesAndObservations(Map); }

			/** Insert the observation information into this map. This method must be implemented
			 *    in derived classes.
			 * \param obs The observation
			 * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use the origin.
			 *
			 * \sa CObservation::insertObservationInto
			 */
			inline bool  insertObservation(
				const CObservation *obs,
				const CPose3D *robotPose = NULL )
			{
				bool done = internal_insertObservation(obs,robotPose);
				if (done)
				{
					OnPostSuccesfulInsertObs(obs);
					publishEvent( mrptEventMetricMapInsert(this,obs,robotPose) );
				}
				return done;
			}

			/** A wrapper for smart pointers, just calls the non-smart pointer version. */
			inline bool  insertObservationPtr(
				const CObservationPtr &obs,
				const CPose3D *robotPose = NULL )
			{
				MRPT_START
				if (!obs.present()) { THROW_EXCEPTION("Trying to pass a null pointer."); }
				return insertObservation(obs.pointer(),robotPose);
				MRPT_END
			}

			/** Computes the log-likelihood of a given observation given an arbitrary robot 3D pose.
			 *
			 * \param takenFrom The robot's pose the observation is supposed to be taken from.
			 * \param obs The observation.
			 * \return This method returns a log-likelihood.
			 *
			 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
			 */
			virtual double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom ) = 0;

			/** Computes the log-likelihood of a given observation given an arbitrary robot 2D pose.
			 *
			 * \param takenFrom The robot's pose the observation is supposed to be taken from.
			 * \param obs The observation.
			 * \return This method returns a log-likelihood.
			 *
			 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
			 */
			double	 computeObservationLikelihood( const CObservation *obs, const CPose2D &takenFrom );

			/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
			 * \param obs The observation.
			 * \sa computeObservationLikelihood
			 */
			virtual bool canComputeObservationLikelihood( const CObservation *obs )
			{
				return true; // Unless implemented otherwise, assume we can always compute the likelihood.
			}

			/** \overload */
			inline bool canComputeObservationLikelihood( const CObservationPtr &obs ) { return canComputeObservationLikelihood(obs.pointer()); }

			/** Returns the sum of the log-likelihoods of each individual observation within a mrpt::slam::CSensoryFrame.
			 *
			 * \param takenFrom The robot's pose the observation is supposed to be taken from.
			 * \param sf The set of observations in a CSensoryFrame.
			 * \return This method returns a log-likelihood.
			 * \sa canComputeObservationsLikelihood
			 */
			double computeObservationsLikelihood( const CSensoryFrame &sf, const CPose2D &takenFrom );

			/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
			 * \param sf The observations.
			 * \sa canComputeObservationLikelihood
			 */
			bool canComputeObservationsLikelihood( const CSensoryFrame &sf );

			/** Constructor */
			CMetricMap();

			/** Destructor */
			virtual ~CMetricMap();

			/** Computes the matching between this and another 2D point map, which includes finding:
			 *		- The set of points pairs in each map
			 *		- The mean squared distance between corresponding pairs.
			 *
			 *   The algorithm is:
			 *		- For each point in "otherMap":
			 *			- Transform the point according to otherMapPose
			 *			- Search with a KD-TREE the closest correspondences in "this" map.
			 *			- Add to the set of candidate matchings, if it passes all the thresholds in params.
			 *
			 *   This method is the most time critical one into ICP-like algorithms.
			 *
			 * \param  otherMap        [IN] The other map to compute the matching with.
			 * \param  otherMapPose    [IN] The pose of the other map as seen from "this".
			 * \param  params          [IN] Parameters for the determination of pairings.
			 * \param  correspondences [OUT] The detected matchings pairs.
			 * \param  extraResults    [OUT] Other results.
			 * \sa compute3DMatchingRatio
			 */
			virtual void  determineMatching2D(
				const CMetricMap      * otherMap,
				const CPose2D         & otherMapPose,
				TMatchingPairList     & correspondences,
				const TMatchingParams & params,
				TMatchingExtraResults & extraResults ) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
			}

			/** Computes the matchings between this and another 3D points map - method used in 3D-ICP.
			 *  This method finds the set of point pairs in each map.
			 *
			 *  The method is the most time critical one into ICP-like algorithms.
			 *
			 *  The algorithm is:
			 *		- For each point in "otherMap":
			 *			- Transform the point according to otherMapPose
			 *			- Search with a KD-TREE the closest correspondences in "this" map.
			 *			- Add to the set of candidate matchings, if it passes all the thresholds in params.
			 *
			 * \param  otherMap        [IN] The other map to compute the matching with.
			 * \param  otherMapPose    [IN] The pose of the other map as seen from "this".
			 * \param  params          [IN] Parameters for the determination of pairings.
			 * \param  correspondences [OUT] The detected matchings pairs.
			 * \param  extraResults    [OUT] Other results.
			 * \sa compute3DMatchingRatio
			 */
			virtual void  determineMatching3D(
				const CMetricMap      * otherMap,
				const CPose3D         & otherMapPose,
				TMatchingPairList     & correspondences,
				const TMatchingParams & params,
				TMatchingExtraResults & extraResults ) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
			}


			/** DEPRECATED: Use alternative version determineMatching2D with struct parameters instead. */
			MRPT_DEPRECATED_PRE("Deprecated")
			void  computeMatchingWith2D(
				const CMetricMap						*otherMap,
				const CPose2D							&otherMapPose,
				float									maxDistForCorrespondence,
				float									maxAngularDistForCorrespondence,
				const CPose2D							&angularDistPivotPoint,
				TMatchingPairList						&correspondences,
				float									&correspondencesRatio,
				float									*sumSqrDist	= NULL,
				bool									onlyKeepTheClosest = true,
				bool									onlyUniqueRobust = false,
				const size_t                            decimation_other_map_points = 1,
				const size_t                            offset_other_map_points = 0 ) const MRPT_DEPRECATED_POST("Deprecated");

			/** DEPRECATED: Use alternative version determineMatching3D with struct parameters instead. */
			MRPT_DEPRECATED_PRE("Deprecated")
			void  computeMatchingWith3D(
				const CMetricMap						*otherMap,
				const CPose3D							&otherMapPose,
				float									maxDistForCorrespondence,
				float									maxAngularDistForCorrespondence,
				const CPoint3D							&angularDistPivotPoint,
				TMatchingPairList						&correspondences,
				float									&correspondencesRatio,
				float									*sumSqrDist	= NULL,
				bool									onlyKeepTheClosest = true,
				bool									onlyUniqueRobust = false,
				const size_t                            decimation_other_map_points = 1,
				const size_t                            offset_other_map_points = 0 ) const MRPT_DEPRECATED_POST("Deprecated");


			/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
			 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
			 * \param  otherMap					  [IN] The other map to compute the matching with.
			 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
			 * \param  maxDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
			 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
			 *
			 * \return The matching ratio [0,1]
			 * \sa determineMatching2D
			 */
			virtual float  compute3DMatchingRatio(
				const CMetricMap								*otherMap,
				const CPose3D							&otherMapPose,
				float									maxDistForCorr = 0.10f,
				float									maxMahaDistForCorr = 2.0f
			) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
			}


			/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
			  */
			virtual void  saveMetricMapRepresentationToFile(
				const std::string	&filNamePrefix
			) const = 0;

			/** Returns a 3D object representing the map.
			  */
			virtual void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const = 0;

			/** When set to true (default=false), calling "getAs3DObject" will have no effects.
			  */
			bool			m_disableSaveAs3DObject;

			/** This method is called at the end of each "prediction-update-map insertion" cycle within "mrpt::slam::CMetricMapBuilderRBPF::processActionObservation".
			  *  This method should normally do nothing, but in some cases can be used to free auxiliary cached variables.
			  */
			virtual void  auxParticleFilterCleanUp()
			{
				// Default implementation: do nothing.
			}

			/** Returns the square distance from the 2D point (x0,y0) to the closest correspondence in the map.
			  */
			virtual float squareDistanceToClosestCorrespondence(
				float x0,
				float y0 ) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
			}


			/** If the map is a simple points map or it's a multi-metric map that contains EXACTLY one simple points map, return it.
			  * Otherwise, return NULL
			  */
			virtual const CSimplePointsMap * getAsSimplePointsMap() const { return NULL; }
			virtual       CSimplePointsMap * getAsSimplePointsMap()       { return NULL; }


		}; // End of class def.

		/** A list of metric maps (used in the mrpt::poses::CPosePDFParticles class):
		  */
		typedef std::deque<CMetricMap*> TMetricMapList;

	} // End of namespace
} // End of namespace

#endif
