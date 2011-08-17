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
#ifndef CMetricMap_H
#define CMetricMap_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/TMatchingPair.h>

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/utils/CObservable.h>
#include <mrpt/slam/CMetricMapEvents.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::utils;

		class CObservation;
		class CPointsMap;
		class CSimplePointsMap;
		class CSimpleMap;
		class CSensoryFrame;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CMetricMap, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a virtual base class for all metric maps storage classes.
			   In this class virtual methods are provided to allow the insertion
				of any type of "CObservation" objects into the metric map, thus
				updating the map (doesn't matter if it is a 2D/3D grid or a points
				map).
			   <b>IMPORTANT</b>: Observations doesn't include any information about the
				robot pose beliefs, just the raw observation and information about
				the sensor pose relative to the robot mobile base coordinates origin.
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
			double	 computeObservationLikelihood( const CObservation *obs, const CPose2D &takenFrom )
			{
				return computeObservationLikelihood(obs,CPose3D(takenFrom));
			}

			/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
			 * \param obs The observation.
			 * \sa computeObservationLikelihood
			 */
			virtual bool canComputeObservationLikelihood( const CObservation *obs )
			{
				return true; // Unless implemented otherwise, we can always compute the likelihood.
			}

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

			/** Constructor
			  */
			CMetricMap();

			/** Destructor
			  */
			virtual ~CMetricMap();

#ifdef MRPT_BACKCOMPATIB_08X	// For backward compatibility
			typedef mrpt::utils::TMatchingPair   	TMatchingPair;
			typedef mrpt::utils::TMatchingPairPtr   TMatchingPairPtr;
			typedef mrpt::utils::TMatchingPairList 	TMatchingPairList;
#endif

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
			 * \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
			 *
			 * \sa compute3DMatchingRatio
			 */
			virtual void  computeMatchingWith2D(
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
				const size_t                            offset_other_map_points = 0 ) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
			}

			/** Computes the matchings between this and another 3D points map - method used in 3D-ICP.
			   This method finds the set of point pairs in each map.

			   The method is the most time critical one into the ICP algorithm.

			 * \param  otherMap					  [IN] The other map to compute the matching with.
			 * \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
			 * \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
			 * \param  maxAngularDistForCorrespondence [IN] In radians: The aim is to allow larger distances to more distant correspondences.
			 * \param  angularDistPivotPoint      [IN] The point used to calculate distances from in both maps.
			 * \param  correspondences			  [OUT] The detected matchings pairs.
			 * \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
			 * \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
			 * \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
			 *
			 * \sa compute3DMatchingRatio
			 */
			virtual void  computeMatchingWith3D(
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
				const size_t                            offset_other_map_points = 0 ) const
			{
				MRPT_START
				THROW_EXCEPTION("Virtual method not implemented in derived class.")
				MRPT_END
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
			virtual float  compute3DMatchingRatio(
				const CMetricMap								*otherMap,
				const CPose3D							&otherMapPose,
				float									minDistForCorr = 0.10f,
				float									minMahaDistForCorr = 2.0f
			) const = 0;

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
				const float   &x0,
				const float   &y0 ) const
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
