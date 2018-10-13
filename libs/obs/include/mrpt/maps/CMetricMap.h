/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/system/CObservable.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/CMetricMapEvents.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/obs/obs_frwds.h>
#include <deque>

namespace mrpt::maps
{
/** Declares a virtual base class for all metric maps storage classes.
 *  In this class virtual methods are provided to allow the insertion
 *  of any type of "CObservation" objects into the metric map, thus
 *  updating the map (doesn't matter if it is a 2D/3D grid, a point map, etc.).
 *
 *  Observations don't include any information about the
 *  robot pose, just the raw observation and information about
 *  the sensor pose relative to the robot mobile base coordinates origin.
 *
 *  Note that all metric maps implement this mrpt::system::CObservable
 *interface,
 *   emitting the following events:
 *	  - mrpt::obs::mrptEventMetricMapClear: Upon call of the ::clear() method.
 *    - mrpt::obs::mrptEventMetricMapInsert: Upon insertion of an observation
 *that effectively modifies the map (e.g. inserting an image into a grid map
 *will NOT raise an event, inserting a laser scan will).
 *
 * To check what observations are supported by each metric map, see: \ref
 *maps_observations
 *
 * \note All derived class must implement a static class factory
 *`<metric_map_class>::MapDefinition()` that builds a default
 *TMetricMapInitializer [New in MRPT 1.3.0]
 *
 * \sa CObservation, CSensoryFrame, CMultiMetricMap
 * \ingroup mrpt_obs_grp
 */
class CMetricMap : public mrpt::serialization::CSerializable,
				   public mrpt::system::CObservable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CMetricMap)

   private:
	/** Internal method called by clear() */
	virtual void internal_clear() = 0;

	/** Internal method called by insertObservation() */
	virtual bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) = 0;

	/** Internal method called by computeObservationLikelihood() */
	virtual double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) = 0;
	/** Internal method called by canComputeObservationLikelihood() */
	virtual bool internal_canComputeObservationLikelihood(
		const mrpt::obs::CObservation* obs) const
	{
		MRPT_UNUSED_PARAM(obs);
		return true;  // Unless implemented otherwise, assume we can always
		// compute the likelihood.
	}

	/** Hook for each time a "internal_insertObservation" returns "true"
	 * This is called automatically from insertObservation() when
	 * internal_insertObservation returns true. */
	virtual void OnPostSuccesfulInsertObs(const mrpt::obs::CObservation*)
	{ /* Default: do nothing */
	}

   public:
	/** Erase all the contents of the map */
	void clear();

	/** Returns true if the map is empty/no observation has been inserted.
	 */
	virtual bool isEmpty() const = 0;

	/** Load the map contents from a CSimpleMap object, erasing all previous
	 * content of the map. This is done invoking `insertObservation()` for each
	 * observation at the mean 3D robot pose of each pose-observations pair in
	 * the CSimpleMap object.
	 *
	 * \sa insertObservation, CSimpleMap
	 * \exception std::exception Some internal steps in invoked methods can
	 * raise exceptions on invalid parameters, etc...
	 */
	void loadFromProbabilisticPosesAndObservations(
		const mrpt::maps::CSimpleMap& Map);

	///! \overload
	inline void loadFromSimpleMap(const mrpt::maps::CSimpleMap& Map)
	{
		loadFromProbabilisticPosesAndObservations(Map);
	}

	/** Insert the observation information into this map. This method must be
	 * implemented
	 *    in derived classes. See: \ref maps_observations
	 * \param obs The observation
	 * \param robotPose The 3D pose of the robot mobile base in the map
	 * reference system, or NULL (default) if you want to use the origin.
	 *
	 * \sa CObservation::insertObservationInto
	 */
	bool insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr);

	/** A wrapper for smart pointers, just calls the non-smart pointer version.
	 * See: \ref maps_observations  */
	bool insertObservationPtr(
		const mrpt::obs::CObservation::Ptr& obs,
		const mrpt::poses::CPose3D* robotPose = nullptr);

	/** Computes the log-likelihood of a given observation given an arbitrary
	 * robot 3D pose.  See: \ref maps_observations
	 *
	 * \param takenFrom The robot's pose the observation is supposed to be taken
	 * from.
	 * \param obs The observation.
	 * \return This method returns a log-likelihood.
	 *
	 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
	 */
	double computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom);

	/** \overload */
	double computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);

	/** Returns true if this map is able to compute a sensible likelihood
	 * function for this observation (i.e. an occupancy grid map cannot with an
	 * image).  See: \ref maps_observations
	 * \param obs The observation.
	 * \sa computeObservationLikelihood,
	 * genericMapParams.enableObservationLikelihood
	 */
	virtual bool canComputeObservationLikelihood(
		const mrpt::obs::CObservation* obs) const;

	/** \overload */
	bool canComputeObservationLikelihood(
		const mrpt::obs::CObservation::Ptr& obs) const;

	/** Returns the sum of the log-likelihoods of each individual observation
	 * within a mrpt::obs::CSensoryFrame.  See: \ref maps_observations
	 *
	 * \param takenFrom The robot's pose the observation is supposed to be taken
	 * from.
	 * \param sf The set of observations in a CSensoryFrame.
	 * \return This method returns a log-likelihood.
	 * \sa canComputeObservationsLikelihood
	 */
	double computeObservationsLikelihood(
		const mrpt::obs::CSensoryFrame& sf,
		const mrpt::poses::CPose2D& takenFrom);

	/** Returns true if this map is able to compute a sensible likelihood
	 * function for this observation (i.e. an occupancy grid map cannot with an
	 * image).  See: \ref maps_observations
	 * \param sf The observations.
	 * \sa canComputeObservationLikelihood
	 */
	bool canComputeObservationsLikelihood(
		const mrpt::obs::CSensoryFrame& sf) const;

	/** Constructor */
	CMetricMap();

	/** Computes the matching between this and another 2D point map, which
	 *includes finding:
	 *   - The set of points pairs in each map
	 *   - The mean squared distance between corresponding pairs.
	 *
	 *   The algorithm is:
	 *		- For each point in "otherMap":
	 *			- Transform the point according to otherMapPose
	 *			- Search with a KD-TREE the closest correspondences in "this"
	 *map.
	 *			- Add to the set of candidate matchings, if it passes all the
	 *thresholds in params.
	 *
	 *   This method is the most time critical one into ICP-like algorithms.
	 *
	 * \param  otherMap        [IN] The other map to compute the matching with.
	 * \param  otherMapPose    [IN] The pose of the other map as seen from
	 *"this".
	 * \param  params          [IN] Parameters for the determination of
	 *pairings.
	 * \param  correspondences [OUT] The detected matchings pairs.
	 * \param  extraResults    [OUT] Other results.
	 * \sa compute3DMatchingRatio
	 */
	virtual void determineMatching2D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose2D& otherMapPose,
		mrpt::tfest::TMatchingPairList& correspondences,
		const TMatchingParams& params,
		TMatchingExtraResults& extraResults) const;

	/** Computes the matchings between this and another 3D points map - method
	 *used in 3D-ICP.
	 *  This method finds the set of point pairs in each map.
	 *
	 *  The method is the most time critical one into ICP-like algorithms.
	 *
	 *  The algorithm is:
	 *		- For each point in "otherMap":
	 *			- Transform the point according to otherMapPose
	 *			- Search with a KD-TREE the closest correspondences in "this"
	 *map.
	 *			- Add to the set of candidate matchings, if it passes all the
	 *thresholds in params.
	 *
	 * \param  otherMap        [IN] The other map to compute the matching with.
	 * \param  otherMapPose    [IN] The pose of the other map as seen from
	 *"this".
	 * \param  params          [IN] Parameters for the determination of
	 *pairings.
	 * \param  correspondences [OUT] The detected matchings pairs.
	 * \param  extraResults    [OUT] Other results.
	 * \sa compute3DMatchingRatio
	 */
	virtual void determineMatching3D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		mrpt::tfest::TMatchingPairList& correspondences,
		const TMatchingParams& params,
		TMatchingExtraResults& extraResults) const;

	/** Computes the ratio in [0,1] of correspondences between "this" and the
	 * "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
	 *   In the case of a multi-metric map, this returns the average between the
	 * maps. This method always return 0 for grid maps.
	 * \param  otherMap      [IN] The other map to compute the matching with.
	 * \param  otherMapPose  [IN] The 6D pose of the other map as seen from
	 * "this".
	 * \param  params        [IN] Matching parameters
	 * \return The matching ratio [0,1]
	 * \sa determineMatching2D
	 */
	virtual float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const;

	/** This virtual method saves the map to a file "filNamePrefix"+<
	 * some_file_extension >, as an image or in any other applicable way (Notice
	 * that other methods to save the map may be implemented in classes
	 * implementing this virtual interface). */
	virtual void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const = 0;

	/** Returns a 3D object representing the map.
	 * \sa genericMapParams, TMapGenericParams::enableSaveAs3DObject */
	virtual void getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr& outObj) const = 0;

	/** Common params to all maps */
	TMapGenericParams genericMapParams;

	/** This method is called at the end of each "prediction-update-map
	 * insertion" cycle within
	 * "mrpt::slam::CMetricMapBuilderRBPF::processActionObservation".
	 *  This method should normally do nothing, but in some cases can be used
	 * to free auxiliary cached variables.
	 */
	virtual void auxParticleFilterCleanUp()
	{ /* Default implementation: do nothing. */
	}

	/** Returns the square distance from the 2D point (x0,y0) to the closest
	 * correspondence in the map. */
	virtual float squareDistanceToClosestCorrespondence(
		float x0, float y0) const;

	/** If the map is a simple points map or it's a multi-metric map that
	 * contains EXACTLY one simple points map, return it.
	 * Otherwise, return NULL
	 */
	virtual const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const
	{
		return nullptr;
	}
	virtual mrpt::maps::CSimplePointsMap* getAsSimplePointsMap()
	{
		return nullptr;
	}

};  // End of class def.

/** A list of metric maps (used in the mrpt::poses::CPosePDFParticles class):
 */
using TMetricMapList = std::deque<CMetricMap*>;

}  // namespace mrpt::maps
