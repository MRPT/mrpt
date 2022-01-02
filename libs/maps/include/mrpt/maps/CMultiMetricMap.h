/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::maps
{
class TSetOfMetricMapInitializers;

/** This class stores any customizable set of metric maps.
 * The internal metric maps can be accessed directly by the user as smart
 *pointers with CMultiMetricMap::mapByIndex() or via `iterator`s.
 * The utility of this container is to operate on several maps simultaneously:
 *update them by inserting observations,
 * evaluate the likelihood of one observation by fusing (multiplying) the
 *likelihoods over the different maps, etc.
 *
 *  <b>These kinds of metric maps can be kept inside</b> (list may be
 *incomplete, refer to classes derived from mrpt::maps::CMetricMap):
 *		- mrpt::maps::CSimplePointsMap: For 2D or 3D range scans, ...
 *		- mrpt::maps::COccupancyGridMap2D: 2D, <b>horizontal</b>  laser range
 *scans, at different altitudes.
 *		- mrpt::maps::COctoMap: For 3D occupancy grids of variable resolution,
 *with octrees (based on the library `octomap`).
 *		- mrpt::maps::CColouredOctoMap: The same than above, but nodes can store
 *RGB data appart from occupancy.
 *		- mrpt::maps::CLandmarksMap: For visual landmarks,etc...
 *		- mrpt::maps::CGasConcentrationGridMap2D: For gas concentration maps.
 *		- mrpt::maps::CWirelessPowerGridMap2D: For wifi power maps.
 *		- mrpt::maps::CBeaconMap: For range-only SLAM.
 *		- mrpt::maps::CHeightGridMap2D: For elevation maps of height for each
 *(x,y) location (Digital elevation model, DEM)
 *		- mrpt::maps::CHeightGridMap2D_MRF: DEMs as Markov Random Field (MRF)
 *		- mrpt::maps::CReflectivityGridMap2D: For maps of "reflectivity" for
 *each
 *(x,y) location.
 *		- mrpt::maps::CColouredPointsMap: For point map with color.
 *		- mrpt::maps::CWeightedPointsMap: For point map with weights (capable of
 *"fusing").
 *
 * See CMultiMetricMap::setListOfMaps() for the method for initializing this
 *class programmatically.
 * See also TSetOfMetricMapInitializers::loadFromConfigFile for a template of
 *".ini"-like configuration
 * file that can be used to define which maps to create and all their
 *parameters.
 * Alternatively, the list of maps is public so it can be directly
 *manipulated/accessed in CMultiMetricMap::maps
 *
 *  Configuring the list of maps: Alternatives
 * --------------------------------------------
 *
 *  **Method #1: Using map definition structures**
 * \code
 * mrpt::maps::TSetOfMetricMapInitializers map_inits;
 * {
 *   mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
 *   def.resolution = 0.05;
 *   def.insertionOpts.maxOccupancyUpdateCertainty = 0.8;
 *   def.insertionOpts.maxDistanceInsertion = 30;
 *   map_inits.push_back(def);
 * }
 * {
 *   mrpt::maps::CSimplePointsMap::TMapDefinition def;
 *   map_inits.push_back(def);
 * }
 * mrpt::maps::CMultiMetricMap theMap;
 * theMap.setListOfMaps(map_inits);
 * \endcode
 *
 *  **Method #2: Using a configuration file**
 * See TSetOfMetricMapInitializers::loadFromConfigFile() for details on expected
 *file format.
 *
 * \code
 * mrpt::config::CConfigFile cfgFile("file.cfg");
 * mrpt::maps::TSetOfMetricMapInitializers map_inits;
 * map_inits.loadFromConfigFile(cfgFile, "MapDefinition");
 *
 * mrpt::maps::CMultiMetricMap theMap;
 * theMap.setListOfMaps(map_inits);
 * \endcode
 *
 *  **Method #3: Manual manipulation**
 *
 * \code
 * mrpt::maps::CMultiMetricMap theMap;
 * {
 *  auto ptMap = mrpt::maps::CSimplePointsMap::Create();
 *  theMap.maps.push_back(ptMap);
 * }
 * \endcode
 *
 * \note [New in MRPT 1.3.0]: `likelihoodMapSelection`, which selected the map
 *to be used when
 *  computing the likelihood of an observation, has been removed. Use the
 *`enableObservationLikelihood`
 *  property of each individual map declaration.
 *
 * \note [New in MRPT 1.3.0]: `enableInsertion_{pointsMap,...}` have been also
 *removed.
 *  Use the `enableObservationInsertion` property of each map declaration.
 *
 * \note This class belongs to [mrpt-slam] instead of [mrpt-maps] due to the
 *dependency on map classes in mrpt-vision.
 * \sa CMetricMap  \ingroup mrpt_maps_grp
 */
class CMultiMetricMap : public mrpt::maps::CMetricMap
{
	DEFINE_SERIALIZABLE(CMultiMetricMap, mrpt::maps)
   public:
	/** Default ctor: empty list of maps */
	CMultiMetricMap() = default;

	/** Constructor with a list of map initializers.
	 * \param initializers One internal map will be created for each entry in
	 * this "TSetOfMetricMapInitializers" struct.
	 */
	CMultiMetricMap(const TSetOfMetricMapInitializers& initializers);

	/** Creates a deep copy */
	CMultiMetricMap(const CMultiMetricMap& o);
	/** Creates a deep copy */
	CMultiMetricMap& operator=(const CMultiMetricMap& o);

	/** Move ctor */
	CMultiMetricMap(CMultiMetricMap&&) = default;
	/** Move operator */
	CMultiMetricMap& operator=(CMultiMetricMap&&) = default;

	/** @name Access to list of maps
		@{ */
	using TListMaps = std::deque<mrpt::maps::CMetricMap::Ptr>;

	/** The list of metric maps in this object. Use dynamic_cast or smart
	 * pointer-based downcast to access maps by their actual type.
	 * You can directly manipulate this list. Helper methods to initialize it
	 * are described in the docs of CMultiMetricMap
	 */
	TListMaps maps;

	using iterator = TListMaps::iterator;
	using const_iterator = TListMaps::const_iterator;
	iterator begin() { return maps.begin(); }
	const_iterator begin() const { return maps.begin(); }
	iterator end() { return maps.end(); }
	const_iterator end() const { return maps.end(); }

	/** Gets the i-th map \exception std::runtime_error On out-of-bounds */
	mrpt::maps::CMetricMap::Ptr mapByIndex(size_t idx) const;

	/** Returns the i'th map of a given class (or of a derived
	 * class), or empty smart pointer if there is no such map.
	 *  Example:
	 * \code
	 *  COccupancyGridMap2D::Ptr obs =
	 * multimap.mapByClass<COccupancyGridMap2D>();
	 * \endcode
	 * By default (ith=0), the first match is returned.
	 */
	template <typename T>
	typename T::Ptr mapByClass(size_t ith = 0) const
	{
		size_t foundCount = 0;
		const auto* class_ID = &T::GetRuntimeClassIdStatic();
		for (const auto& m : maps)
			if (m && m->GetRuntimeClass()->derivedFrom(class_ID))
				if (foundCount++ == ith) return std::dynamic_pointer_cast<T>(m);
		return typename T::Ptr();  // Not found: return empty smart pointer
	}

	/** Count how many maps exist of the given class (or derived class) */
	template <typename T>
	typename std::size_t countMapsByClass() const
	{
		size_t foundCount = 0;
		const auto* class_ID = &T::GetRuntimeClassIdStatic();
		for (const auto& m : maps)
			if (m->GetRuntimeClass()->derivedFrom(class_ID)) foundCount++;
		return foundCount;
	}
	/** @} */

	/** Sets the list of internal map according to the passed list of map
	 * initializers (current maps will be deleted) */
	void setListOfMaps(const mrpt::maps::TSetOfMetricMapInitializers& init);

	// Implementation of virtual CMetricMap methods.
	// See docs in base class:

	/** Returns true if **all** maps returns true in their isEmpty() method */
	bool isEmpty() const override;
	void determineMatching2D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose2D& otherMapPose,
		mrpt::tfest::TMatchingPairList& correspondences,
		const mrpt::maps::TMatchingParams& params,
		mrpt::maps::TMatchingExtraResults& extraResults) const override;
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;
	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;
	void auxParticleFilterCleanUp() override;
	void getVisualizationInto(
		mrpt::opengl::CSetOfObjects& outObj) const override;
	const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;

	/** Returns a short description of the map. */
	std::string asString() const override
	{
		return mrpt::format(
			"Multi-map with %u children.",
			static_cast<unsigned int>(maps.size()));
	}

   protected:
	// See base class docs:
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation& obs,
		const std::optional<const mrpt::poses::CPose3D>& robotPose =
			std::nullopt) override;
	bool internal_canComputeObservationLikelihood(
		const mrpt::obs::CObservation& obs) const override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D& takenFrom) const override;

};	// End of class def.

}  // namespace mrpt::maps
