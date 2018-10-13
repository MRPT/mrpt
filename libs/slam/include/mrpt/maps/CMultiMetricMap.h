/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
class TSetOfMetricMapInitializers;

/** This class stores any customizable set of metric maps.
 * The internal metric maps can be accessed directly by the user as smart
 *pointers with CMultiMetricMap::getMapByIndex() or via `iterator`s.
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
 *  mrpt::maps::CSimplePointsMap::Ptr ptMap =
 *mrpt::make_aligned_shared<mrpt::maps::CSimplePointsMap>();
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
 * \note [New in MRPT 1.3.0]: Plain list of maps is exposed in `maps` member.
 *Proxies named `m_pointsMaps`,`m_gridMaps`, etc.
 *  are provided for backwards-compatibility and for their utility.
 *
 * \note This class belongs to [mrpt-slam] instead of [mrpt-maps] due to the
 *dependency on map classes in mrpt-vision.
 * \sa CMetricMap  \ingroup mrpt_slam_grp
 */
class CMultiMetricMap : public mrpt::maps::CMetricMap
{
	DEFINE_SERIALIZABLE(CMultiMetricMap)
   protected:
	/** Deletes all maps and clears the internal lists of maps (with
	 * clear_unique(), so user copies remain alive) */
	void deleteAllMaps();
	/** Clear all elements of the map. */
	void internal_clear() override;
	// See base class docs
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	/** Returns true if any of the inner maps is able to compute a sensible
	 * likelihood function for this observation.
	 * \param obs The observation.
	 * \sa computeObservationLikelihood
	 */
	bool internal_canComputeObservationLikelihood(
		const mrpt::obs::CObservation* obs) const override;
	// See docs in base class
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

   public:
	/** @name Access to internal list of maps: direct list, iterators, utility
	   methods and proxies
		@{ */
	using TListMaps = std::deque<
		mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr>>;

	/** The list of MRPT metric maps in this object. Use dynamic_cast or smart
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
	mrpt::maps::CMetricMap::Ptr getMapByIndex(size_t idx) const;

	/** Returns the i'th observation of a given class (or of a descendant
	 * class), or nullptr if there is no such observation in the array.
	 *  Example:
	 * \code
	 *  CObservationImage::Ptr obs =
	 * m_SF->getObservationByClass<CObservationImage>();
	 * \endcode
	 * By default (ith=0), the first observation is returned.
	 */
	template <typename T>
	typename T::Ptr getMapByClass(const size_t& ith = 0) const
	{
		size_t foundCount = 0;
		const mrpt::rtti::TRuntimeClassId* class_ID =
			&T::GetRuntimeClassIdStatic();
		for (const auto& it : *this)
			if (it->GetRuntimeClass()->derivedFrom(class_ID))
				if (foundCount++ == ith)
					return std::dynamic_pointer_cast<T>(it.get_ptr());
		return typename T::Ptr();  // Not found: return empty smart pointer
	}

	/** Takes a const ref of a STL non-associative container of smart pointers
	 * at construction and exposes an interface
	 * mildly similar to that of another STL container containing only those
	 * elements
	 * in the original container that can be `dynamic_cast`ed to
	 * `SELECTED_CLASS_PTR` */
	template <class SELECTED_CLASS_PTR, class CONTAINER>
	struct ProxyFilterContainerByClass
	{
		using ptr_t = typename SELECTED_CLASS_PTR::element_type*;
		using const_ptr_t = const typename SELECTED_CLASS_PTR::element_type*;
		ProxyFilterContainerByClass(CONTAINER& source) : m_source(&source) {}
		ProxyFilterContainerByClass(
			ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&)
			: m_source()
		{
		}  // m_source init in parent copy ctor

		ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>& operator=(
			const ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>& o)
		{
			return *this;
		}  // Do nothing, we must keep refs to our own parent
		ProxyFilterContainerByClass(
			ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&&)
			: m_source()
		{
		}  // m_source init in parent copy ctor
		ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>& operator=(
			ProxyFilterContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&& o)
		{
			return *this;
		}  // Do nothing, we must keep refs to our own parent
		bool empty() const { return size() == 0; }
		size_t size() const
		{
			size_t cnt = 0;
			for (auto it = m_source->begin(); it != m_source->end(); ++it)
				if (dynamic_cast<const_ptr_t>(it->get())) cnt++;
			return cnt;
		}
		SELECTED_CLASS_PTR operator[](size_t index) const
		{
			size_t cnt = 0;
			for (auto it = m_source->begin(); it != m_source->end(); ++it)
				if (dynamic_cast<const_ptr_t>(it->get()))
					if (cnt++ == index)
					{
						return std::dynamic_pointer_cast<
							typename SELECTED_CLASS_PTR::element_type>(
							it->get_ptr());
					}
			throw std::out_of_range("Index is out of range");
		}
		template <typename ELEMENT>
		void push_back(const ELEMENT& element)
		{
			m_source->push_back(element);
		}

	   private:
		CONTAINER* m_source;
	};  // end ProxyFilterContainerByClass

	/** A proxy like ProxyFilterContainerByClass, but it directly appears as if
	 * it was a single smart pointer (empty if no matching object is found in
	 * the container) */
	template <class SELECTED_CLASS_PTR, class CONTAINER>
	struct ProxySelectorContainerByClass
	{
		using pointee_t = typename SELECTED_CLASS_PTR::element_type;
		using ptr_t = typename SELECTED_CLASS_PTR::element_type*;
		using const_ptr_t = const typename SELECTED_CLASS_PTR::element_type*;
		ProxySelectorContainerByClass(CONTAINER& source) : m_source(&source) {}
		ProxySelectorContainerByClass(
			ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&)
			: m_source()
		{
		}  // m_source init in parent copy ctor
		ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>& operator=(
			const ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&
				o)
		{
			return *this;
		}  // Do nothing, we must keep refs to our own parent
		ProxySelectorContainerByClass(
			ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&&)
			: m_source()
		{
		}  // m_source init in parent copy ctor
		ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>& operator=(
			ProxySelectorContainerByClass<SELECTED_CLASS_PTR, CONTAINER>&& o)
		{
			return *this;
		}  // Do nothing, we must keep refs to our own parent

		operator const SELECTED_CLASS_PTR&() const
		{
			internal_update_ref();
			return m_ret;
		}
		explicit operator bool() const
		{
			internal_update_ref();
			return m_ret ? true : false;
		}
		ptr_t get()
		{
			internal_update_ref();
			return m_ret.get();
		}
		ptr_t operator->() const
		{
			internal_update_ref();
			if (m_ret)
				return m_ret.get();
			else
				throw std::runtime_error("Tried to derefer nullptr pointer");
		}
		pointee_t& operator*() const
		{
			internal_update_ref();
			if (m_ret)
				return *m_ret.get();
			else
				throw std::runtime_error("Tried to derefer nullptr pointer");
		}

	   private:
		CONTAINER* m_source;
		mutable SELECTED_CLASS_PTR m_ret;
		void internal_update_ref() const
		{
			for (auto it = m_source->begin(); it != m_source->end(); ++it)
			{
				if (dynamic_cast<const_ptr_t>(it->get()))
				{
					m_ret = std::dynamic_pointer_cast<pointee_t>(it->get_ptr());
					return;
				}
			}
			m_ret = SELECTED_CLASS_PTR();  // Not found
		}

	};  // end ProxySelectorContainerByClass

	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<mrpt::maps::CSimplePointsMap::Ptr, TListMaps>
		m_pointsMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<mrpt::maps::COccupancyGridMap2D::Ptr, TListMaps>
		m_gridMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<mrpt::maps::COctoMap::Ptr, TListMaps>
		m_octoMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<mrpt::maps::CColouredOctoMap::Ptr, TListMaps>
		m_colourOctoMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<
		mrpt::maps::CGasConcentrationGridMap2D::Ptr, TListMaps>
		m_gasGridMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<
		mrpt::maps::CWirelessPowerGridMap2D::Ptr, TListMaps>
		m_wifiGridMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<mrpt::maps::CHeightGridMap2D::Ptr, TListMaps>
		m_heightMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<
		mrpt::maps::CHeightGridMap2D_MRF::Ptr, TListMaps>
		m_heightMRFMaps;
	/** STL-like proxy to access this kind of maps in \ref maps */
	ProxyFilterContainerByClass<
		mrpt::maps::CReflectivityGridMap2D::Ptr, TListMaps>
		m_reflectivityMaps;
	/** Proxy that looks like a smart pointer to the first matching object in
	 * \ref maps */
	ProxySelectorContainerByClass<
		mrpt::maps::CColouredPointsMap::Ptr, TListMaps>
		m_colourPointsMap;
	/** Proxy that looks like a smart pointer to the first matching object in
	 * \ref maps */
	ProxySelectorContainerByClass<
		mrpt::maps::CWeightedPointsMap::Ptr, TListMaps>
		m_weightedPointsMap;
	/** Proxy that looks like a smart pointer to the first matching object in
	 * \ref maps */
	ProxySelectorContainerByClass<mrpt::maps::CLandmarksMap::Ptr, TListMaps>
		m_landmarksMap;
	/** Proxy that looks like a smart pointer to the first matching object in
	 * \ref maps */
	ProxySelectorContainerByClass<mrpt::maps::CBeaconMap::Ptr, TListMaps>
		m_beaconMap;

	/** @} */

	/** Constructor.
	 * \param initializers One internal map will be created for each entry in
	 * this "TSetOfMetricMapInitializers" struct.
	 *  If initializers is nullptr, no internal map will be created.
	 */
	CMultiMetricMap(
		const mrpt::maps::TSetOfMetricMapInitializers* initializers = nullptr);

	CMultiMetricMap(const CMultiMetricMap& o);
	CMultiMetricMap& operator=(const CMultiMetricMap& o);

	CMultiMetricMap(CMultiMetricMap&& o);
	CMultiMetricMap& operator=(CMultiMetricMap&& o);

	/** Sets the list of internal map according to the passed list of map
	 * initializers (Current maps' content will be deleted!) */
	void setListOfMaps(
		const mrpt::maps::TSetOfMetricMapInitializers* initializers);
	/** \overload */
	void setListOfMaps(
		const mrpt::maps::TSetOfMetricMapInitializers& initializers)
	{
		this->setListOfMaps(&initializers);
	}

	/** Returns true if all maps returns true to their isEmpty() method, which
	 * is map-dependent. Read the docs of each map class */
	bool isEmpty() const override;

	// See docs in base class.
	void determineMatching2D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose2D& otherMapPose,
		mrpt::tfest::TMatchingPairList& correspondences,
		const mrpt::maps::TMatchingParams& params,
		mrpt::maps::TMatchingExtraResults& extraResults) const override;

	/** See the definition in the base class: Calls in this class become a call
	 * to every single map in this set. */
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;

	/** The implementation in this class just calls all the corresponding method
	 * of the contained metric maps */
	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	/** This method is called at the end of each "prediction-update-map
	 * insertion" cycle within
	 * "mrpt::slam::CMetricMapBuilderRBPF::processActionObservation".
	 *  This method should normally do nothing, but in some cases can be used
	 * to free auxiliary cached variables.
	 */
	void auxParticleFilterCleanUp() override;

	/** Returns a 3D object representing the map.
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** If the map is a simple point map or it's a multi-metric map that
	 * contains EXACTLY one simple point map, return it.
	 * Otherwise, return NULL
	 */
	const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() const override;
	mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() override;

	/** An auxiliary variable that can be used freely by the users (this will be
	 * copied to other maps using the copy constructor, copy operator,
	 * streaming,etc) The default value is 0.
	 */
	unsigned int m_ID{0};

};  // End of class def.

}  // namespace mrpt::maps
