/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/core/pimpl.h>

namespace mrpt::maps
{
/** A three-dimensional probabilistic occupancy grid, implemented as an
 * octo-tree with the "octomap" C++ library.
 *  This base class represents a 3D map where each voxel may contain an
 * "occupancy" property, RGBD data, etc. depending on the derived class.
 *
 * As with any other mrpt::maps::CMetricMap, you can obtain a 3D representation
 * of the map calling getAs3DObject() or getAsOctoMapVoxels()
 *
 * To use octomap's iterators to go through the voxels, use
 * COctoMap::getOctomap()
 *
 * The octomap library was presented in:
 *  - K. M. Wurm, A. Hornung, M. Bennewitz, C. Stachniss, and W. Burgard,
 *     <i>"OctoMap: A Probabilistic, Flexible, and Compact 3D Map Representation
 * for Robotic Systems"</i>
 *     in Proc. of the ICRA 2010 Workshop on Best Practice in 3D Perception and
 * Modeling for Mobile Manipulation, 2010. Software available at
 * http://octomap.sf.net/.
 *
 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
 * \ingroup mrpt_maps_grp
 */
template <class octree_t, class octree_node_t>
class COctoMapBase : public mrpt::maps::CMetricMap
{
   public:
	using myself_t = COctoMapBase<octree_t, octree_node_t>;

	/** Constructor, defines the resolution of the octomap (length of each voxel
	 * side) */
	COctoMapBase(double resolution);
	~COctoMapBase() override = default;
	/** Get a reference to the internal octomap object. Example:
	 * \code
	 *  mrpt::maps::COctoMap  map;
	 *  octomap::OcTree &om = map.getOctomap<octomap::OcTree>();
	 * \endcode
	 */
	template <class OCTOMAP_CLASS>
	inline OCTOMAP_CLASS& getOctomap()
	{
		return m_impl->m_octomap;
	}

	/** With this struct options are provided to the observation insertion
	 * process.
	 * \sa CObservation::insertObservationInto()
	 */
	struct TInsertionOptions : public mrpt::config::CLoadableOptions
	{
		/** Initilization of default parameters */
		TInsertionOptions(myself_t& parent);

		TInsertionOptions();  //!< Especial constructor, not attached to a real
		//! COctoMap object: used only in limited
		//! situations, since get*() methods don't work,
		//! etc.
		TInsertionOptions& operator=(const TInsertionOptions& o)
		{
			// Copy all but the m_parent pointer!
			maxrange = o.maxrange;
			pruning = o.pruning;
			const bool o_has_parent = o.m_parent.get() != nullptr;
			setOccupancyThres(
				o_has_parent ? o.getOccupancyThres() : o.occupancyThres);
			setProbHit(o_has_parent ? o.getProbHit() : o.probHit);
			setProbMiss(o_has_parent ? o.getProbMiss() : o.probMiss);
			setClampingThresMin(
				o_has_parent ? o.getClampingThresMin() : o.clampingThresMin);
			setClampingThresMax(
				o_has_parent ? o.getClampingThresMax() : o.clampingThresMax);
			return *this;
		}

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		double maxrange{
			-1.};  //!< maximum range for how long individual beams are
		//! inserted (default -1: complete beam)
		bool pruning{true};  //!< whether the tree is (losslessly) pruned after
		//! insertion (default: true)

		/// (key name in .ini files: "occupancyThres") sets the threshold for
		/// occupancy (sensor model) (Default=0.5)
		void setOccupancyThres(double prob)
		{
			if (m_parent.get()) m_parent->setOccupancyThres(prob);
		}
		/// (key name in .ini files: "probHit")sets the probablility for a "hit"
		/// (will be converted to logodds) - sensor model (Default=0.7)
		void setProbHit(double prob)
		{
			if (m_parent.get()) m_parent->setProbHit(prob);
		}
		/// (key name in .ini files: "probMiss")sets the probablility for a
		/// "miss" (will be converted to logodds) - sensor model (Default=0.4)
		void setProbMiss(double prob)
		{
			if (m_parent.get()) m_parent->setProbMiss(prob);
		}
		/// (key name in .ini files: "clampingThresMin")sets the minimum
		/// threshold for occupancy clamping (sensor model) (Default=0.1192, -2
		/// in log odds)
		void setClampingThresMin(double thresProb)
		{
			if (m_parent.get()) m_parent->setClampingThresMin(thresProb);
		}
		/// (key name in .ini files: "clampingThresMax")sets the maximum
		/// threshold for occupancy clamping (sensor model) (Default=0.971, 3.5
		/// in log odds)
		void setClampingThresMax(double thresProb)
		{
			if (m_parent.get()) m_parent->setClampingThresMax(thresProb);
		}

		/// @return threshold (probability) for occupancy - sensor model
		double getOccupancyThres() const
		{
			if (m_parent.get())
				return m_parent->getOccupancyThres();
			else
				return this->occupancyThres;
		}
		/// @return threshold (logodds) for occupancy - sensor model
		float getOccupancyThresLog() const
		{
			return m_parent->getOccupancyThresLog();
		}

		/// @return probablility for a "hit" in the sensor model (probability)
		double getProbHit() const
		{
			if (m_parent.get())
				return m_parent->getProbHit();
			else
				return this->probHit;
		}
		/// @return probablility for a "hit" in the sensor model (logodds)
		float getProbHitLog() const { return m_parent->getProbHitLog(); }
		/// @return probablility for a "miss"  in the sensor model (probability)
		double getProbMiss() const
		{
			if (m_parent.get())
				return m_parent->getProbMiss();
			else
				return this->probMiss;
		}
		/// @return probablility for a "miss"  in the sensor model (logodds)
		float getProbMissLog() const { return m_parent->getProbMissLog(); }
		/// @return minimum threshold for occupancy clamping in the sensor model
		/// (probability)
		double getClampingThresMin() const
		{
			if (m_parent.get())
				return m_parent->getClampingThresMin();
			else
				return this->clampingThresMin;
		}
		/// @return minimum threshold for occupancy clamping in the sensor model
		/// (logodds)
		float getClampingThresMinLog() const
		{
			return m_parent->getClampingThresMinLog();
		}
		/// @return maximum threshold for occupancy clamping in the sensor model
		/// (probability)
		double getClampingThresMax() const
		{
			if (m_parent.get())
				return m_parent->getClampingThresMax();
			else
				return this->clampingThresMax;
		}
		/// @return maximum threshold for occupancy clamping in the sensor model
		/// (logodds)
		float getClampingThresMaxLog() const
		{
			return m_parent->getClampingThresMaxLog();
		}

	   private:
		mrpt::ignored_copy_ptr<myself_t> m_parent;

		double occupancyThres{0.5};  // sets the threshold for occupancy (sensor
		// model) (Default=0.5)
		double probHit{
			0.7};  // sets the probablility for a "hit" (will be converted
		// to logodds) - sensor model (Default=0.7)
		double probMiss{0.4};  // sets the probablility for a "miss" (will be
		// converted to logodds) - sensor model (Default=0.4)
		double clampingThresMin{
			0.1192};  // sets the minimum threshold for occupancy
		// clamping (sensor model) (Default=0.1192, -2
		// in log odds)
		double clampingThresMax{
			0.971};  // sets the maximum threshold for occupancy
		// clamping (sensor model) (Default=0.971, 3.5
		// in log odds)
	};

	TInsertionOptions insertionOptions;  //!< The options used when inserting
	//! observations in the map

	/** Options used when evaluating "computeObservationLikelihood"
	 * \sa CObservation::computeObservationLikelihood
	 */
	struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
	{
		/** Initilization of default parameters
		 */
		TLikelihoodOptions();
		~TLikelihoodOptions() override = default;
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** Binary dump to stream */
		void writeToStream(mrpt::serialization::CArchive& out) const;
		/** Binary dump to stream */
		void readFromStream(mrpt::serialization::CArchive& in);

		uint32_t decimation{1};  //!< Speed up the likelihood computation by
		//! considering only one out of N rays (default=1)
	};

	TLikelihoodOptions likelihoodOptions;

	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	/** Options for the conversion of a mrpt::maps::COctoMap into a
	 * mrpt::opengl::COctoMapVoxels */
	struct TRenderingOptions
	{
		bool generateGridLines{
			false};  //!< Generate grid lines for all octree nodes,
		//! useful to draw the "structure" of the
		//! octree, but computationally costly (Default:
		//! false)

		bool generateOccupiedVoxels{
			true};  //!< Generate voxels for the occupied
		//! volumes  (Default=true)
		bool visibleOccupiedVoxels{
			true};  //!< Set occupied voxels visible (requires
		//! generateOccupiedVoxels=true)
		//!(Default=true)

		bool generateFreeVoxels{true};  //!< Generate voxels for the freespace
		//!(Default=true)
		bool visibleFreeVoxels{true};  //!< Set free voxels visible (requires
		//! generateFreeVoxels=true) (Default=true)

		TRenderingOptions() = default;

		/** Binary dump to stream */
		void writeToStream(mrpt::serialization::CArchive& out) const;
		/** Binary dump to stream */
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	TRenderingOptions renderingOptions;

	/** Returns a 3D object representing the map.
	 * \sa renderingOptions
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override
	{
		auto gl_obj = mrpt::opengl::COctoMapVoxels::Create();
		this->getAsOctoMapVoxels(*gl_obj);
		outObj->insert(gl_obj);
	}

	/** Builds a renderizable representation of the octomap as a
	 * mrpt::opengl::COctoMapVoxels object.
	 * \sa renderingOptions
	 */
	virtual void getAsOctoMapVoxels(
		mrpt::opengl::COctoMapVoxels& gl_obj) const = 0;

	/** Get the occupancy probability [0,1] of a point
	 * \return false if the point is not mapped, in which case the returned
	 * "prob" is undefined. */
	bool getPointOccupancy(
		const float x, const float y, const float z,
		double& prob_occupancy) const;

	/** Update the octomap with a 2D or 3D scan, given directly as a point cloud
	 * and the 3D location of the sensor (the origin of the rays) in this map's
	 * frame of reference.
	 * Insertion parameters can be found in \a insertionOptions.
	 * \sa The generic observation insertion method
	 * CMetricMap::insertObservation()
	 */
	void insertPointCloud(
		const CPointsMap& ptMap, const float sensor_x, const float sensor_y,
		const float sensor_z);

	/** Performs raycasting in 3d, similar to computeRay().
	 *
	 * A ray is cast from origin with a given direction, the first occupied
	 * cell is returned (as center coordinate). If the starting coordinate is
	 * already
	 * occupied in the tree, this coordinate will be returned as a hit.
	 *
	 * @param origin starting coordinate of ray
	 * @param direction A vector pointing in the direction of the raycast. Does
	 * not need to be normalized.
	 * @param end returns the center of the cell that was hit by the ray, if
	 * successful
	 * @param ignoreUnknownCells whether unknown cells are ignored. If false
	 * (default), the raycast aborts when an unkown cell is hit.
	 * @param maxRange Maximum range after which the raycast is aborted (<= 0:
	 * no limit, default)
	 * @return whether or not an occupied cell was hit
	 */
	bool castRay(
		const mrpt::math::TPoint3D& origin,
		const mrpt::math::TPoint3D& direction, mrpt::math::TPoint3D& end,
		bool ignoreUnknownCells = false, double maxRange = -1.0) const;

	virtual void setOccupancyThres(double prob) = 0;
	virtual void setProbHit(double prob) = 0;
	virtual void setProbMiss(double prob) = 0;
	virtual void setClampingThresMin(double thresProb) = 0;
	virtual void setClampingThresMax(double thresProb) = 0;
	virtual double getOccupancyThres() const = 0;
	virtual float getOccupancyThresLog() const = 0;
	virtual double getProbHit() const = 0;
	virtual float getProbHitLog() const = 0;
	virtual double getProbMiss() const = 0;
	virtual float getProbMissLog() const = 0;
	virtual double getClampingThresMin() const = 0;
	virtual float getClampingThresMinLog() const = 0;
	virtual double getClampingThresMax() const = 0;
	virtual float getClampingThresMaxLog() const = 0;

   protected:
	/**  Builds the list of 3D points in global coordinates for a generic
	 * observation. Used for both, insertObservation() and computeLikelihood().
	 * \param[out] point3d_sensorPt Is a pointer to a "point3D".
	 * \param[out] ptr_scan Is in fact a pointer to "octomap::Pointcloud". Not
	 * declared as such to avoid headers dependencies in user code.
	 * \return false if the observation kind is not applicable.
	 */
	template <class octomap_point3d, class octomap_pointcloud>
	bool internal_build_PointCloud_for_observation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose, octomap_point3d& sensorPt,
		octomap_pointcloud& scan) const;

	struct Impl;

	mrpt::pimpl<Impl> m_impl;

   private:
	// See docs in base class
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;

};  // End of class def.
}  // namespace mrpt::maps
