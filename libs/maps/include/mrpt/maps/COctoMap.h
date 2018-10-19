/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/COctoMapBase.h>

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/obs/obs_frwds.h>

namespace octomap
{
class OcTree;
}
namespace octomap
{
class OcTreeNode;
}

namespace mrpt
{
namespace maps
{
/** A three-dimensional probabilistic occupancy grid, implemented as an
 * octo-tree with the "octomap" C++ library.
 *  This version only stores occupancy information at each octree node. See the
 * base class mrpt::maps::COctoMapBase.
 *
 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
 * \ingroup mrpt_maps_grp
 */
class COctoMap : public COctoMapBase<octomap::OcTree, octomap::OcTreeNode>
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE(COctoMap)

   public:
	COctoMap(const double resolution = 0.10);  //!< Default constructor
	~COctoMap() override;  //!< Destructor

	void getAsOctoMapVoxels(
		mrpt::opengl::COctoMapVoxels& gl_obj) const override;

	MAP_DEFINITION_START(COctoMap)
	double resolution{
		0.10};  //!< The finest resolution of the octomap (default: 0.10
	//! meters)
	mrpt::maps::COctoMap::TInsertionOptions
		insertionOpts;  //!< Observations insertion options
	mrpt::maps::COctoMap::TLikelihoodOptions
		likelihoodOpts;  //!< Probabilistic observation likelihood options
	MAP_DEFINITION_END(COctoMap)

	/** Returns true if the map is empty/no observation has been inserted */
	bool isEmpty() const override { return size() == 1; }
	/** @name Direct access to octomap library methods
	@{ */

	/** Just like insertPointCloud but with a single ray. */
	void insertRay(
		const float end_x, const float end_y, const float end_z,
		const float sensor_x, const float sensor_y, const float sensor_z);
	/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied
	 * (true) or free (false), using the log-odds parameters in \a
	 * insertionOptions */
	void updateVoxel(
		const double x, const double y, const double z, bool occupied);
	/** Check whether the given point lies within the volume covered by the
	 * octomap (that is, whether it is "mapped") */
	bool isPointWithinOctoMap(
		const float x, const float y, const float z) const;
	double getResolution() const;
	unsigned int getTreeDepth() const;
	/// \return The number of nodes in the tree
	size_t size() const;
	/// \return Memory usage of the complete octree in bytes (may vary between
	/// architectures)
	size_t memoryUsage() const;
	/// \return Memory usage of the a single octree node
	size_t memoryUsageNode() const;
	/// \return Memory usage of a full grid of the same size as the OcTree in
	/// bytes (for comparison)
	size_t memoryFullGrid() const;
	double volume();
	/// Size of OcTree (all known space) in meters for x, y and z dimension
	void getMetricSize(double& x, double& y, double& z);
	/// Size of OcTree (all known space) in meters for x, y and z dimension
	void getMetricSize(double& x, double& y, double& z) const;
	/// minimum value of the bounding box of all known space in x, y, z
	void getMetricMin(double& x, double& y, double& z);
	/// minimum value of the bounding box of all known space in x, y, z
	void getMetricMin(double& x, double& y, double& z) const;
	/// maximum value of the bounding box of all known space in x, y, z
	void getMetricMax(double& x, double& y, double& z);
	/// maximum value of the bounding box of all known space in x, y, z
	void getMetricMax(double& x, double& y, double& z) const;
	/// Traverses the tree to calculate the total number of nodes
	size_t calcNumNodes() const;
	/// Traverses the tree to calculate the total number of leaf nodes
	size_t getNumLeafNodes() const;

	void setOccupancyThres(double prob) override;
	void setProbHit(double prob) override;
	void setProbMiss(double prob) override;
	void setClampingThresMin(double thresProb) override;
	void setClampingThresMax(double thresProb) override;
	double getOccupancyThres() const override;
	float getOccupancyThresLog() const override;
	double getProbHit() const override;
	float getProbHitLog() const override;
	double getProbMiss() const override;
	float getProbMissLog() const override;
	double getClampingThresMin() const override;
	float getClampingThresMinLog() const override;
	double getClampingThresMax() const override;
	float getClampingThresMaxLog() const override;
	/** @} */

   protected:
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose) override;
};  // End of class def.
}  // namespace maps
}  // namespace mrpt
