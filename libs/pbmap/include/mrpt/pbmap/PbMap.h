/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a
 * href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#pragma once

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/serialization/CSerializable.h>

#include <mrpt/pbmap/Plane.h>
#include <mrpt/pbmap/Miscellaneous.h>  // For type PointT;

//#include <boost/thread/thread.hpp>

namespace mrpt::pbmap
{
/** A class used to store a Plane-based Map (PbMap). A PbMap consists of a set
 * of planar patches
 * described by geometric features (shape, relative position, etc.) and/or
 * radiometric features
 * (dominant color). It is organized as an annotated, undirected graph, where
 * nodes stand for planar
 * patches and edges connect neighbor planes when the distance between their
 * closest points is under
 * a threshold. This graph structure permits to find efficiently the closest
 * neighbors of a plane,
 * or to select groups of nearby planes representing part of the scene.
 *
 *\ingroup mrpt_pbmap_grp
 */
class PbMap : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(PbMap)

   public:
	/*!Constructor.*/
	PbMap();

	/*!Vector to store the 3D-planes which are the basic characteristic of our
	 * map.*/
	std::vector<Plane> vPlanes;

	/*!Label to store a semantic attribute*/
	std::string label;

	/*!Floor plane id*/
	int FloorPlane;

	/*!Registered point cloud from the RGB-D or Depth frames and visual
	 * odometry.*/
	pcl::PointCloud<PointT>::Ptr globalMapPtr;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edgeCloudPtr;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outEdgeCloudPtr;
	unsigned background, foreground, groundplane;

	/*!Save PbMap in the given filePath*/
	void savePbMap(std::string filePath);

	/*!Load a PbMap from the given filePath*/
	void loadPbMap(std::string PbMapFile);

	/*!Merge two pbmaps*/
	void MergeWith(PbMap& pbm, Eigen::Matrix4f& T);

	/*! Print PbMap content to a text file*/
	void printPbMap(std::string txtFilePbm);

	//    boost::mutex mtx_pbmap_busy;
};
}  // namespace mrpt::pbmap
#endif
