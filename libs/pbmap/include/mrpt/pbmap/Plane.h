/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __PBMAP_PLANE_H
#define __PBMAP_PLANE_H

#include <mrpt/config.h>

#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>
#include <mrpt/pbmap/link_pragmas.h>

#include <mrpt/utils/CSerializable.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#define USE_COMPLETNESS_HEURISTICS 1
#define USE_INFERRED_STRUCTURE 1

static std::vector<size_t> DEFAULT_VECTOR;

namespace mrpt {
namespace pbmap {
	using namespace mrpt::utils;

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( Plane, PBMAP_IMPEXP)

	/** A class used to store a planar feature (Plane for short).
	 *  It is described with geometric features representing the shape and relative
	 *  location of the patch (area, normal vector, elongation, 3D-convex hull, etc.)
	 *  and radiometric features (the most representative color).
	 *
	 * \ingroup mrpt_pbmap_grp
	 */
  class PBMAP_IMPEXP Plane : public mrpt::utils::CSerializable
  {
    // This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE( Plane )

   public:
    Plane() :
      elongation(1.0),
      bFullExtent(false),
      bFromStructure(false),
//      contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      polygonContourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      planePointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
    {
    }

    /*!
     * Force the plane inliers to lay on the plane
     */
    void forcePtsLayOnPlane();

    /**!
     * Calculate the plane's convex hull with the monotone chain algorithm.
    */
    void calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, std::vector<size_t> &indices = DEFAULT_VECTOR );

    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param polygonContourPtr the point cloud (planar)
      * \param normal the plane normal
      */
    float compute2DPolygonalArea (/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &polygonContourPtr, Vector<3> &normal*/);

    /** \brief Compute the patch's convex-hull area and mass center
      */
    void computeMassCenterAndArea();

    /*!
     * Calculate plane's elongation and principal direction
     */
    void calcElongationAndPpalDir();


    /*!Returns true when the closest distance between the patches "this" and "plane" is under distThreshold.*/
    bool isPlaneNearby(Plane &plane, const float distThreshold);

    /*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
     * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
    bool isSamePlane(Plane &plane, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

    /*! Merge the two input patches into "updatePlane".
     *  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
     */
    void mergePlane(Plane &plane);


    /**!
     *  Parameters to allow the plane-based representation of the map by a graph
    */
    unsigned id;
    unsigned numObservations;
    unsigned semanticGroup;
    std::set<unsigned> nearbyPlanes;
    std::map<unsigned,unsigned> neighborPlanes;
    std::string label;

    /**!
     *  Geometric description
    */
    Eigen::Vector3f v3center;
    Eigen::Vector3f v3normal;
    Eigen::Vector3f v3PpalDir;
    float elongation; // This is the reatio between the lengths of the plane in the two principal directions
    float areaVoxels;
    float areaHull;
    bool bFullExtent;
    bool bFromStructure;
    unsigned nFramesAreaIsStable;

    /**!
     *  Radiometric description
    */
    Eigen::Vector3f v3colorNrgb;
    Eigen::Vector3f v3colorNrgbDev;

    /**!
     *  Convex Hull
    */
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr polygonContourPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outerPolygonPtr; // This is going to be deprecated
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloudPtr; // This is going to be deprecated

    /*!
     * Calculate plane's main color using "MeanShift" method
     */
    void calcMainColor();


   private:
    /*!
     * Calculate plane's main color in C1C2C3 representation
     */
    void getPlaneNrgb();

    std::vector<float> r;
    std::vector<float> g;
    std::vector<float> b;

  };

} } // End of namespaces

#endif

#endif
