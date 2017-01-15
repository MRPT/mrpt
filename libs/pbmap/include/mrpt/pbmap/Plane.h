/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
#include <set>
#include <map>

#define USE_COMPLETNESS_HEURISTICS 1
#define USE_INFERRED_STRUCTURE 1

static std::vector<size_t> DEFAULT_VECTOR;

namespace mrpt {
namespace pbmap {
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
//      vector< vector<int> > vec(4, vector<int>(4));
    }

    /*!
     * Force the plane inliers to lay on the plane
     */
    void forcePtsLayOnPlane();

    /**!
     * Calculate the plane's convex hull with the monotone chain algorithm.
    */
//    void calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud );
    void calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, std::vector<size_t> &indices = DEFAULT_VECTOR );

    void calcConvexHullandParams(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, std::vector<size_t> &indices = DEFAULT_VECTOR );

    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
//      * \param polygonContourPtr the point cloud (planar)
//      * \param normal the plane normal
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

    bool isSamePlane(Eigen::Matrix4f &Rt, Plane &plane_, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

    bool hasSimilarDominantColor(Plane &plane, const float colorThreshold);

    /*! Merge the two input patches into "updatePlane".
     *  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
     */
    void mergePlane(Plane &plane);
    void mergePlane2(Plane &plane);// Adaptation for RGBD360

    void transform(Eigen::Matrix4f &Rt);


    /**!
     *  Parameters to allow the plane-based representation of the map by a graph
    */
    unsigned id;
    unsigned numObservations;
    unsigned semanticGroup;
    std::set<unsigned> nearbyPlanes;
    std::map<unsigned,unsigned> neighborPlanes;

    /*!Labels to store semantic attributes*/
    std::string label;
    std::string label_object;
    std::string label_context;

    /**!
     *  Geometric description
    */
    Eigen::Vector3f v3center;
    Eigen::Vector3f v3normal;
    float d;
    Eigen::Matrix4f information; // Fisher information matrix (the inverse of the plane covariance)
    float curvature;
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
    float dominantIntensity;
    bool bDominantColor;
    Eigen::Vector3f v3colorNrgbDev;

    Eigen::Vector3f v3colorC1C2C3; // Color paper
    std::vector<float> hist_H; // Normalized, Saturated Hue histogram (including 2 bins for black and white)

    std::vector<double> prog_area;
    std::vector<double> prog_elongation; // This is the reatio between the lengths of the plane in the two principal directions
    std::vector<Eigen::Vector3f> prog_C1C2C3;
    std::vector<Eigen::Vector3f> prog_Nrgb;
    std::vector<float> prog_intensity;
    std::vector<std::vector<float> > prog_hist_H;

    /**!
     *  Convex Hull
    */
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr;
    std::vector<int32_t> inliers;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr polygonContourPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outerPolygonPtr; // This is going to be deprecated
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloudPtr; // This is going to be deprecated

    /*!
     * Calculate plane's main color using "MeanShift" method
     */
    void calcMainColor();
    void calcMainColor2();
    void calcPlaneHistH();

   private:
    /*!
     * Calculate plane's main color in normalized rgb space
     */
    void getPlaneNrgb();
    std::vector<float> r;
    std::vector<float> g;
    std::vector<float> b;
    std::vector<float> intensity;

    // Color paper
    /*!
     * Calculate plane's main color in C1C2C3 representation
     */
    std::vector<float> c1;
    std::vector<float> c2;
    std::vector<float> c3;
    void getPlaneC1C2C3();

    /*!
     * Calculate plane's main color in HSV representation
     */
//    vector<float> H;
//    vector<float> S;
//    vector<vector<float> > HSV;

  };
	DEFINE_SERIALIZABLE_POST_CUSTOM_LINKAGE( Plane, PBMAP_IMPEXP)

} } // End of namespaces

#endif

#endif
