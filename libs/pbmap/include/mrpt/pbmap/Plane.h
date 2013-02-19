/*
 *  Plane-based Map
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Copyright (c) 2012, Eduardo Fernández-Moral eduardofernandez@uma.es
 *
 *  http://code.google.com/p/PbMap******************************************************************************* /
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __PLANE_H
#define __PLANE_H

#include <mrpt/utils/CSerializable.h>
//#include <mrpt/pbmap/link_pragmas.h>
//#include <mrpt/math/CArray.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/common/pca.h>

using namespace std;

#define USE_COMPLETNESS_HEURISTICS 1
#define USE_INFERRED_STRUCTURE 1

static vector<size_t> DEFAULT_VECTOR;

namespace pbmap
{
  class Plane;

  // This must be added to any CSerializable derived class:
//  DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( Plane, )

	/** A class used to store a planar feature (Plane for short).
	 *  It is described with geometric features representing the shape and relative
	 *  location of the patch (area, normal vector, elongation, 3D-convex hull, etc.)
	 *  and radiometric features (the most representative color).
	 *
	 */
  class Plane //: public mrpt::utils::CSerializable
  {
    // This must be added to any CSerializable derived class:
//    DEFINE_SERIALIZABLE( Plane )

   public:
    Plane() :
      planePointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      polygonContourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
  //    planeRawPointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
  //    outerPolygonPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      bFullExtent(false),
      bFromStructure(false),
      semanticGroup(-1),
      elongation(1.0)
    {
    };

  //  /*!
  //   * Check if the the input plane is the same than this plane for some given angle and distance thresholds.
  //   * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.
  //   */
  //  bool isSamePlane(Plane &plane, const float &angleThreshold=5.0, const float &distThreshold=0.05);

    /*!
     * Force the plane inliers to lay on the plane
     */
    void forcePtsLayOnPlane();

    /**!
     * Calculate the plane's convex hull with the monotone chain algorithm.
    */
    void calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, vector<size_t> &indices = DEFAULT_VECTOR );

    /**!
     * Verify that the plane's convex hull is efectively convex, and if it isn't, then recalculate its points
    */
    bool verifyConvexHull();

    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param polygonContourPtr the point cloud (planar)
      * \param normal the plane normal
      */
    float compute2DPolygonalArea (/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &polygonContourPtr, Vector<3> &normal*/);

    /** \brief Compute the patch's convex-hull area and mass center
      */
    float computeMassCenterAndArea();

    /*!
     * Calculate plane's elongation and principal direction
     */
    void calcElongationAndPpalDir();

    /**!
     * Calculate the median of the RGB values
    */
    void getMedianColor();


    /**!
     *  Parameters to allow the plane-based representation of the map by a graph
    */
    unsigned id;
    unsigned numObservations;
    int semanticGroup;
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
    vector<float> r;
    vector<float> g;
    vector<float> b;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr polygonContourPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outerPolygonPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloudPtr;
    pcl::PlanarPolygon<pcl::PointXYZRGBA> polygon;
    pcl::PlanarRegion<pcl::PointXYZRGBA> planar_region;

  //  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edgeCloudPtr;
  //  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outEdgeCloudPtr;
  //  unsigned background, foreground, groundplane;

    /*!
     * Calculate plane's main color in C1C2C3 representation
     */
    void getPlaneNrgb();

    /*!
     * Calculate plane's main color using "MeanShift" method
     */
    void calcMainColor();

  };
} // End namespace pbmap

#endif
