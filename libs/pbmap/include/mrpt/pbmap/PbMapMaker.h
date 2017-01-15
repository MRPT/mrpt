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

#ifndef __PBMAPMAKER_H
#define __PBMAPMAKER_H

#include <mrpt/config.h>

#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mrpt/pbmap/Plane.h>
#include <mrpt/pbmap/PlaneInferredInfo.h>
#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/PbMapLocaliser.h>
#include <mrpt/pbmap/SemanticClustering.h>
#include <mrpt/pbmap/link_pragmas.h>
#include <set>

typedef pcl::PointXYZRGBA PointT;

namespace mrpt {
namespace pbmap {

  /*!frameRGBDandPose stores a dupla containing a pointCloud (built from a RGBD frame) and a pose.
   * \ingroup mrpt_pbmap_grp
   */
  struct PBMAP_IMPEXP frameRGBDandPose
  {
    pcl::PointCloud<PointT>::Ptr cloudPtr;
    Eigen::Matrix4f pose;
  };

  /*! This class construct the PbMap extracting planar segments from Range images, which pose must be also provided.
   *  The range images and their poses are communicated with the object frameQueue.
   *  PbMapMaker run its own thread, which is created at initialization.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PBMAP_IMPEXP PbMapMaker
  {
   public:

  /*!PbMapMaker's constructor sets some threshold for plane segmentation and map growing from a configuration file (or default).
     This constructor also starts PbMapMaker's own thread.*/
    PbMapMaker(const std::string &config_file);

  /*!PbMapMaker's destructor is used to save some debugging info to file.*/
    ~PbMapMaker();

  /*!Get the PbMap.*/
    PbMap getPbMap(){return mPbMap;};

  /*!Serialize the PbMap.*/
    void serializePbMap(std::string path);

  /*!frameQueue is a vector containing the frameRGBDandPose (range image + pose) to be processed.*/
    std::vector<frameRGBDandPose> frameQueue;

  /*!observedPlanes is a list containing the current observed planes.*/
    std::set<unsigned> sQueueObservedPlanes;

  /*!PCL viewer. It runs in a different thread.*/
    pcl::visualization::CloudViewer cloudViewer;

   private:

    /*!Find planar patches in the input organised point cloud "pointCloudPtr_arg", and update the PbMap with them (it update previous planes and
    initialize new ones when they are first observed), the input pose "poseInv" is used to place the current observations into a common frame of
    reference. Different thresholds are used to control the plane segmentation:
     - "distThreshold" defines the maximum distance of an inlier to the plane
     - "angleThreshold" defines the maximum angle between an inlier's normal and the plane's normal
     - "minInliersF" defines the minimum number of inliers as a fraction of the total number of points in the input cloud
     */
    void detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg, Eigen::Matrix4f &poseKF, double distThreshold, double angleThreshold, double minInliersF);

    /*!Returns true when the closest distance between the patches "plane1" and "plane2" is under distThreshold.*/
    bool arePlanesNearby(Plane &plane1, Plane &plane2, const float distThreshold);

    /*!Check for new graph connections of the input plane. These connections are stablished when the minimum distance between two patches is under
    the input threshold "proximity"*/
    void checkProximity(Plane &plane, float proximity);

    /*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
     * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
    bool areSamePlane(Plane &plane1, Plane &plane2, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

    /*! Merge the two input patches into "updatePlane".
     *  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
     */
    void mergePlanes(Plane &updatePlane, Plane &discardPlane);

    /*!File containing some paramteres and heuristic thresholds"*/
    FILE *config_Param;

    /*!Object to detect previous places.*/
    PbMapLocaliser *mpPbMapLocaliser;

    /*!Object to cluster set of planes according to their co-visibility.*/
    SemanticClustering *clusterize;

    boost::mutex mtx_pbmap_busy;

   protected:

    /*!The current PbMap.*/
    PbMap mPbMap;

    /*!List of planes observed in that last frame introduced.*/
    std::set<unsigned> observedPlanes;

    /*!Object to infer some knowledge in the map planes.*/
    PlaneInferredInfo *mpPlaneInferInfo;

    /*!PCL visualizer callback*/
    void viz_cb (pcl::visualization::PCLVisualizer& viz);

    /*!This executes the PbMapMaker's thread*/
    void run();

    /*!PbMapMaker's thread handle*/
    mrpt::system::TThreadHandle pbmaker_hd;

    /*!PbMapMaker's exit thread*/
    bool stop_pbMapMaker();

    /*!PbMapMaker's stop controller*/
    bool	m_pbmaker_must_stop;

    /*!PbMapMaker's stop var*/
    bool	m_pbmaker_finished;

    // Color paper
    void watchProperties(std::set<unsigned> &observedPlanes, Plane &observedPlane);
    void saveInfoFiles();
    // Unary
    float rejectAreaF, acceptAreaF, rejectAreaT, acceptAreaT;
    float rejectElongF, acceptElongF, rejectElongT, acceptElongT;
    float rejectC1C2C3_F, acceptC1C2C3_F, rejectC1C2C3_T, acceptC1C2C3_T;
    float rejectNrgb_F, acceptNrgb_F, rejectNrgb_T, acceptNrgb_T;
    float rejectIntensity_F, acceptIntensity_F, rejectIntensity_T, acceptIntensity_T;
    float rejectColor_F, acceptColor_F, rejectColor_T, acceptColor_T;
    float rejectHistH_F, acceptHistH_F, rejectHistH_T, acceptHistH_T;
  };

} } // End of namespaces

#endif

#endif
