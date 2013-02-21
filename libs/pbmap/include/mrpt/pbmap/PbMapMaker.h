/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __PBMAPMAKER_H
#define __PBMAPMAKER_H

#if MRPT_HAS_PCL

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Plane.h"
#include "PlaneInferredInfo.h"
#include "PbMap.h"
#include "PbMapLocaliser.h"
#include "SemanticClustering.h"

typedef pcl::PointXYZRGBA PointT;

namespace mrpt {
namespace pbmap {

  /*!frameRGBDandPose stores a dupla containing a pointCloud (built from a RGBD frame) and a pose.*/
  struct frameRGBDandPose
  {
    pcl::PointCloud<PointT>::Ptr cloudPtr;
    Eigen::Matrix4f pose;
  };

  /*! This class construct the PbMap extracting planar segments from Range images, which pose must be also provided.
   *  The range images and their poses are communicated with the object frameQueue.
   *  PbMapMaker run its own thread, which is created at initialization.
   */
  class PbMapMaker
  {
   public:

  /*!PbMapMaker's constructor sets some threshold for plane segmentation and map growing from a configuration file (or default).
     This constructor also starts PbMapMaker's own thread.*/
  //  PbMapMaker(PbMap &mPbM);
    PbMapMaker();

  /*!PbMapMaker's destructor is used to save some debugging info to file.*/
    ~PbMapMaker();

  /*!Get the PbMap.*/
    PbMap getPbMap(){return mPbMap;};

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

    /*!Serialize PbMap"*/  // Cambiar nombre o Quitar!
    void PbMapSerialization(std::string sCommand, std::string sParams);

  //  /*!PbMapMaker command callback"*/
  //  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

    /*!File containing some paramteres and heuristic thresholds"*/
    FILE *config_Param;

    /*!Object to detect previous places.*/
    PbMapLocaliser *mpPbMapLocaliser;

    /*!Object to cluster set of planes according to their co-visibility.*/
    SemanticClustering *clusterize;

   protected:

    /*!The current PbMap.*/
    PbMap mPbMap;

    /*!List of planes observed in that last frame introduced.*/
    std::set<unsigned> observedPlanes;

    /*!Object to infer some knowledge in the map planes.*/
    PlaneInferredInfo *mpPlaneInferInfo;

  //  static bool graphRepresentation; // Swithches between regular representation and graph of planes representation
  //  static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

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
  };

} } // End of namespaces

#endif

#endif
