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

#ifndef __PlaneInferredInfo_H
#define __PlaneInferredInfo_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/Plane.h>

namespace mrpt {
namespace pbmap {


	/** A class used to infer some semantic meaning to the planes of a PbMap. This knowledge
	 *  is inferred through some heristics to determine if a plane correspond to the floor, or
	 *  to a wall ("structural plane") or if instead it is a contextual plane (e.g. TV, worktop).
	 *
	 * \ingroup mrpt_pbmap_grp
	 */
  class PBMAP_IMPEXP PlaneInferredInfo
  {
   public:

    // Context-Planes recognition functions:
  ///*!Floor plane id*/
  //  int FloorPlaneMap;

    // Constructor
    PlaneInferredInfo(PbMap &mPbM) :
      mPbMap(mPbM)
    {
    };

  /*!Check if the input plane fulfill some heuristics and so we can infer some knowledge, e.g. the plane correspond to the floor,
     a wall, the ceiling, in the world. It also makes use of the current sensor pose to verify some assumptions. // Modificar con un define esto ultimo
    */
    bool searchTheFloor(Eigen::Matrix4f &poseSensor, Plane &plane);

    // Functions to check if a plane has been fully seen. We use heuristics that are likely but not sure to give the correct result
    /*!Check if the input planar patch (after segmentation) is cut by the image. It checks that the inliers "planeIndices" fulfill a minimum distance "threshold" with the border
      of the image, whose size is defined by "widthSampledImage" x "heightSampledImage"
    */
    bool isPlaneCutbyImage(std::vector<int> &planeIndices, unsigned &widthSampledImage, unsigned &heightSampledImage, unsigned threshold);

    /*!Check if the input planar patch (after segmentation) is cut by the image. It checks that the nearer 3D points to the plane's convex hull are behind the plane
    */
    bool isSurroundingBackground(Plane &plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &frame, std::vector<int> &planeIndices, unsigned threshold);

    /*!Check if the input plane represents completely a 3D planar surface. It uses some heuristics to mark a plane as "Complete" when its area does not grow after
      subsequent observations.
    */
    void isFullExtent(Plane &plane, double newArea);

   private:

    PbMap &mPbMap;

  };

} } // End of namespaces

#endif

#endif
