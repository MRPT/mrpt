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

#ifndef __PlaneInferredInfo_H
#define __PlaneInferredInfo_H

#if MRPT_HAS_PCL

#include <mrpt/config.h>
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

  //  inline void setCurrentPbMap(PbMap &mPbM)
  //  {
  //    mPbMap = mPbM;
  //  };

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
    */ // Mover a plane o a
    void isFullExtent(Plane &plane, double newArea);

   private:

    PbMap &mPbMap;

  };

} } // End of namespaces

#endif

#endif
