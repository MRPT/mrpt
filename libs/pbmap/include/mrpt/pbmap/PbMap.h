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

#ifndef __PBMAP_H
#define __PBMAP_H

#if MRPT_HAS_PCL

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/pbmap/link_pragmas.h>

#include <mrpt/pbmap/Plane.h>
#include <mrpt/pbmap/Miscellaneous.h>  // For typedef PointT;


namespace mrpt {
namespace pbmap {

//// This must be added to any CSerializable derived class:
//DEFINE_SERIALIZABLE_PRE( PbMap )

	/** A class used to store a Plane-based Map (PbMap), as a set of planar patches (Planes).
	 *
     * \ingroup mrpt_pbmap_grp
	 */
  class PbMap //: public mrpt::utils::CSerializable
  {
//    // This must be added to any CSerializable derived class:
//    DEFINE_SERIALIZABLE( PbMap )

   public:
  /*!Constructor.*/
  //  PbMap();
    PbMap():
      currentSemanticGroup(0),
      FloorPlane(-1),
      globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
      edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
      {};


  /*!Vector to store the 3D-planes which are the basic characteristic of our map.*/
    std::vector<Plane> vPlanes;

  /*!Current semantic group*/
    unsigned currentSemanticGroup;

  /*!Floor plane id*/
    int FloorPlane; // Modificar nombres, y clase. Crear la clase inferKnowledge

  /*!Registered point cloud from the RGB-D or Depth frames and visual odometry.*/
    pcl::PointCloud<PointT>::Ptr globalMapPtr;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edgeCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outEdgeCloudPtr;
    unsigned background, foreground, groundplane;
  };

} } // End of namespaces

#endif

#endif
