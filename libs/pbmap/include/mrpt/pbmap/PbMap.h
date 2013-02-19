/*
 *  Plane-based Map
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Copyright (c) 2012, Eduardo Fernández-Moral
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

#ifndef __PBMAP_H
#define __PBMAP_H

#include <mrpt/utils/CSerializable.h>
//#include <mrpt/utils/CStream.h>
#include <vector>
#include <map>
#include <set>
#include <string>

#include "Plane.h"

typedef pcl::PointXYZRGBA PointT;

namespace pbmap
{

//// This must be added to any CSerializable derived class:
//DEFINE_SERIALIZABLE_PRE( PbMap )

	/** A class used to store a Plane-based Map (PbMap), as a set of planar patches (Planes).
	 *
	 */
  class PbMap //: public mrpt::utils::CSerializable
  {
//    // This must be added to any CSerializable derived class:
//    DEFINE_SERIALIZABLE( PbMap )

   public:

  /*!Constructor.*/
  //  PbMap();
    PbMap():
      globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
      edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
      FloorPlane(-1)
      {};


  /*!Vector to store the 3D-planes which are the basic characteristic of our map.*/
    std::vector<Plane> vPlanes;

  /*!Floor plane id*/
    int FloorPlane; // Modificar nombres, y clase. Crear la clase inferKnowledge

  /*!Registered point cloud from the RGB-D or Depth frames and visual odometry.*/
    pcl::PointCloud<PointT>::Ptr globalMapPtr;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edgeCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outEdgeCloudPtr;
    unsigned background, foreground, groundplane;
  };
} // End namespace pbmap

#endif
