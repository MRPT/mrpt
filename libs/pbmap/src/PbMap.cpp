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

#include <mrpt/utils/CStream.h>

#include "../include/PbMap.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace pbmap;

//mrpt::utils::registerClass(CLASS_ID(Plane));

//IMPLEMENTS_SERIALIZABLE(PbMap, CSerializable, pbmap)

///*---------------------------------------------------------------
//	Constructor
//  ---------------------------------------------------------------*/
//PbMap::PbMap() :
//    globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
//    edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
//    outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
//    FloorPlane(-1)
//{
//};

///*---------------------------------------------------------------
//						writeToStream
// ---------------------------------------------------------------*/
//void  PbMap::writeToStream(CStream &out, int *out_Version) const
//{
//	if (out_Version)
//		*out_Version = 0;
//	else
//	{
//		uint32_t						n;
//
//		// The data
//		n = uint32_t( vPlanes.size() );
//		out << n;
//		for (uint32_t i=0; i < n; i++)
//			out << vPlanes[i];
//	}
//}
//
///*---------------------------------------------------------------
//						readFromStream
// ---------------------------------------------------------------*/
//void  PbMap::readFromStream(CStream &in, int version)
//{
//	switch(version)
//	{
//	case 0:
//		{
//			uint32_t	n;
//			uint32_t	i;
//
//			// Delete previous content:
//			vPlanes.clear();
//
//			// The data
//			// First, write the number of planes:
//			in >> n;
//			vPlanes.resize(n);
//      for (uint32_t i=0; i < n; i++)
//				in >> vPlanes[i];
//		} break;
//	default:
//		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
//	};
//}
