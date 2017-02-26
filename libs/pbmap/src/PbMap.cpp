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
#include "pbmap-precomp.h"  // Precompiled headers

#include <mrpt/pbmap.h>

#include <mrpt/utils/CStream.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::pbmap;


IMPLEMENTS_SERIALIZABLE(PbMap, CSerializable, mrpt::pbmap)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
PbMap::PbMap() :
    FloorPlane(-1),
    globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
    edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
}

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  PbMap::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
//cout << "Write PbMap. Version " << *out_Version << endl;
	if (out_Version){//cout << "There is version\n";
		*out_Version = 0;}
	else
	{
    // Write label
    out << label;

		// The data
		uint32_t n = uint32_t( vPlanes.size() );
		out << n;
//  cout << "Write " << n << " planes\n";
		for (uint32_t i=0; i < n; i++)
			out << vPlanes[i];
	}
//cout << "Exit Write PbMap. " << endl;
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  PbMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
    {
//      cout << "Read planes\n";

        // Read label
        in >> label;
//      cout << "PbMap label " << label << endl;

        // Delete previous content:
        vPlanes.clear();

        // The data
        // First, write the number of planes:
        uint32_t	n;
        in >> n;
        vPlanes.resize(n);
          for (uint32_t i=0; i < n; i++)
          {
//      cout << "plane\n";

            Plane pl;
            pl.id = i;
            in >> pl;
            vPlanes[i] = pl;
          }
//        cout << "Finish reading planes\n";
    } break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void PbMap::savePbMap(string filePath)
{
//  boost::mutex::scoped_lock lock (mtx_pbmap_busy);

//cout << "PbMap::savePbMap\n";
  // Serialize PbMap
  mrpt::utils::CFileGZOutputStream serialize_pbmap(filePath + "/planes.pbmap");
  serialize_pbmap << *this;
  serialize_pbmap.close();

//cout << "PbMap::save cloud\n";
  // Save reconstructed point cloud
  pcl::io::savePCDFile(filePath + "/cloud.pcd", *this->globalMapPtr);
}

void PbMap::loadPbMap(std::string filePath)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  string PbMapFile = filePath;
  reader.read (PbMapFile.append("/cloud.pcd"), *(this->globalMapPtr));
//  cout << "Size " << globalMapPtr->size() << " " << globalMapPtr->empty() << endl;

  // Load Previous Map
  PbMapFile = filePath;
  mrpt::utils::CFileGZInputStream serialized_pbmap;
  if (serialized_pbmap.open(PbMapFile.append("/planes.pbmap")))
  {
    serialized_pbmap >> *this;
  }
  else
    cout << "Error: cannot open " << PbMapFile << "\n";
  serialized_pbmap.close();

//  std::cout << "Load PbMap from " << filePath << "\n";
}


// Merge two pbmaps.
void PbMap::MergeWith(PbMap &pbm, Eigen::Matrix4f &T)
{
  // Rotate and translate PbMap
  for(size_t i = 0; i < pbm.vPlanes.size(); i++)
  {
    Plane plane = pbm.vPlanes[i];
//    Plane plane = &pbm.vPlanes[i]; //Warning: It modifies the source!!!

    // Transform normal and ppal direction
    plane.v3normal = T.block(0,0,3,3) * plane.v3normal;
    plane.v3PpalDir = T.block(0,0,3,3) * plane.v3PpalDir;

    // Transform centroid
    plane.v3center = T.block(0,0,3,3) * plane.v3center + T.block(0,3,3,1);

    // Transform convex hull points
    pcl::transformPointCloud(*plane.polygonContourPtr, *plane.polygonContourPtr, T);

    pcl::transformPointCloud(*plane.planePointCloudPtr, *plane.planePointCloudPtr, T);

    plane.id = vPlanes.size();

    vPlanes.push_back(plane);
  }

  // Rotate and translate the point cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*pbm.globalMapPtr,*alignedPointCloud,T);

  *globalMapPtr += *alignedPointCloud;

}

#include <fstream>
// Print PbMap content to a text file
void PbMap::printPbMap(string txtFilePbm)
{
cout << "PbMap 0.2\n\n";

  ofstream pbm;
  pbm.open(txtFilePbm.c_str());
  pbm << "PbMap 0.2\n\n";
  pbm << "MapPlanes " << vPlanes.size() << endl;
  for(unsigned i=0; i < vPlanes.size(); i++)
  {
    pbm << " ID " << vPlanes[i].id << " obs " << vPlanes[i].numObservations;
    pbm << " areaVoxels " << vPlanes[i].areaVoxels << " areaHull " << vPlanes[i].areaHull;
    pbm << " ratioXY " << vPlanes[i].elongation << " structure " << vPlanes[i].bFromStructure << " label " << vPlanes[i].label;
    pbm << "\n normal\n" << vPlanes[i].v3normal << "\n center\n" << vPlanes[i].v3center;
    pbm << "\n PpalComp\n" << vPlanes[i].v3PpalDir << "\n RGB\n" << vPlanes[i].v3colorNrgb;
    pbm << "\n Neighbors (" << vPlanes[i].neighborPlanes.size() << "): ";
    for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
      pbm << it->first << " ";
    pbm << "\n CommonObservations: ";
    for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
      pbm << it->second << " ";
    pbm << "\n ConvexHull (" << vPlanes[i].polygonContourPtr->size() << "): \n";
    for(unsigned j=0; j < vPlanes[i].polygonContourPtr->size(); j++)
      pbm << "\t" << vPlanes[i].polygonContourPtr->points[j].x << " " << vPlanes[i].polygonContourPtr->points[j].y << " " << vPlanes[i].polygonContourPtr->points[j].z << endl;
    pbm << endl;
  }
  pbm.close();
}
