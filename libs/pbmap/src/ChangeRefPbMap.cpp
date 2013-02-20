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

//#include "../include/ChangeRefPbMap.h"
#include "../include/PbMapSerializer.h"
#include <pcl/io/pcd_io.h>

#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>

using namespace std;

void printHelp()
{
  cout << "./ChangeRefPbMap <Path to the folder containing MapPlanes.xml and MapPlanes.pcd> <destinyFileName>" << std::endl;
};

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    printHelp();
    return 0;
  }

//  string mapFile = argv[1];
  string destinyMapFile = argv[1];
  string bar = "/";

  // Load PbMap
  cout << "ChangeRefPbMap -> Load PbMap from " << argv[1] << " and save it with the new Ref in " << argv[2] << endl;

  string filepath = "/home/edu/Projects/PbMapLocalizer/Build", file = "/PbMap_die.xml", cloudFile = "/pointCloud_PbMap.pcd";
//      floorPlane = -1;

  cout << "Load map: " << filepath /*<< mapFile*/ << file << endl;

  PbMap mPbMap;
  PbMapSerializer serializer(mPbMap);
  if(serializer.LoadPbMap(filepath /*+ mapFile*/ + file) == PbMapSerializer::MAP_FAILED2 )
    cerr << "Error loading PbMap\n";

  cout << "previous PbMap loaded\n";

  // Get transformation matrix
  cout << "Floor plane is " << mPbMap.FloorPlane << endl;

  Plane &floorPlane = mPbMap.vPlanes[mPbMap.FloorPlane];
  Matrix<3,3,double> m3Rot = Identity;
  m3Rot[2] = floorPlane.v3normal;
  m3Rot[0] = m3Rot[0] - (m3Rot[2] * (m3Rot[0] * m3Rot[2]));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

//  SO3<double> rot(m3Rot); // Not exactly the same, probably because of the change after applying coerce()
//cout << "m3Rot\n" << m3Rot << "SO3\n" << rot;

  SE3<double> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3centerFloor = m3Rot * floorPlane.v3center;
  se3Aligner.get_translation() = -v3centerFloor;

//cout << "v3centerFloor" << v3centerFloor << endl;
//cout << "se3Aligner\n" << se3Aligner;
//cout << "se3Aligner\n" << se3Aligner.get_rotation();

  Eigen::Matrix4f aligner;
  for(unsigned k=0; k < 3; k++)
    for(unsigned h=0; h < 3; h++)
      aligner(k,h) = m3Rot[k][h];
  for(unsigned k=0; k < 3; k++)
    aligner(k,3) = se3Aligner.get_translation()[k];
  for(unsigned h=0; h < 3; h++)
    aligner(3,h) = 0.0;
  aligner(3,3) = 1.0;

//cout << "uno " << floorPlane.v3normal * floorPlane.v3normal << endl;
//cout << "uno " << m3Rot * floorPlane.v3normal << endl;
//cout << "normal " << se3Aligner.get_rotation().get_matrix()[2] << " vs " << floorPlane.v3normal << endl;
//cout << "zero " << se3Aligner * floorPlane.v3center << endl;
//cout << "zero " << m3Rot * floorPlane.v3center -v3centerFloor << endl;
  // Rotate and translate PbMap
  for(size_t i = 0; i < mPbMap.vPlanes.size(); i++)
  {
    Plane &plane = mPbMap.vPlanes[i];

    // Transform normal and ppal direction
    plane.v3normal = m3Rot * plane.v3normal;
    plane.v3PpalDir = m3Rot * plane.v3PpalDir;

    // Transform centroid
    plane.v3center = m3Rot * plane.v3center - v3centerFloor;

    // Transform convex hull points
    pcl::transformPointCloud(*plane.polygonContourPtr, *plane.polygonContourPtr, aligner);
  }

  // Save PbMap
  cout << "Save PbMap\n";
  serializer.SavePlanes( filepath /*+ mapFile*/ + "/" + destinyMapFile + ".xml" );


  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
  reader.read (filepath /*+ mapFile*/ + cloudFile, *model);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedModelPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // Rotate and translate the point cloud
  pcl::transformPointCloud(*model,*alignedModelPtr,aligner);
  //Save the global map as a PCD file
  pcl::io::savePCDFile(filepath /*+ mapFile*/ + "/" + destinyMapFile + ".pcd", *alignedModelPtr);


  cout << "\n\nPbMap reference system changed\n";

  return 0;
}
