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

#include "../include/PbMapVisualizer.h"
#include "../include/PbMapSerializer.h"

using namespace std;

void printHelp()
{
    std::cout<<"---------------------------------------------------------------------------------------"<< std::endl;
    std::cout<<"./PbMapVisualizer <pointCloud.pcd> <MapPlanes.xml>" << std::endl;
    std::cout<<"       options: " << std::endl;
    std::cout<<"         -p | P: Show/hide point cloud" << std::endl;
    std::cout<<"         -l | L: Show/hide PbMap" << std::endl;
    std::cout<<"         -r | R: Switch between point cloud and graph representation" << std::endl;
};

PbMapVisualizer::PbMapVisualizer() :
  globalMapPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
  cloudViewer("Map")
{
};

bool PbMapVisualizer::loadPlaneMap(string &pointCloudFile, string &pbmapFile)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read (pointCloudFile, *globalMapPtr);

  // Load Previous Map
  int floorPlane = -1;
  PbMapSerializer loader(pbmap);
//  string filepath = "MapPlanes/" + pbmapFile;

  if(loader.LoadPbMap(pbmapFile) == PbMapSerializer::MAP_FAILED2 )
    return false;

  for(unsigned j=0; j < pbmap.vPlanes.size(); j++)
  {
    pbmap.vPlanes[j].pt1 = pcl::PointXYZ (pbmap.vPlanes[j].v3center[0], pbmap.vPlanes[j].v3center[1], pbmap.vPlanes[j].v3center[2]);
    pbmap.vPlanes[j].pt2 = pcl::PointXYZ (pbmap.vPlanes[j].v3center[0] + (0.5f * pbmap.vPlanes[j].v3normal[0]),
                                          pbmap.vPlanes[j].v3center[1] + (0.5f * pbmap.vPlanes[j].v3normal[1]),
                                          pbmap.vPlanes[j].v3center[2] + (0.5f * pbmap.vPlanes[j].v3normal[2]));
  }

  return true;
}


bool graphRepresentation = false;
bool showPointCloud = true;
bool showPbMap = true;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
  {
//    std::cout << "r was pressed => change between regular/graph representation" << std::endl;
    graphRepresentation = !graphRepresentation;
  }
  else if ( (event.getKeySym () == "p" || event.getKeySym () == "P") && event.keyDown ())
    showPointCloud = !showPointCloud;
  else if ( (event.getKeySym () == "l" || event.getKeySym () == "L") && event.keyDown ())
    showPbMap = !showPbMap;
}

// Color = (red[i], grn[i], blu[i])
// The color order is: red, green, blue, yellow, pink, turquoise, orange, purple, dark green, beige
unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

double ared [10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
double agrn [10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
double ablu [10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

boost::mutex PbMapVisualizer_mutex;

void PbMapVisualizer::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  if (globalMapPtr->empty())
  {
//    viz.removeAllPointClouds();
//    viz.addPointCloud (frameRGBD.pointCloudPtr, "cloud");
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

  {
    boost::mutex::scoped_lock lock (PbMapVisualizer_mutex);


    viz.removeAllShapes();
    viz.removeAllPointClouds();

    char name[1024];

    if(graphRepresentation)
    {
//      cout << "show graphRepresentation\n";
      for(size_t i=0; i<pbmap.vPlanes.size(); i++)
      {
        pcl::PointXYZ center(2*pbmap.vPlanes[i].v3center[0], 2*pbmap.vPlanes[i].v3center[1], 2*pbmap.vPlanes[i].v3center[2]);
        double radius = 0.1 * sqrt(pbmap.vPlanes[i].areaVoxels);
//        cout << "radius " << radius << endl;
        sprintf (name, "sphere%zu", i);
        viz.addSphere (center, radius, ared[i%10], agrn[i%10], ablu[i%10], name);

        if( !pbmap.vPlanes[i].label.empty() )
            viz.addText3D (pbmap.vPlanes[i].label, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], pbmap.vPlanes[i].label);
        else
        {
          sprintf (name, "P%zu", i);
          viz.addText3D (name, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
        }

        // Draw edges
        for(map<unsigned,unsigned>::iterator it = pbmap.vPlanes[i].neighborPlanes.begin(); it != pbmap.vPlanes[i].neighborPlanes.end(); it++)
        {
          if(it->first > pbmap.vPlanes[i].id)
            break;

          sprintf (name, "commonObs%zu_%u", i, it->first);
          pcl::PointXYZ center_it(2*pbmap.vPlanes[it->first].v3center[0], 2*pbmap.vPlanes[it->first].v3center[1], 2*pbmap.vPlanes[it->first].v3center[2]);
          viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);

          sprintf (name, "edge%zu_%u", i, it->first);
          char commonObs[8];
          sprintf (commonObs, "%u", it->second);
          pcl::PointXYZ half_edge( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
          viz.addText3D (commonObs, half_edge, 0.05, 1.0, 1.0, 1.0, name);
        }

      }
    }
    else
    { // Regular representation
      if(showPointCloud)
        if (!viz.updatePointCloud (globalMapPtr, "cloud"))
          viz.addPointCloud (globalMapPtr, "cloud");

      sprintf (name, "PointCloud size %zu", globalMapPtr->size() );
      viz.addText(name, 10, 20);

      if(showPbMap)
      {
        for(size_t i=0; i<pbmap.vPlanes.size(); i++)
        {
          sprintf (name, "normal_%zu", i);
          viz.addArrow (pbmap.vPlanes[i].pt2, pbmap.vPlanes[i].pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

  //        if( pbmap.vPlanes[i].elongation > 1.25 )
  //        {
  //          sprintf (name, "ppalComp_%zu", i);
  //          pcl::PointXYZ pt3 = pcl::PointXYZ ( pbmap.vPlanes[i].v3center[0] + (0.2f * pbmap.vPlanes[i].v3PpalDir[0]),
  //                                              pbmap.vPlanes[i].v3center[1] + (0.2f * pbmap.vPlanes[i].v3PpalDir[1]),
  //                                              pbmap.vPlanes[i].v3center[2] + (0.2f * pbmap.vPlanes[i].v3PpalDir[2]));
  //          viz.addArrow (pt3, pbmap.vPlanes[i].pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
  //        }

          if( !pbmap.vPlanes[i].label.empty() )
            viz.addText3D (pbmap.vPlanes[i].label, pbmap.vPlanes[i].pt1, 0.1, ared[i%10], agrn[i%10], ablu[i%10], pbmap.vPlanes[i].label);
          else
          {
            sprintf (name, "P%zu", i);
            viz.addText3D (name, pbmap.vPlanes[i].pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
          }

  //        sprintf (name, "plane_%02zu", i);
  //        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color (pbmap.vPlanes[i].planePointCloudPtr, red[i%10], grn[i%10], blu[i%10]);
  //        viz.addPointCloud (pbmap.vPlanes[i].planePointCloudPtr, color, name);// contourPtr, planePointCloudPtr, polygonContourPtr
  //        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

          sprintf (name, "approx_plane_%02d", int (i));
          viz.addPolygon<pcl::PointXYZRGBA> (pbmap.vPlanes[i].polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
  //        for (unsigned idx = 0; idx < pbmap.vPlanes[i].polygonContourPtr->points.size (); ++idx)
  //        {
  //          sprintf (name, "approx_plane_%02d_%03d", int (i), int(idx));
  //          viz.addLine (pbmap.vPlanes[i].polygonContourPtr->points [idx],
  //                       pbmap.vPlanes[i].polygonContourPtr->points[(idx+1)%pbmap.vPlanes[i].polygonContourPtr->points.size ()],
  //                       0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
  //        }
        }
      } // End if showPbMap
    }

//  boost::this_thread::sleep (boost::posix_time::milliseconds (10));
//    new_cloud_ = false;
  }
}


void PbMapVisualizer::Run()
{
//cout << "PbMapVisualizer::run()\n";

  cloudViewer.runOnVisualizationThread (boost::bind(&PbMapVisualizer::viz_cb, this, _1), "viz_cb");
//  cloudViewer.registerKeyboardCallback (static_cast<void (*)(const pcl::visualization::KeyboardEvent&, void*)>(keyboardEventOccurred) );
  cloudViewer.registerKeyboardCallback ( keyboardEventOccurred );

  while (!cloudViewer.wasStopped() )
  {
//    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
};


int main(int argc, char** argv)
{
//  for(unsigned i=0; i<argc; i++)
//    cout << static_cast<string>(argv[i]) << endl;

  if(argc != 3)
  {
    printHelp();
    return -1;
  }

//  string pointCloudFile = "/home/edu/Projects/PbMaps/" + static_cast<string>(argv[1]) + "/MapPlanes.pcd";
//  string PbMapFile = "/home/edu/Projects/PbMaps/" + static_cast<string>(argv[1]) + "/MapPlanes.xml";
  string pointCloudFile = static_cast<string>(argv[1]);
  string PbMapFile = static_cast<string>(argv[2]);

  PbMapVisualizer mapViewer;

  if( !mapViewer.loadPlaneMap(pointCloudFile, PbMapFile) )
    cerr << "viewMapPlane cannot load the map\n";

  mapViewer.Run();

  return 0;
}
