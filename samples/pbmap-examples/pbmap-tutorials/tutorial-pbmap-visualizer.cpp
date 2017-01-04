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
#include <mrpt/pbmap.h> // precomp. hdr

#include <mrpt/utils/CFileGZInputStream.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace mrpt::pbmap;

void printHelp()
{
    cout<<"---------------------------------------------------------------------------------------"<< endl;
    cout<<"./pbmap_visualizer <pointCloud.pcd> <planes.pbmap>" << endl;
    cout<<"       options: " << endl;
    cout<<"         -p | P: Show/hide point cloud" << endl;
    cout<<"         -l | L: Show/hide PbMap" << endl;
    cout<<"         -r | R: Switch between point cloud and graph representation" << endl;
}

bool graphRepresentation = false;
bool showPointCloud = true;
bool showPbMap = true;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
  {
    cout << "r was pressed => change between regular/graph representation" << endl;
    graphRepresentation = !graphRepresentation;
  }
  else if ( (event.getKeySym () == "p" || event.getKeySym () == "P") && event.keyDown ())
    showPointCloud = !showPointCloud;
  else if ( (event.getKeySym () == "l" || event.getKeySym () == "L") && event.keyDown ())
    showPbMap = !showPbMap;
}

class PbMapVisualizer
{
  public:
    PbMapVisualizer();

    pcl::visualization::CloudViewer cloudViewer;

    void Visualize();

    PbMap pbmap;

    void viz_cb (pcl::visualization::PCLVisualizer& viz);
};

PbMapVisualizer::PbMapVisualizer() :
  cloudViewer("PbMap")
{
}

// Color = (red[i], grn[i], blu[i])
// The color order is: red, green, blue, yellow, pink, turquoise, orange, purple, dark green, beige
unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

double ared [10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
double agrn [10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
double ablu [10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

void PbMapVisualizer::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  if (pbmap.globalMapPtr->empty())
  {
    mrpt::system::sleep(10);
    return;
  }

  // Render the data
  {
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
        sprintf (name, "sphere%u", static_cast<unsigned>(i));
        viz.addSphere (center, radius, ared[i%10], agrn[i%10], ablu[i%10], name);

        if( !pbmap.vPlanes[i].label.empty() )
            viz.addText3D (pbmap.vPlanes[i].label, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], pbmap.vPlanes[i].label);
        else
        {
          sprintf (name, "P%u", static_cast<unsigned>(i));
          viz.addText3D (name, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
        }

        // Draw edges
//        for(set<unsigned>::iterator it = pbmap.vPlanes[i].nearbyPlanes.begin(); it != pbmap.vPlanes[i].nearbyPlanes.end(); it++)
//        {
//          if(*it > pbmap.vPlanes[i].id)
//            break;
//
//          sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(*it));
//          pcl::PointXYZ center_it(2*pbmap.vPlanes[*it].v3center[0], 2*pbmap.vPlanes[*it].v3center[1], 2*pbmap.vPlanes[*it].v3center[2]);
//          viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);
//        }
        for(map<unsigned,unsigned>::iterator it = pbmap.vPlanes[i].neighborPlanes.begin(); it != pbmap.vPlanes[i].neighborPlanes.end(); it++)
        {
          if(it->first > pbmap.vPlanes[i].id)
            break;

          sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
          pcl::PointXYZ center_it(2*pbmap.vPlanes[it->first].v3center[0], 2*pbmap.vPlanes[it->first].v3center[1], 2*pbmap.vPlanes[it->first].v3center[2]);
          viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);

          sprintf (name, "edge%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
          char commonObs[8];
          sprintf (commonObs, "%u", it->second);
          pcl::PointXYZ half_edge( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
          viz.addText3D (commonObs, half_edge, 0.05, 1.0, 1.0, 1.0, name);
        }
      }
    }
    else
    { // Regular representation
      if (!viz.updatePointCloud (pbmap.globalMapPtr, "cloud"))
        viz.addPointCloud (pbmap.globalMapPtr, "cloud");

      sprintf (name, "PointCloud size %u", static_cast<unsigned>( pbmap.globalMapPtr->size() ) );
      viz.addText(name, 10, 20);

      for(size_t i=0; i<pbmap.vPlanes.size(); i++)
      {
        Plane &plane_i = pbmap.vPlanes[i];
//        sprintf (name, "normal_%u", static_cast<unsigned>(i));
        name[0] = *(mrpt::format("normal_%u", static_cast<unsigned>(i)).c_str());
        pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
        pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
        pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
                            plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
                            plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
        viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

        if( !plane_i.label.empty() )
          viz.addText3D (plane_i.label, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], plane_i.label);
        else
        {
          sprintf (name, "n%u", static_cast<unsigned>(i));
          viz.addText3D (name, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
        }

        sprintf (name, "approx_plane_%02d", int (i));
        viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
      }
    }
  }
}

void PbMapVisualizer::Visualize()
{
  cloudViewer.runOnVisualizationThread (boost::bind(&PbMapVisualizer::viz_cb, this, _1), "viz_cb");
  cloudViewer.registerKeyboardCallback ( keyboardEventOccurred );

  while (!cloudViewer.wasStopped() )
    mrpt::system::sleep(10);
}

// pbmap_visualizer reads a pbmap and its corresponding point-cloud and visualizes them
int main(int argc, char** argv)
{
  if(argc != 3)
  {
    printHelp();
    return -1;
  }

  string pointCloudFile = static_cast<string>(argv[1]);
  string PbMapFile = static_cast<string>(argv[2]);

  PbMapVisualizer mapViewer;

  // Read in the cloud data
  pcl::PCDReader reader;
  reader.read (pointCloudFile, *mapViewer.pbmap.globalMapPtr);
  cout << "Size " << mapViewer.pbmap.globalMapPtr->size() << " " << mapViewer.pbmap.globalMapPtr->empty() << endl;

  // Load Previous Map
  mrpt::utils::CFileGZInputStream serialized_pbmap;
  if (serialized_pbmap.open(PbMapFile))
  {
    serialized_pbmap >> mapViewer.pbmap;
  }
  else
    cout << "Error: cannot open " << PbMapFile << "\n";
  serialized_pbmap.close();

  // Visualize PbMap
  mapViewer.Visualize();

  return 0;
}
