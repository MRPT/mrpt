// -*- c++ -*-
// Author: Eduardo Fernandez
// MAPIR Group. Dept. Ingenieria de Sistemas y Automatica. Universidad de Malaga
// http://mapir.isa.uma.es/

#ifndef __PBMAPVISUALIZER_H
#define __PBMAPVISUALIZER_H

#include "PbMap.h"

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

class PbMapVisualizer
{
  public:
    PbMapVisualizer();

    bool loadPlaneMap(std::string &pointCloudFile, std::string &pbmapFile);

    pcl::visualization::CloudViewer cloudViewer;

//    virtual
    void Run();

  private:

    PbMap pbmap;

//    std::vector<Plane> vPlanes;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMapPtr;

    void viz_cb (pcl::visualization::PCLVisualizer& viz);
};

#endif
