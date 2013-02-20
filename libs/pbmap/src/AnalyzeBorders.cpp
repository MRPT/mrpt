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
#include <mrpt/pbmap.h> // precomp. hdr

#include <mrpt/pbmap/AnalyzeBorders.h>

//#include <boost/thread/thread.hpp>
//#include <boost/make_shared.hpp>
#if MRPT_HAS_PCL
#  include <pcl/io/io.h>
#  include <pcl/io/pcd_io.h>
#  include <pcl/features/integral_image_normal.h>
#  include <pcl/features/normal_3d.h>
#  include <pcl/ModelCoefficients.h>
#  include <pcl/segmentation/planar_region.h>
#  include <pcl/segmentation/organized_multi_plane_segmentation.h>
#  include <pcl/segmentation/organized_connected_component_segmentation.h>
#  include <pcl/surface/convex_hull.h>
#  include <pcl/filters/extract_indices.h>
#  include <pcl/filters/voxel_grid.h>
#  include <pcl/common/pca.h>
#  include <pcl/filters/fast_bilateral.h>
#endif

using namespace std;

bool keyPressed = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
//  if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
  if ( event.keyDown() )
    keyPressed = !keyPressed;
}


AnalyzeBorders::AnalyzeBorders(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
  : pointCloudPtr(cloud),
    polygonPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
//    testPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    cloudViewer("CheckPatchEdges")
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

void AnalyzeBorders::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  if (pointCloudPtr->empty())
  {
//    viz.removeAllPointClouds();
//    viz.addPointCloud (frameRGBD.pointCloudPtr, "cloud");
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

//  {
//  boost::mutex::scoped_lock lock (PbMapVisualizer_mutex);

  viz.removeAllShapes();
  viz.removeAllPointClouds();

  char name[1024];

  // Visualize point cloud
  if (!viz.updatePointCloud (pointCloudPtr, "cloud"))
    viz.addPointCloud (pointCloudPtr, "cloud");

  sprintf (name, "PointCloud size %zu", pointCloudPtr->size() );
  viz.addText(name, 10, 20);

  // Visualize polygon
  sprintf (name, "polygon");
  viz.addPolygon<pcl::PointXYZRGBA> (polygonPtr, 0.5 * red[0], 0.5 * grn[0], 0.5 * blu[0], name);

  sprintf (name, "normal");
  viz.addArrow (arrow2, arrow1, ared[0], agrn[0], ablu[0], false, name);

  // Visualize edge
  sprintf (name, "edge");
  viz.addLine (v1, v2, ared[1], agrn[1], ablu[1], name);

  sprintf (name, "pt_normal");
  viz.addArrow (v3, v4, ared[6], agrn[6], ablu[6], false, name);

//  sprintf (name, "test");
//  pcl::visualization::PointCloudColorHandlerCustom <PointT> color3 (testPtr, red[6], grn[6], blu[6]);
//  viz.addPointCloud (testPtr, color3, name);
//  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

  sprintf (name, "edge_in");
  pcl::visualization::PointCloudColorHandlerCustom <PointT> color (edgeCloudPtr, red[2], grn[2], blu[2]);
  viz.addPointCloud (edgeCloudPtr, color, name);
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

  sprintf (name, "edge_out");
  pcl::visualization::PointCloudColorHandlerCustom <PointT> color2 (outEdgeCloudPtr, red[7], grn[7], blu[7]);
  viz.addPointCloud (outEdgeCloudPtr, color2, name);
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

//  for(size_t i=0; i<vPlanes.size(); i++)
//  {
//    sprintf (name, "normal_%zu", i);
//    viz.addArrow (vPlanes[i].pt2, vPlanes[i].pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
//
//    if( !vPlanes[i].label.empty() )
//      viz.addText3D (vPlanes[i].label, vPlanes[i].pt1, 0.1, ared[i%10], agrn[i%10], ablu[i%10], vPlanes[i].label);
//    else
//    {
//      sprintf (name, "P%zu", i);
//      viz.addText3D (name, vPlanes[i].pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
//    }
//
////        sprintf (name, "plane_%02zu", i);
////        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color (vPlanes[i].planePointCloudPtr, red[i%10], grn[i%10], blu[i%10]);
////        viz.addPointCloud (vPlanes[i].planePointCloudPtr, color, name);// contourPtr, planePointCloudPtr, polygonContourPtr
////        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
//
//    sprintf (name, "approx_plane_%02d", int (i));
//    viz.addPolygon<pcl::PointXYZRGBA> (vPlanes[i].polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
//  }
}

void AnalyzeBorders::Run()
{
cout << "AnalyzeBorders::Run...\n";
  cloudViewer.runOnVisualizationThread (boost::bind(&AnalyzeBorders::viz_cb, this, _1), "viz_cb");
  cloudViewer.registerKeyboardCallback ( keyboardEventOccurred );

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f); // For VGA: 0.02f, 10.0f
  ne.setNormalSmoothingSize (5.0f);
  ne.setDepthDependentSmoothing (true);

  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers (192);
  mps.setAngularThreshold (0.069812); // (0.017453 * 2.0) // 3 degrees
  mps.setDistanceThreshold (0.03); //2cm

//  std::vector<pcl::PlanarRegion<PointT> > regions;
//  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
//  size_t prev_models_size = 0;
  char name[1024];

//  if (pointCloudPtr && cloud_mutex.try_lock ())
//  {
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
//    double normal_start = pcl::getTime ();
//    ne.setInputCloud (alignedCloudPtr);
//cout << "Ready to calc normals\n";

  // Filter the input point cloud before the plane extraction stage
  pcl::FastBilateralFilter<pcl::PointXYZRGBA> filter;
	//filter.setSigmaS (sigma_s);
	//filter.setSigmaR (sigma_r);
	//filter.setEarlyDivision (early_division);
	filter.setInputCloud (pointCloudPtr);
	filter.filter (*pointCloudPtr);

  ne.setInputCloud (pointCloudPtr);
//cout << "Compute normals\n";

  ne.compute (*normal_cloud);

//cout << "detectPlanesCloud4\n";

//    double normal_end = pcl::getTime ();
//    std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

  double plane_extract_start = pcl::getTime ();
  mps.setInputNormals (normal_cloud);
//    mps.setInputCloud (alignedCloudPtr);
  mps.setInputCloud (pointCloudPtr);

//cout << "detectPlanesCloud5\n";

  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;

  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  double plane_extract_end = pcl::getTime();

  #ifdef _VERBOSE
    std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
//    std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;
    cout << regions.size() << " planes detected\n";
  #endif

  // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
  // in the global reference
  for (size_t i = 0; i < regions.size (); i++)
  {
//      cout << "Analyse segmented regions\n";
//      cout << "Inliers " << inlier_indices[i].indices.size() << endl;
    Plane plane;

    Eigen::Vector3f centroid = regions[i].getCentroid ();
    plane.v3center = makeVector(centroid[0],centroid[1],centroid[2]);
    plane.v3normal[0] = model_coefficients[i].values[0]; plane.v3normal[1] = model_coefficients[i].values[1]; plane.v3normal[2] = model_coefficients[i].values[2];

    plane.pt1 = pcl::PointXYZ (plane.v3center[0], plane.v3center[1], plane.v3center[2]);
    plane.pt2 = pcl::PointXYZ (plane.v3center[0] + (0.5f * plane.v3normal[0]),
                                plane.v3center[1] + (0.5f * plane.v3normal[1]),
                                plane.v3center[2] + (0.5f * plane.v3normal[2]));

    arrow1 = pcl::PointXYZ (plane.v3center[0], plane.v3center[1], plane.v3center[2]);
    arrow2 = pcl::PointXYZ (plane.v3center[0] + (0.5f * plane.v3normal[0]),
                                plane.v3center[1] + (0.5f * plane.v3normal[1]),
                                plane.v3center[2] + (0.5f * plane.v3normal[2]));

//      if(poseKF.get_rotation().get_matrix()[2] * plane.v3normal > 0)
//        plane.v3normal *= -1;

//      cout << "Analyse segmented regions1a\n";

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (pointCloudPtr);
//      extract.setInputCloud (alignedCloudPtr);
    extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
    extract.setNegative (false);
//    cout << "Analyse segmented regions1b\n";

    extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud

//      cout << "Analyse segmented regions1_\n";

    static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
    plane_grid.setLeafSize(0.05,0.05,0.05);
    pcl::PointCloud<pcl::PointXYZRGBA> planeCloud;
    plane_grid.setInputCloud (plane.planePointCloudPtr);
    plane_grid.filter (*plane.planePointCloudPtr);
//    plane.planePointCloudPtr = &planeCloud;
//    cout << "area (voxels) region " << plane.planePointCloudPtr->size() * 0.0025 << endl;

    plane.contourPtr->points = regions[i].getContour ();
//    plane_grid.setInputCloud (plane.contourPtr);
//    plane_grid.filter (*plane.polygonContourPtr);
//    plane.contourPtr.swap(plane.polygonContourPtr);

//      pcl::approximatePolygon (regions[i], plane.polygon, 0.05, true);
//      plane.polygonContourPtr->points = plane.polygon.getContour ();
//      plane.areaHull = plane.compute2DPolygonalArea();
//    cout << "Area poly region: " << plane.areaHull << endl;
////      pcl::transformPointCloud(*plane.polygonContourPtr,*plane.contourPtr,pose);
//      plane.contourPtr.swap (plane.polygonContourPtr);
//      cout << "Analyse segmented regions2\n";

    // Create a Convex Hull representation of the projected inliers
//      plane.contourPtr->clear();
//    double chull_reconstruct_start = pcl::getTime ();
    pcl::ConvexHull<pcl::PointXYZRGBA> chull;
    chull.setComputeAreaVolume(true);
    chull.setDimension(2);
    chull.setInputCloud (plane.contourPtr);
//      chull.reconstruct (*plane.polygonContourPtr);
    std::vector< pcl::Vertices > polygons;
    chull.reconstruct (*plane.polygonContourPtr, polygons);

//    polygonPtr = plane.polygonContourPtr;
    pcl::copyPointCloud(*plane.polygonContourPtr,*polygonPtr);

//    cout << "Conv Hull pts " << plane.polygonContourPtr->points.size() << " polygons size " << polygons.size() << " vertices " << polygons[0].vertices.size() << ": " << endl;
//    for (size_t j = 0; j < polygons[0].vertices.size (); j++)
//     cout << " " << polygons[0].vertices[j];
//    cout << "XYZ_1\n";
//    for (size_t j = 0; j < plane.polygonContourPtr->points.size (); j++)
//    {
//     cout << plane.polygonContourPtr->points[j].x << " " << plane.polygonContourPtr->points[j].y << " " << plane.polygonContourPtr->points[j].z << endl;
////     cout << plane.contourPtr->points[polygons[0].vertices[j]].x << " " << plane.contourPtr->points[polygons[0].vertices[j]].y << " " << plane.contourPtr->points[polygons[0].vertices[j]].z << endl;
//    }
//    cout << endl;

//    plane.verifyConvexHull();

//    cout << plane.polygonContourPtr->points.front().x << " " << plane.polygonContourPtr->points.front().y << " " << plane.polygonContourPtr->points.front().z << " - "
//          << plane.polygonContourPtr->points.back().x << " " << plane.polygonContourPtr->points.back().y << " " << plane.polygonContourPtr->points.back().z << endl;
//    std::cout << "CHull reconstruction took " << double (pcl::getTime () - chull_reconstruct_start) << std::endl;
//      plane.contourPtr.swap (plane.polygonContourPtr);
    plane.areaHull = chull.getTotalArea();

//      plane.areaVoxels= chull.getTotalArea();
    plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

    #ifdef _VERBOSE
      cout << "Area plane region " << plane.areaVoxels<< " of Chull " << plane.areaHull << " of polygon " << plane.compute2DPolygonalArea() << endl;
    #endif

    // Search indices
//      vector<size_t> indices;
//      plane.calcConvexHull(plane.polygonContourPtr, indices);
    vector<size_t> indices(plane.polygonContourPtr->size());
    for(size_t j=0; j < plane.polygonContourPtr->size(); j++)
      for(size_t k=0; k < plane.contourPtr->size(); k++)
        if( plane.polygonContourPtr->points[j].x == plane.contourPtr->points[k].x &&
            plane.polygonContourPtr->points[j].y == plane.contourPtr->points[k].y &&
            plane.polygonContourPtr->points[j].z == plane.contourPtr->points[k].z )
          indices[j] = k; // Get the indices referred to contourPtr (getContour()

    vector<size_t> indices_x( indices.size() ), indices_y( indices.size() );
    for (unsigned int j = 0; j < indices.size(); j++)
    {
      // Get image indices
      indices[j] = boundary_indices[i].indices[ indices[j] ]; // Get indices referred to pointCloudPtr
      indices_x[j] = indices[j] % pointCloudPtr->width;
      indices_y[j] = indices[j] / pointCloudPtr->width;
    }

    unsigned threshold = pointCloudPtr->width / 20;
  cout << "isPlaneCutbyImage... pointCloudPtr->width " << pointCloudPtr->width << " pointCloudPtr->height " << pointCloudPtr->height << " threshold " << threshold << endl;
    bool cut = false;
    unsigned upperLimWidth = pointCloudPtr->width - threshold;
    unsigned upperLimHeight = pointCloudPtr->height - threshold;
    unsigned u, v;
  //cout << "threshold " << threshold << " upperLimWidth " << upperLimWidth << " upperLimHeight " << upperLimHeight << endl;
    for(unsigned j=0; j < indices.size(); j++)
    {
//      u = indices[j] % pointCloudPtr->width;
//      v = indices[j] / pointCloudPtr->width;
////    cout << "idx " << indices[j] << " u " << u << " v " << v << endl;
//      if(u < threshold || u > upperLimWidth || v < threshold || v > upperLimHeight){cout << "\n CUT RETURN TRUE " << u << " " << v << endl;
      if(indices_x[j] < threshold || indices_x[j] > upperLimWidth || indices_y[j] < threshold || indices_y[j] > upperLimHeight){cout << "\n CUT RETURN TRUE " << u << " " << v << endl;
        cut = true;
        break;}
    }
    if(cut)
      continue;

//    for(unsigned j = 100000; j < 100400; j++)
//      testPtr->points.push_back(pointCloudPtr->points[j]);

    double depth_threshold = 0.03, curvature_threshold = 10;
    // For every segment in the convex Hull
    for (unsigned int j = 0; j < indices.size(); j++)
    {
      v1 = plane.polygonContourPtr->points[j];
      v2 = plane.polygonContourPtr->points[(j + 1) % plane.polygonContourPtr->size()];

      int dist_x = indices_x[ (j + 1) % indices.size() ] - indices_x[j];
      int dist_y = indices_y[ (j + 1) % indices.size() ] - indices_y[j];
      double length = sqrt(dist_x*dist_x + dist_y*dist_y);
      double dir[2] = {dist_x/length, dist_y/length};
      double dir_out[2] = {-dir[1], dir[0]};
    cout << "dir " << dir[0] << " " << dir[1] << " out " << dir_out[0] << " " << dir_out[1] << endl;
    cout << "Border " << j << " is " << length << " pixels long\nDepths: ";

      // Analyze outer pixels along the polygon segment
      background = 0, foreground = 0, groundplane = 0, highCurvature = 0, normalNaN = 0;
      edgeCloudPtr->clear();
      outEdgeCloudPtr->clear();
      double distPixelsOut = 3;
      unsigned u = indices_x[j], v = indices_y[j];
      unsigned u_out = indices_x[j] + distPixelsOut*dir_out[0], v_out = indices_y[j] + distPixelsOut*dir_out[1];
      set<unsigned> pts_analysed;
//      double depth1 = pointCloudPtr->points
//      assert(plane.polygonContourPtr->size() == indices.size() );
//      assert(plane.polygonContourPtr->points[j].z == pointCloudPtr->points[indices[j]].z);
      double depth1 = plane.polygonContourPtr->points[j].z;
      double depth2 = plane.polygonContourPtr->points[(j + 1) % plane.polygonContourPtr->size()].z;
    cout << "j " << j << " j+1 " << (j + 1) % plane.polygonContourPtr->size() << endl;
    cout << "j " << v1 << " j+1 " << v2 << endl;
    cout << "depth1 " << depth1 << " depth2 " << depth2 << endl;
      float u_float = u;
      float v_float = v;
      float u_out_float = u_out;
      float v_out_float = v_out;
      for (unsigned int k = 0; k < length; k++,
                                              u_out_float += dir[0], v_out_float += dir[1], u_out = u_out_float, v_out = v_out_float,
                                              u_float += dir[0], v_float += dir[1], u = u_float, v = v_float)
      {
        assert(pcl_isfinite(pointCloudPtr->points[u + pointCloudPtr->width * v].z) );
//        assert( u + pointCloudPtr->width * v < pointCloudPtr->size() );
//        assert( u_out + pointCloudPtr->width * v_out < pointCloudPtr->size() );
        if( pcl_isfinite(pointCloudPtr->points[u + pointCloudPtr->width * v].z) )
          edgeCloudPtr->push_back(pointCloudPtr->points[u + pointCloudPtr->width * v]);

        if( pts_analysed.count(u_out + pointCloudPtr->width * v_out) > 0 )
          continue;
        pts_analysed.insert(u_out + pointCloudPtr->width * v_out);

        if( !pcl_isfinite(pointCloudPtr->points[u_out + pointCloudPtr->width * v_out].z) || pointCloudPtr->points[u_out + pointCloudPtr->width * v_out].z == 0 )
          continue;
        outEdgeCloudPtr->push_back(pointCloudPtr->points[u_out + pointCloudPtr->width * v_out]);

        // Check out-border pixel depth
        double dif_depth = pointCloudPtr->points[u_out + pointCloudPtr->width * v_out].z - (depth1 + (depth2 - depth1)*k/length);
      cout << dif_depth << " ";
      cout << "\n id " << u_out*pointCloudPtr->width + v_out << " depth " << pointCloudPtr->points[u_out + pointCloudPtr->width * v_out].z << " ref " << (depth1 + (depth2 - depth1)*k/length) << endl;
        if(dif_depth > depth_threshold)
          background++;
        else if(dif_depth < -depth_threshold)
          foreground++;
        else
        {
          groundplane++;

          // Check out-border pixel curvature
//          cout << "P-N " << plane.v3normal << " normal "
//            << normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal_x << " "
//            << normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal_y << " "
//            << normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal_z << " "
//            << normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[0] << " "
//            << normal_cloud->points[u_out + pointCloudPtr->width * v_out].data_c[0] << endl;
          if( pcl_isfinite(normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal_x) )
          {
            v3 = v4 = pointCloudPtr->points[u_out + pointCloudPtr->width * v_out];
            v4.x += 0.2*normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[0]; v3.x += 0.2*normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[1]; v3.x += 0.2*normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[2];
            if( plane.v3normal[0] * normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[0] +
                plane.v3normal[1] * normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[1] +
                plane.v3normal[2] * normal_cloud->points[u_out + pointCloudPtr->width * v_out].normal[2]
                < 0.9848) // 10º
              highCurvature++;
          }
          else
            normalNaN++;
        }

        char c;
        do {
          usleep(1000);
          c = getchar();
        } while (c != '-');
      }
      cout << "\nEdge length " << length << " outerEdge " << outEdgeCloudPtr->size() << " : B " << background << " F " << foreground << " P " << groundplane
            << " : HC " << highCurvature << " nNaN " << normalNaN << endl;

      char c;
      do {
        usleep(1000);
        c = getchar();
      } while (c != '.');

    }

//vPlanes.push_back(plane);
//  usleep(5000000);
//vPlanes.pop_back();

    vPlanes.push_back(plane);
  }
}

