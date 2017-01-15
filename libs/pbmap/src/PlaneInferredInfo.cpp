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

#include <mrpt/pbmap/PlaneInferredInfo.h>

#if MRPT_HAS_PCL

using namespace std;
using namespace mrpt::pbmap;

/**!
 * Check if the input plane correpond to the floor. For that we assume that:
 * a) the vertical axis of the sensor's reference frame and the gravity axis are simmilarly oriented,
 * b) the sensor is placed between 0.5 and 2 m above the floor plane
 * c) the input plane is big enough > 5m2.
 * d) ***we are not checking this now*** Most of the map planes are above the floor plane
*/

bool PlaneInferredInfo::searchTheFloor(Eigen::Matrix4f &poseSensor, Plane &plane)
{
  if(plane.areaVoxels< 2.0)
    return false;

  // The angle between the Y axis of the camera and the normal of the plane will normally be around: a) -1 for the floor b) 1 for the ceiling c) 0 for the walls
  double cosGravityDir = poseSensor.col(1).head(3).dot(plane.v3normal);
  if( cosGravityDir < -0.94 ) // cos 20º = 0.9397
  {
    double sensorHeight = ( plane.v3normal.dot( poseSensor.col(3).head(3) - plane.v3center) );

    if( sensorHeight < 0.7 || sensorHeight > 2.0 )
      return false;

    mPbMap.FloorPlane = plane.id;
    plane.label = mrpt::format("Floor%u", plane.id);
  }
  else if( cosGravityDir > 0.94 )
    plane.label = mrpt::format("Ceiling%u", plane.id);
  else if( fabs(cosGravityDir) < 0.34 )
    plane.label = mrpt::format("Wall%u", plane.id);
  else
    return false;

  // For bounding planes (e.g. floor) -> Check that the rest of planes in the map are above this one
  for(unsigned i=0; i < mPbMap.vPlanes.size(); i++)
  {
    if(plane.id == mPbMap.vPlanes[i].id)
      continue;
    if( plane.v3normal.dot(mPbMap.vPlanes[i].v3center - plane.v3center) < -0.1 )
    {
      plane.label = "";
      return false;
    }
  }

  plane.bFromStructure = true;

  return true;
}

/**!
 * Check if the input plane has points too close to the image limits. Assume squared cells downsampling
 * Check that the convex hull vertex are 'dist_threshold' away from the image limits.
*/
bool PlaneInferredInfo::isPlaneCutbyImage(vector<int> &planeIndices, unsigned &widthSampledImage, unsigned &heightSampledImage, unsigned threshold)
{
  unsigned upperLimWidth = widthSampledImage - threshold;
  unsigned upperLimHeight = heightSampledImage - threshold;
  unsigned u, v;
//cout << "threshold " << threshold << " upperLimWidth " << upperLimWidth << " upperLimHeight " << upperLimHeight << endl;
  for(unsigned i=0; i < planeIndices.size(); i++)
  {
    u = planeIndices[i] % widthSampledImage;
    v = planeIndices[i] / widthSampledImage;
//  cout << "idx " << planeIndices[i] << " u " << u << " v " << v << endl;
    if(u < threshold || u > upperLimWidth || v < threshold || v > upperLimHeight){//cout << "\nRETURN TRUE " << u << " " << v << endl;
      return true;}
  }
//cout << "\n\n\nRETURN FALSE\n";
  return false;
}

/**!
 * Check if the surrounding points of the input plane in the input frame are behind the plane
*/
bool PlaneInferredInfo::isSurroundingBackground(Plane &plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &frame, vector<int> &planeIndices, unsigned threshold)
{
//cout << "isSurroundingBackground\n";
  // Find in the frame the 2D position of the convex hull vertex by brute force // Innefficient
  vector< pair<unsigned, unsigned> > polyImg;
  for(unsigned i=0; i < plane.polygonContourPtr->size(); i++)
    for(unsigned j=0; j < planeIndices.size(); j++)
      if( frame->points[planeIndices[j] ].x == plane.polygonContourPtr->points[i].x &&
          frame->points[planeIndices[j] ].y == plane.polygonContourPtr->points[i].y &&
          frame->points[planeIndices[j] ].z == plane.polygonContourPtr->points[i].z )
          {
            polyImg.push_back(pair<unsigned, unsigned>(planeIndices[j] % frame->width, planeIndices[j] / frame->width) );
            break;
          }

//cout << "polyImg size " << polyImg.size() << endl;

  // Calc center
  unsigned left = polyImg[0].first, right = polyImg[0].first, up = polyImg[0].second, down = polyImg[0].second;
  for(unsigned i=1; i < polyImg.size(); i++)
  {
    if(polyImg[i].first < left) left = polyImg[i].first;
    else if(polyImg[i].first > right) right = polyImg[i].first;
    if(polyImg[i].second < down) down = polyImg[i].second;
    else if(polyImg[i].second > up) up = polyImg[i].second;
  }
  pair<int, int> center((left+right)/2,(up+down)/2);

//cout << "center " << center.first << " " << center.second << endl;

  vector<Eigen::Vector2i> outerPolygon;
  unsigned Threshold = threshold * frame->width / 640;
  for(unsigned i=1; i < polyImg.size(); i++)
  {
    Eigen::Vector2f dir;
    double normDir = sqrt( double( (polyImg[i].first - center.first)*(polyImg[i].first - center.first) + (polyImg[i].second - center.second)*(polyImg[i].second - center.second) ) );
    dir[0] = (polyImg[i].first - center.first) / normDir;
    dir[1] = (polyImg[i].second - center.second) / normDir;
//  cout << "polyImg[i] " << polyImg[i].first << " " << polyImg[i].second << endl;
//  cout << "dir " << dir << endl;
//  cout << "normDir " << normDir << endl;
    Eigen::Vector2i outVertex;
    outVertex[0] = dir[0]*Threshold + polyImg[i].first;
    outVertex[1] = dir[1]*Threshold + polyImg[i].second;
    if(outVertex[0] < 0) outVertex[0] = 0;
    else if(outVertex[0] >= static_cast<int>(frame->width)) outVertex[0] = frame->width-1;
    if(outVertex[1] < 0) outVertex[1] = 0;
    else if(outVertex[1] >= static_cast<int>(frame->height)) outVertex[1] = frame->height-1;
//  cout << "outVertex " << outVertex << endl;
    outerPolygon.push_back(outVertex);
  }

//cout << "Fill outer pc\n";
  // Fill outerPolygonPtr point cloud
  for(unsigned i=1; i < outerPolygon.size(); i++)
  {
    double edgeLenght = std::sqrt(static_cast<double>( (outerPolygon[i][0] - outerPolygon[i-1][0])*(outerPolygon[i][0] - outerPolygon[i-1][0]) + (outerPolygon[i][1] - outerPolygon[i-1][1])*(outerPolygon[i][1] - outerPolygon[i-1][1])) );
    Eigen::Vector2f direction;
    direction[0] = (outerPolygon[i][0] - outerPolygon[i-1][0]) / edgeLenght;
    direction[1] = (outerPolygon[i][1] - outerPolygon[i-1][1]) / edgeLenght;
//  cout << "outerPolygon[i] " << outerPolygon[i] << " outerPolygon[i-1] " << outerPolygon[i-1] << endl;
//  cout << "direction " << direction << endl;
//  cout << "edgeLenght " << edgeLenght << endl;
//    Eigen::Vector2f direction = normalize(outerPolygon[i] - outerPolygon[i-1]);
    Eigen::Vector2i outContour;
    for(unsigned j=0; j < edgeLenght; j++)
    {
      outContour[0] = outerPolygon[i-1][0] + int(direction[0] * j);
      outContour[1] = outerPolygon[i-1][1] + int(direction[1] * j);
//  cout << "outContour " << outContour << endl;
      pcl::PointXYZRGBA &outerPt = frame->points[outContour[0] + outContour[1]*frame->width];
      if(!pcl_isfinite(outerPt.x))
        continue;

      // Fill pointCloud corresponding to the outerPolygon
      plane.outerPolygonPtr->points.push_back(outerPt);
      double dist2Plane = plane.v3normal.dot(getVector3fromPointXYZ(outerPt) - plane.v3center);
      if(dist2Plane > 0.1)
        return false;
    }
  }

//  cout << "outerPolygonPtr size " << plane.outerPolygonPtr->size() << endl;

  return true;
}

/**!
 * Check if the area of input plane is stable and bounded (e.g < 1 m2) along the last keyframes that observe it
*/
void PlaneInferredInfo::isFullExtent(Plane &plane, double newArea)
{
//cout << "Plane " << plane.id << " newArea " << newArea << " limit " << 1.1*plane.areaVoxels<< endl;
  if(plane.areaVoxels> 1.0 || newArea > 1.0)
  {
    plane.bFullExtent = false;
    return;
  }

  if(newArea < 1.1*plane.areaVoxels)
    plane.nFramesAreaIsStable++;
  else
    plane.nFramesAreaIsStable = 0;

  if( plane.nFramesAreaIsStable > 2 )
  {
    plane.bFullExtent = true;
  }
}

#endif
