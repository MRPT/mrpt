/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#include <mrpt/pbmap.h> // precomp. hdr

#if MRPT_HAS_PCL

#include <mrpt/pbmap/Plane.h>
#include <mrpt/pbmap/Miscellaneous.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>

using namespace mrpt::pbmap;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(Plane, CSerializable, pbmap)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  Plane::writeToStream(CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		// The data
		out << static_cast<uint32_t>(numObservations);//out << uint32_t(numObservations);
		out << areaVoxels;
		out << areaHull;
		out << elongation;
    out << v3normal(0) << v3normal(1) << v3normal(2);
    out << v3center(0) << v3center(1) << v3center(2);
    out << v3PpalDir(0) << v3PpalDir(1) << v3PpalDir(2);
    out << v3colorNrgb(0) << v3colorNrgb(1) << v3colorNrgb(2);
    out << v3colorNrgbDev(0) << v3colorNrgbDev(1) << v3colorNrgbDev(2);
//    out.WriteBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3normal(0),3);
//    out.WriteBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3center(0),3);
//    out.WriteBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3PpalDir(0),3);
//    out.WriteBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3colorNrgb(0),3);
//    out.WriteBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3colorNrgbDev(0),3);

    out << (uint32_t)neighborPlanes.size();
    for (std::map<unsigned,unsigned>::const_iterator it=neighborPlanes.begin(); it != neighborPlanes.end(); it++)
      out << static_cast<uint32_t>(it->first) << static_cast<uint32_t>(it->second);

    out << (uint32_t)polygonContourPtr->size();
    for (uint32_t i=0; i < polygonContourPtr->size(); i++)
      out << polygonContourPtr->points[i].x << polygonContourPtr->points[i].y << polygonContourPtr->points[i].z;

    out << bFullExtent;
    out << bFromStructure;
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  Plane::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			// The data
			uint32_t n;
			in >> n;
			numObservations = (unsigned)n;
			in >> areaVoxels;
			in >> areaHull;
			in >> elongation;
			in >> v3normal(0) >> v3normal(1) >> v3normal(2);
			in >> v3center(0) >> v3center(1) >> v3center(2);
			in >> v3PpalDir(0) >> v3PpalDir(1) >> v3PpalDir(2);
			in >> v3colorNrgb(0) >> v3colorNrgb(1) >> v3colorNrgb(2);
			in >> v3colorNrgbDev(0) >> v3colorNrgbDev(1) >> v3colorNrgbDev(2);
//			in.ReadBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3normal[0],sizeof(v3normal[0])*3);
//			in.ReadBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3center[0],sizeof(v3center[0])*3);
//			in.ReadBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3PpalDir[0],sizeof(v3PpalDir[0])*3);
//			in.ReadBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3colorNrgb[0],sizeof(v3colorNrgb[0])*3);
//			in.ReadBufferFixEndianness<Eigen::Vector3f::Scalar>(&v3colorNrgbDev[0],sizeof(v3colorNrgbDev[0])*3);

			in >> n;
			neighborPlanes.clear();
      for (uint32_t i=0; i < n; i++)
      {
        uint32_t neighbor, commonObs;
        in >> neighbor >> commonObs;
        neighborPlanes[neighbor] = commonObs;
      }

			in >> n;
			polygonContourPtr->resize(n);
      for (unsigned i=0; i < n; i++)
        in >> polygonContourPtr->points[i].x >> polygonContourPtr->points[i].y >> polygonContourPtr->points[i].z;

      in >> bFullExtent;
      in >> bFromStructure;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
  };
}

/*!
 * Force the plane inliers to lay on the plane
 */
void Plane::forcePtsLayOnPlane()
{
  // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter
  const double D = -(v3normal.dot(v3center));
  for(unsigned i = 0; i < planePointCloudPtr->size(); i++)
  {
    double dist = v3normal[0]*planePointCloudPtr->points[i].x + v3normal[1]*planePointCloudPtr->points[i].y + v3normal[2]*planePointCloudPtr->points[i].z + D;
    planePointCloudPtr->points[i].x -= v3normal[0] * dist;
    planePointCloudPtr->points[i].y -= v3normal[1] * dist;
    planePointCloudPtr->points[i].z -= v3normal[2] * dist;
  }
  // Do the same with the points defining the convex hull
  for(unsigned i = 0; i < polygonContourPtr->size(); i++)
  {
    double dist = v3normal[0]*polygonContourPtr->points[i].x + v3normal[1]*polygonContourPtr->points[i].y + v3normal[2]*polygonContourPtr->points[i].z + D;
    polygonContourPtr->points[i].x -= v3normal[0] * dist;
    polygonContourPtr->points[i].y -= v3normal[1] * dist;
    polygonContourPtr->points[i].z -= v3normal[2] * dist;
  }
}


/** \brief Compute the area of a 2D planar polygon patch
  */
float Plane::compute2DPolygonalArea ()
{
  int k0, k1, k2;

  // Find axis with largest normal component and project onto perpendicular plane
  k0 = (fabs (v3normal[0] ) > fabs (v3normal[1])) ? 0  : 1;
  k0 = (fabs (v3normal[k0]) > fabs (v3normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  // cos(theta), where theta is the angle between the polygon and the projected plane
  float ct = fabs ( v3normal[k0] );
  float AreaX2 = 0.0;
  float p_i[3], p_j[3];

  for (unsigned int i = 0; i < polygonContourPtr->points.size (); i++)
  {
    p_i[0] = polygonContourPtr->points[i].x; p_i[1] = polygonContourPtr->points[i].y; p_i[2] = polygonContourPtr->points[i].z;
    int j = (i + 1) % polygonContourPtr->points.size ();
    p_j[0] = polygonContourPtr->points[j].x; p_j[1] = polygonContourPtr->points[j].y; p_j[2] = polygonContourPtr->points[j].z;

    AreaX2 += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
  }
  AreaX2 = fabs (AreaX2) / (2 * ct);

  return AreaX2;
}

/** \brief Compute the patch's convex-hull area and mass center
  */
void Plane::computeMassCenterAndArea()
{
  int k0, k1, k2;

  // Find axis with largest normal component and project onto perpendicular plane
  k0 = (fabs (v3normal[0] ) > fabs (v3normal[1])) ? 0  : 1;
  k0 = (fabs (v3normal[k0]) > fabs (v3normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  // cos(theta), where theta is the angle between the polygon and the projected plane
  float ct = fabs ( v3normal[k0] );
  float AreaX2 = 0.0;
  Eigen::Vector3f massCenter = Eigen::Vector3f::Zero();
  float p_i[3], p_j[3];

  for (unsigned int i = 0; i < polygonContourPtr->points.size (); i++)
  {
    p_i[0] = polygonContourPtr->points[i].x; p_i[1] = polygonContourPtr->points[i].y; p_i[2] = polygonContourPtr->points[i].z;
    int j = (i + 1) % polygonContourPtr->points.size ();
    p_j[0] = polygonContourPtr->points[j].x; p_j[1] = polygonContourPtr->points[j].y; p_j[2] = polygonContourPtr->points[j].z;
    double cross_segment = p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];

    AreaX2 += cross_segment;
    massCenter[k1] += (p_i[k1] + p_j[k1]) * cross_segment;
    massCenter[k2] += (p_i[k2] + p_j[k2]) * cross_segment;
  }
  areaHull = fabs (AreaX2) / (2 * ct);

  massCenter[k1] /= (3*AreaX2);
  massCenter[k2] /= (3*AreaX2);
  massCenter[k0] = (v3normal.dot(v3center) - v3normal[k1]*massCenter[k1] - v3normal[k2]*massCenter[k2]) / v3normal[k0];

  v3center = massCenter;
}

void Plane::calcElongationAndPpalDir()
{
  pcl::PCA< PointT > pca;
  pca.setInputCloud(planePointCloudPtr);
  Eigen::VectorXf eigenVal = pca.getEigenValues();
  if( eigenVal[0] > 2 * eigenVal[1] )
  {
    elongation = sqrt(eigenVal[0] / eigenVal[1]);
    Eigen::MatrixXf eigenVect = pca.getEigenVectors();
//    v3PpalDir = makeVector(eigenVect(0,0), eigenVect(1,0), eigenVect(2,0));
    v3PpalDir[0] = eigenVect(0,0);
    v3PpalDir[1] = eigenVect(1,0);
    v3PpalDir[2] = eigenVect(2,0);
  }
}

void Plane::getPlaneNrgb()
{
  r.resize(planePointCloudPtr->size());
  g.resize(planePointCloudPtr->size());
  b.resize(planePointCloudPtr->size());

  size_t countPix=0;
  for(size_t i=0; i < planePointCloudPtr->size(); i++)
  {
    float sumRGB = (float)planePointCloudPtr->points[i].r + planePointCloudPtr->points[i].g + planePointCloudPtr->points[i].b;
    if(sumRGB != 0)
    {
      r[countPix] = planePointCloudPtr->points[i].r / sumRGB;
      g[countPix] = planePointCloudPtr->points[i].g / sumRGB;
      b[countPix] = planePointCloudPtr->points[i].b / sumRGB;
      ++countPix;
    }
  }
}


void Plane::calcMainColor()
{
  getPlaneNrgb();
  v3colorNrgb(0) = getHistogramMeanShift(r, 1.0, v3colorNrgbDev(0));
  v3colorNrgb(1) = getHistogramMeanShift(g, 1.0, v3colorNrgbDev(1));
  v3colorNrgb(2) = getHistogramMeanShift(b, 1.0, v3colorNrgbDev(2));
}

/**!
 * mPointHull serves to calculate the convex hull of a set of points in 2D, which are defined by its position (x,y)
 * and an identity id
*/
struct mPointHull {
    double x, y;
    size_t id;

    bool operator <(const mPointHull &p) const {
        return x < p.x || (x == p.x && y < p.y);
    }
};

double cross(const mPointHull &O, const mPointHull &A, const mPointHull &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

/**!
 * Calculate the plane's convex hull with the monotone chain algorithm.
*/
void Plane::calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, std::vector<size_t> &indices)
{
  // Find axis with largest normal component and project onto perpendicular plane
  int k0;//, k1, k2;
  k0 = (fabs (v3normal(0) ) > fabs(v3normal[1])) ? 0  : 1;
  k0 = (fabs (v3normal(k0) ) > fabs(v3normal(2))) ? k0 : 2;

  std::vector<mPointHull> P;//(pointCloud->size() );
  P.resize(pointCloud->size() );
  if(k0 == 0)
    for(size_t i=0; i < pointCloud->size(); i++)
    {
      P[i].x = pointCloud->points[i].y;
      P[i].y = pointCloud->points[i].z;
      P[i].id = i;
    }
  else if(k0 == 1)
    for(size_t i=0; i < pointCloud->size(); i++)
    {
      P[i].x = pointCloud->points[i].x;
      P[i].y = pointCloud->points[i].z;
      P[i].id = i;
    }
  else // (k0 == 2)
    for(size_t i=0; i < pointCloud->size(); i++)
    {
      P[i].x = pointCloud->points[i].x;
      P[i].y = pointCloud->points[i].y;
      P[i].id = i;
    }

  int n = P.size(), k = 0;
  std::vector<mPointHull> H(2*n);

  // Sort points lexicographically
  std::sort(P.begin(), P.end());

  // Build lower hull
  for (int i = 0; i < n; i++)
  {
    while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0)
      k--;
    H[k++] = P[i];
    if(k > 0)
        assert(H[k-1].id != H[k-2].id);
  }

  // Build upper hull
  for (int i = n-2, t = k+1; i >= 0; i--)
  {
    while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0)
      k--;
    H[k++] = P[i];
  }

  // Fill convexHull vector
  size_t hull_noRep = k-1; // Neglect the last_point = first_point
  H.resize(k);
  polygonContourPtr->resize(hull_noRep);
  indices.resize(hull_noRep);

  for(size_t i=0; i < hull_noRep; i++)
  {
    polygonContourPtr->points[i] = pointCloud->points[ H[i].id ];
    indices[i] = H[i].id;
  }

}


/*!Returns true when the closest distance between the patches "this" and the input "plane_nearby" is under distThreshold. This function is approximated*/
bool Plane::isPlaneNearby(Plane &plane_nearby, const float distThreshold)
{
  float distThres2 = distThreshold * distThreshold;

  // First we check distances between centroids and vertex to accelerate this check
  if( (v3center - plane_nearby.v3center).squaredNorm() < distThres2 )
    return true;

  for(unsigned i=1; i < polygonContourPtr->size(); i++)
    if( (getVector3fromPointXYZ(polygonContourPtr->points[i]) - plane_nearby.v3center).squaredNorm() < distThres2 )
      return true;

  for(unsigned j=1; j < plane_nearby.polygonContourPtr->size(); j++)
    if( (v3center - getVector3fromPointXYZ(plane_nearby.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
      return true;

  for(unsigned i=1; i < polygonContourPtr->size(); i++)
    for(unsigned j=1; j < plane_nearby.polygonContourPtr->size(); j++)
      if( (diffPoints(polygonContourPtr->points[i], plane_nearby.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
        return true;

  return false;
}

/*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
* If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
bool Plane::isSamePlane(Plane &plane_nearby, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold)
{
  // Check that both planes have similar orientation
  if( v3normal .dot (plane_nearby.v3normal) < cosAngleThreshold )
    return false;

  // Check the normal distance of the planes centers using their average normal
  float dist_normal = v3normal .dot (plane_nearby.v3center - v3center);
  if(fabs(dist_normal) > distThreshold ) // Then merge the planes
    return false;

  // Check that the distance between the planes centers is not too big
  if( !isPlaneNearby(plane_nearby, proxThreshold) )
    return false;

  return true;
}

/*! Merge the input "plane_nearby" into "this".
*  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
*/
void Plane::mergePlane(Plane &plane_nearby)
{
  // Update normal and center
  v3normal = (areaVoxels*v3normal + plane_nearby.areaVoxels*plane_nearby.v3normal);
  v3normal = v3normal / norm(v3normal);

  // Update point inliers
//  *polygonContourPtr += *plane_nearby.polygonContourPtr; // Merge polygon points
  *planePointCloudPtr += *plane_nearby.planePointCloudPtr; // Add the points of the new detection and perform a voxel grid

  // Filter the points of the patch with a voxel-grid. This points are used only for visualization
  static pcl::VoxelGrid<pcl::PointXYZRGBA> merge_grid;
  merge_grid.setLeafSize(0.05,0.05,0.05);
  pcl::PointCloud<pcl::PointXYZRGBA> mergeCloud;
  merge_grid.setInputCloud (planePointCloudPtr);
  merge_grid.filter (mergeCloud);
  planePointCloudPtr->clear();
  *planePointCloudPtr = mergeCloud;

//  if(configPbMap.use_color)
//    calcMainColor();

  *plane_nearby.polygonContourPtr += *planePointCloudPtr;
  calcConvexHull(plane_nearby.polygonContourPtr);
  computeMassCenterAndArea();

  // Move the points to fulfill the plane equation
  forcePtsLayOnPlane();

  // Update area
//  double area_recalc = planePointCloudPtr->size() * 0.0025;
//  mpPlaneInferInfo->isFullExtent(plane_nearby, area_recalc);
  areaVoxels= planePointCloudPtr->size() * 0.0025;

}


#endif
