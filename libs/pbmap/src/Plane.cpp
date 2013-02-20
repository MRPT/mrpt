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

#include <mrpt/pbmap/Plane.h"
#include <mrpt/pbmap/Miscellaneous.h"
#if MRPT_HAS_PCL
#    include <pcl/common/time.h>
#endif

using namespace pbmap;

//IMPLEMENTS_SERIALIZABLE(Plane, CSerializable, pbmap)
//
///*---------------------------------------------------------------
//						writeToStream
// ---------------------------------------------------------------*/
//void  Plane::writeToStream(CStream &out, int *out_Version) const
//{
//	if (out_Version)
//		*out_Version = 0;
//	else
//	{
//		// The data
//		out << (uint32_t)numObservations;
//		out << areaVoxels;
//		out << areaHull;
//		out << elongation;
////    out << v3normal;
////    out << v3center;
////    out << v3PpalDir;
////    out << v3colorNrgb;
////    out << v3colorNrgbDev;
//    out.WriteBufferFixEndianness<Scalar>(&v3normal(0),3);
//    out.WriteBufferFixEndianness<Scalar>(&v3center(0),3);
//    out.WriteBufferFixEndianness<Scalar>(&v3PpalDir(0),3);
//    out.WriteBufferFixEndianness<Scalar>(&v3colorNrgb(0),3);
//    out.WriteBufferFixEndianness<Scalar>(&v3colorNrgbDev(0),3);
//
//    out << (uint32_t)neighborPlanes.size();
//    for (std::map<unsigned,unsigned>::iterator it=neighborPlanes.begin(); it != neighborPlanes.end(); it++)
//      out << it->first << it->second;
//
//    out << (uint32_t)polygonContourPtr->size();
//    for (uint32_t i=0; i < polygonContourPtr->size(); i++)
//      out << polygonContourPtr->points[i].x << polygonContourPtr->points[i].y << polygonContourPtr->points[i].z;
//
//    out << bFullExtent;
//    out << bFromStructure;
//    out << semanticGroup;
////    out << semanticLabel;
//    out << label;
//
//	}
//
//}
//
///*---------------------------------------------------------------
//						readFromStream
// ---------------------------------------------------------------*/
//void  Plane::readFromStream(CStream &in, int version)
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
//			in >> numObservations;
//			in >> areaVoxels;
//			in >> areaHull;
//			in >> elongation;
////			in >> v3normal;
////			in >> v3center;
////			in >> v3PpalDir;
////			in >> v3colorNrgb;
////			in >> v3colorNrgbDev;
//			in.ReadBuffer(v3normal[0],sizeof(v3normal[0])*3);
//			in.ReadBuffer(v3normal[0],sizeof(v3center[0])*3);
//			in.ReadBuffer(v3normal[0],sizeof(v3PpalDir[0])*3);
//			in.ReadBuffer(v3normal[0],sizeof(v3colorNrgb[0])*3);
//			in.ReadBuffer(v3normal[0],sizeof(v3colorNrgbDev[0])*3);
//
//			uint32_t n;
//			in >> n;
//			unsigned neighbor, commonObs;
//			neighborPlanes.clear();
//      for (unsigned i=0; i < n; i++)
//      {
//        in >> neighbor >> commonObs;
//        neighborPlanes[neighbor] = commonObs;
//      }
//
//			in >> n;
//			polygonContourPtr->resize(n);
//      for (unsigned i=0; i < n; i++)
//        in >> polygonContourPtr->points[i].x >> polygonContourPtr->points[i].y >> polygonContourPtr->points[i].z;
//
//      in >> bFullExtent;
//      in >> bFromStructure;
//      in >> semanticGroup;
//  //    out << semanticLabel;
//      in >> label;
//		} break;
//	default:
//		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
//  };
//}

//SE3<> calcPlanePose(Vector<3> &normal, Vector<3> &center)
//{
//    SE3<> planePose;
//    Matrix<3> m3Rot = Identity;
//    m3Rot[2] = normal;
//    m3Rot[0] = m3Rot[0] - (normal * (m3Rot[0] * normal));
//    normalize(m3Rot[0]);
//    m3Rot[1] = m3Rot[2] ^ m3Rot[0];
//    planePose.get_rotation() = m3Rot;
//    planePose.get_translation() = -(m3Rot * center);
//};

/*!
 * Force the plane inliers to lay on the plane
 */
void Plane::forcePtsLayOnPlane()
{
  // The plane equation has the form Ax + By + Cz + D = 0, where the vector N=(A,B,C) is the normal and the constant D can be calculated as D = -N*(PlanePoint) = -N*PlaneCenter
  const double D = -(v3normal .dot (v3center));
  for(unsigned i = 0; i < planePointCloudPtr->size(); i++)
  {
    double dist = v3normal[0]*planePointCloudPtr->points[i].x + v3normal[1]*planePointCloudPtr->points[i].y + v3normal[2]*planePointCloudPtr->points[i].z + D;
//  cout << "dist " << dist << endl;
    planePointCloudPtr->points[i].x -= v3normal[0] * dist;
    planePointCloudPtr->points[i].y -= v3normal[1] * dist;
    planePointCloudPtr->points[i].z -= v3normal[2] * dist;
  }
  // Do the same with the points defining the convex hull
  for(unsigned i = 0; i < polygonContourPtr->size(); i++)
  {
    double dist = v3normal[0]*polygonContourPtr->points[i].x + v3normal[1]*polygonContourPtr->points[i].y + v3normal[2]*polygonContourPtr->points[i].z + D;
//  cout << "dist " << dist << endl;
    polygonContourPtr->points[i].x -= v3normal[0] * dist;
    polygonContourPtr->points[i].y -= v3normal[1] * dist;
    polygonContourPtr->points[i].z -= v3normal[2] * dist;
  }
}


/** \brief Compute the area of a 2D planar polygon patch
  */
float Plane::compute2DPolygonalArea (/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &polygonContourPtr, Vector<3> &normal*/)
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
float Plane::computeMassCenterAndArea()
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
//  Vector<3> massCenter = Zeros;
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

  int i=0, countPix=0;
  for(int i=0; i < planePointCloudPtr->size(); i++)
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
  v3colorNrgb(0) = getHistogramMeanShift(r, (float)1.0, v3colorNrgbDev(0));
  v3colorNrgb(1) = getHistogramMeanShift(g, (float)1.0, v3colorNrgbDev(1));
  v3colorNrgb(2) = getHistogramMeanShift(b, (float)1.0, v3colorNrgbDev(2));
}

//**!
// * mPointHull serves to calculate the convex hull of a set of points in 2D, which are defined by its position (x,y)
// * and an identity id
//*/
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
};

/**!
 * Calculate the plane's convex hull with the monotone chain algorithm.
*/
void Plane::calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, vector<size_t> &indices)
{
  // Find axis with largest normal component and project onto perpendicular plane
  int k0;//, k1, k2;
  k0 = (fabs (v3normal(0) ) > fabs(v3normal[1])) ? 0  : 1;
  k0 = (fabs (v3normal(k0) ) > fabs(v3normal(2))) ? k0 : 2;

  vector<mPointHull> P;//(pointCloud->size() );
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
  vector<mPointHull> H(2*n);

  // Sort points lexicographically
  sort(P.begin(), P.end());

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

//cout << "\nHull k " << k0 << endl;
  for(size_t i=0; i < hull_noRep; i++)
  {
    polygonContourPtr->points[i] = pointCloud->points[ H[i].id ];
    indices[i] = H[i].id;
//  cout << H[i].id << " pt hull " << polygonContourPtr->points[i].x << " " << polygonContourPtr->points[i].y << " " << polygonContourPtr->points[i].z << endl;
  }

};

/**!
 * Verify that the plane's convex hull is efectively convex, and if it isn't, then recalculate its points
*/
bool Plane::verifyConvexHull()
{
  Eigen::Matrix3f m3Rot;
  m3Rot.Identity();
  m3Rot.row(2) = v3normal;
  m3Rot.row(0) = m3Rot.row(0) - (m3Rot.row(2) * (m3Rot.row(0).dot(m3Rot.row(2))));
  m3Rot.row(0).normalize();
  m3Rot.row(1) = m3Rot.row(2).cross(m3Rot.row(0));

  // Create a vector of 2D-points from the plane vertex laying on the plane
  vector<mPointHull> planePts;
  for(unsigned i=0; i < polygonContourPtr->size(); i++)
  {
    Eigen::Vector3f v3Pt(polygonContourPtr->points[i].x, polygonContourPtr->points[i].y, polygonContourPtr->points[i].z);
//  cout << "v3Pt " << v3Pt << endl;
    mPointHull v2PtPlane;
    v2PtPlane.x = m3Rot.row(0).dot(v3Pt);
    v2PtPlane.y = m3Rot.row(1).dot(v3Pt);
    v2PtPlane.id = i;
    planePts.push_back(v2PtPlane);
  }

  int n = planePts.size(), k = 0;
  vector<mPointHull> H(2*n);

  // Sort points lexicographically
  sort(planePts.begin(), planePts.end());

  // Build lower hull
  for (int i = 0; i < n; i++) {
      while (k >= 2 && cross(H[k-2], H[k-1], planePts[i]) <= 0) k--;
      H[k++] = planePts[i];
  }

  // Build upper hull
  for (int i = n-2, t = k+1; i >= 0; i--) {
      while (k >= t && cross(H[k-2], H[k-1], planePts[i]) <= 0) k--;
      H[k++] = planePts[i];
  }

  H.resize(k);

  // Check that the arrangement of the points defining the convex hull has not changed, in which case the previous convex hull was correct
  bool isBad = false;
  for (int i = 0; i < H.size(); i++) {
    if( H[i].id != i )
    {
      isBad = true;
      break;
    }
  }
  if(isBad) // If the previous convex hull was not correct, swap with the one calculated in this function
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int i = 0; i < H.size(); i++)
      newHull->points.push_back( polygonContourPtr->points[ H[i].id ] );
    newHull.swap(polygonContourPtr);
  }

  assert(isBad);

  return isBad;
}
