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

#if MRPT_HAS_PCL

using namespace mrpt::pbmap;

#define SMALL_NUM  0.00000001 // anything that avoids division overflow
float mrpt::pbmap::dist3D_Segment_to_Segment2( Segment S1, Segment S2)
{
  Eigen::Vector3f   u = diffPoints(S1.P1, S1.P0);
  Eigen::Vector3f   v = diffPoints(S2.P1, S2.P0);
  Eigen::Vector3f   w = diffPoints(S1.P0, S2.P0);
  float    a = u.dot(u);        // always >= 0
  float    b = u.dot(v);
  float    c = v.dot(v);        // always >= 0
  float    d = u.dot(w);
  float    e = v.dot(w);
  float    D = a*c - b*b;       // always >= 0
  float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
  float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) { // the lines are almost parallel
      sN = 0.0;        // force using point P0 on segment S1
      sD = 1.0;        // to prevent possible division by 0.0 later
      tN = e;
      tD = c;
  }
  else {                // get the closest points on the infinite lines
      sN = (b*e - c*d);
      tN = (a*e - b*d);
      if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
          sN = 0.0;
          tN = e;
          tD = c;
      }
      else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
          sN = sD;
          tN = e + b;
          tD = c;
      }
  }

  if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
      tN = 0.0;
      // recompute sc for this edge
      if (-d < 0.0)
          sN = 0.0;
      else if (-d > a)
          sN = sD;
      else {
          sN = -d;
          sD = a;
      }
  }
  else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
      tN = tD;
      // recompute sc for this edge
      if ((-d + b) < 0.0)
          sN = 0;
      else if ((-d + b) > a)
          sN = sD;
      else {
          sN = (-d + b);
          sD = a;
      }
  }
  // finally do the division to get sc and tc
  sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  Eigen::Vector3f dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

  return dP.squaredNorm();   // return the closest distance
}

bool mrpt::pbmap::isInHull(PointT &point3D, pcl::PointCloud<PointT>::Ptr hull3D)
{
  Eigen::Vector2f normalLine; // This vector points inward the hull
  Eigen::Vector2f r;
  for(size_t i=1; i < hull3D->size(); i++)
  {
    normalLine[0] = hull3D->points[i-1].y - hull3D->points[i].y;
    normalLine[1] = hull3D->points[i].x - hull3D->points[i-1].x;
    r[0] = point3D.x - hull3D->points[i].x;
    r[1] = point3D.y - hull3D->points[i].y;
    if( (r .dot( normalLine) ) < 0)
      return false;
  }
  return true;
}

#endif
