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

#if MRPT_HAS_PCL

#include <mrpt/pbmap/Miscellaneous.h>

#define SMALL_NUM  0.00000001 // anything that avoids division overflow
float dist3D_Segment_to_Segment2( Segment S1, Segment S2)
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

//    return norm(dP);   // return the closest distance
  return norm2(dP);   // return the closest distance
}

bool isInHull(PointT &point3D, pcl::PointCloud<PointT>::Ptr hull3D)
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

//#include <sstream>
//#include <gvars3/GStringUtil.h>
//void PruneWhiteSpace(std::string & str)
//{
//  std::vector< std::string > tokens = GVars3::ChopAndUnquoteString(str);
//  std::ostringstream os;
//  os << tokens;
//
//  str = os.str();
//
//  if(str.at( str.size() - 1 ) == ' ' ) {
//    str.erase(  str.end() - 1 );
//  }
//}

#endif
