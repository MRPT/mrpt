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

#ifndef __MISCELLANEOUS_H
#define __MISCELLANEOUS_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/base.h>
#include <string>
#include <iostream>
#include <iterator>
#include <vector>
#include <pcl/point_types.h>
#include <mrpt/pbmap/link_pragmas.h>

namespace mrpt {
namespace pbmap {
	typedef pcl::PointXYZRGBA PointT;

  /*!Transform the (x,y,z) coordinates of a PCL point into a Eigen::Vector3f.*/
  template<class pointPCL>
  Eigen::Vector3f getVector3fromPointXYZ(pointPCL &pt)
  {
    return Eigen::Vector3f(pt.x,pt.y,pt.z);
  }

  template <class POINT>
  inline Eigen::Vector3f diffPoints(const POINT &P1, const POINT &P2)
  {
    Eigen::Vector3f diff;
    diff[0] = P1.x - P2.x;
    diff[1] = P1.y - P2.y;
    diff[2] = P1.z - P2.z;
    return diff;
  }

  /*!Compose a 3D-point with a pose.*/
  template<class dataType>
  Eigen::Matrix<dataType,3,1> compose(Eigen::Matrix<dataType,4,4> &pose, Eigen::Matrix<dataType,3,1> &point)
  {
    Eigen::Matrix<dataType,3,1> transformedPoint = pose.block(0,0,3,3) * point + pose.block(0,3,3,1);
    return transformedPoint;
  }

  /*!Compose two poses.*/
  template<class dataType>
  Eigen::Matrix<dataType,4,4> compose(Eigen::Matrix<dataType,4,4> &pose1, Eigen::Matrix<dataType,4,4> &pose2)
  {
    Eigen::Matrix<dataType,4,4> transformedPose;
    transformedPose.block(0,0,3,3) = pose1.block(0,0,3,3) * pose2.block(0,0,3,3);
    transformedPose.block(0,3,3,1) = pose1.block(0,3,3,1) + pose1.block(0,0,3,3)*pose2.block(0,3,3,1);
    transformedPose.row(3) << 0,0,0,1;
    return transformedPose;
  }

  /*!Get the pose's inverse.*/
  template<class dataType>
  Eigen::Matrix<dataType,4,4> inverse(Eigen::Matrix<dataType,4,4> &pose)
  {
    Eigen::Matrix<dataType,4,4> inverse;
    inverse.block(0,0,3,3) = pose.block(0,0,3,3).transpose();
    inverse.block(0,3,3,1) = inverse.block(0,0,3,3) * pose.block(0,3,3,1);
    inverse.row(3) << 0,0,0,1;
    return inverse;
  }

  struct Segment
  {
    Segment(PointT p0, PointT p1) :
      P0(p0), P1(p1)
    {};

    PointT P0, P1;
  };

  /*! Square of the distance between two segments */
  float PBMAP_IMPEXP dist3D_Segment_to_Segment2( Segment S1, Segment S2);

  /*! Check if a point lays inside a convex hull */
  bool PBMAP_IMPEXP isInHull(PointT &point3D, pcl::PointCloud<PointT>::Ptr hull3D);

  template<typename dataType>
  dataType getMode(std::vector<dataType> data, dataType range)
  {
    float normalizeConst = 255.0/range;
    std::vector<int> data2(data.size() );
    for(size_t i=0; i < data.size(); i++)
      data2[i] = (int)(data[i]*normalizeConst);

    std::map<int,int> histogram;
    for(size_t i=0; i < data2.size(); i++)
      if(histogram.count(data2[i]) == 0)
        histogram[data2[i] ] = 1;
      else
        histogram[data2[i] ]++;

    int mode = 0, count = 0;
    for(std::map<int,int>::iterator bin = histogram.begin(); bin != histogram.end(); bin++)
      if(bin->second > count)
      {
        count = bin->second;
        mode = bin->first;
      }

    return (dataType)mode/normalizeConst;
  }

  // Gets the center of a single-mode distribution, it performs variable mean shift
  template<typename dataType>
  dataType getHistogramMeanShift(std::vector<dataType> &data, double range, dataType &stdDevHist_out)
  {
  //  mrpt::utils::CTicTac clock;
  //  clock.Tic();
    size_t size = data.size();
    std::vector<dataType> dataTemp = data;

//    dataType sum = 0;
//    for(size_t i=0; i < data.size(); i++)
//      sum += data[i];
//    dataType meanShift =sum/size;
//    stdDevHist = mrpt::math::stddev(data);

//    dataType meanShift;
    double meanShift, stdDevHist;
    mrpt::math::meanAndStd(data,meanShift,stdDevHist);
    double sum = meanShift*data.size();

    //dataType step = 1;
    double shift = 1000;
    int iteration_counter = 0;
    double convergence = range * 0.001;
    while(2*dataTemp.size() > size && shift > convergence)
    {
  //std::cout << "iteration " << iteration_counter << " Std " << stdDevHist << std::endl;

      for(typename std::vector<dataType>::iterator it=dataTemp.begin(); it != dataTemp.end(); )
      {
        if(fabs(*it - meanShift) > stdDevHist)
        {
          sum -= *it;
          dataTemp.erase(it);
        }
        else
          it++;
      }
      double meanUpdated = sum / dataTemp.size();
      shift = fabs(meanUpdated - meanShift);
      meanShift = meanUpdated;
      stdDevHist = mrpt::math::stddev(dataTemp);

      iteration_counter++;
    }
  //  std::cout << "Number of iterations: " << iteration_counter << " shift " << shift
  //            << " size " << (float)dataTemp.size() / size << " in " << clock.Tac() * 1e3 << " ms." << std::endl;

  //  stdDevHist = calcStdDev(data, meanShift);

    stdDevHist_out = static_cast<float>(stdDevHist);
    return static_cast<float>(meanShift);
  }

  /**
   * Output a vector as a stream that is space separated.
   * @param os Output stream
   * @param v Vector to output
   * @return the stream output
   */
  template<class T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
  {
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    return os;
  }

} } // End of namespaces

#endif

#endif
