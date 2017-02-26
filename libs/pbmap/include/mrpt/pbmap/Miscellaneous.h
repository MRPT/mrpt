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

#ifndef __MISCELLANEOUS_H
#define __MISCELLANEOUS_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/types_math.h> // Eigen
#include <map>
#include <string>
#include <iostream>
#include <iterator>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mrpt/pbmap/link_pragmas.h>
#include <mrpt/math.h>

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
    inverse.block(0,3,3,1) = -(inverse.block(0,0,3,3) * pose.block(0,3,3,1));
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

//  Eigen::Matrix4f& getMoorePenroseInverse(Eigen::Matrix4f &input)
//  {
////    Eigen::Matrix4f generalizedInverse;
////    Eigen::JacobiSVD<Eigen::Matrix3f> svd(input);
////    stdDevHist = svd.singularValues().maxCoeff() / sqrt(size);
//   void pinv( MatrixType& pinvmat) const
//   {
////     eigen_assert(m_isInitialized && "SVD is not initialized.");
//     double pinvtoler=1.e-6; // choose your tolerance wisely!
//     Eigen::SingularValuesType singularValues_inv = m_singularValues;
//     for ( long i=0; i<m_workMatrix.cols(); ++i) {
//        if ( m_singularValues(i) > pinvtoler )
//           singularValues_inv(i)=1.0/m_singularValues(i);
//       else singularValues_inv(i)=0;
//     }
//     pinvmat= (m_matrixV*singularValues_inv.asDiagonal()*m_matrixU.transpose());
//   }

  // Gets the center of a single-mode distribution, it performs variable mean shift
  template<typename dataType>
  Eigen::Vector4f getMultiDimMeanShift_color(std::vector<Eigen::Vector4f> &data, dataType &stdDevHist, dataType &concentration)
  {
//    cout << "Do meanShift\n";

    std::vector<Eigen::Vector4f> dataTemp = data;
    size_t size = data.size();

//    This one is specific for normalized color
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for(size_t i=0; i < data.size(); i++)
    {
      sum += data[i].head(3);
    }
    Eigen::Vector3f meanShift = sum/size;
//cout << "First meanShift " << meanShift.transpose() << endl;

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(size_t i=0; i < data.size(); i++)
    {
      Eigen::Vector3f diff = data[i].head(3) - meanShift;
      cov += diff * diff.transpose();
    }
//    cov /= size;
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov);
    stdDevHist = svd.singularValues().maxCoeff() / sqrt((double) size);
//    stdDevHist = 0.05;

    double shift = 1000; // Large limit
    int iteration_counter = 0;
    double convergence = 0.001;
    while(2*dataTemp.size() > size && shift > convergence)
    {
//  std::cout << "iteration " << iteration_counter << " Std " << stdDevHist << " maxEig " << svd.singularValues().maxCoeff() << std::endl;
      for(typename std::vector<Eigen::Vector4f>::iterator it=dataTemp.begin(); it != dataTemp.end(); )
      {
//        cout << "CHeck\n";
        Eigen::Vector3f diff = (*it).head(3) - meanShift;
        if(diff.norm() > stdDevHist)
        {
          sum -= (*it).head(3);
          cov -= diff * diff.transpose();
          dataTemp.erase(it);
        }
        else
          it++;
      }
//    cout << "sum " << sum.transpose() << " newdatasize " << dataTemp.size() << endl;
      Eigen::Vector3f meanUpdated = sum / dataTemp.size();
      shift = (meanUpdated - meanShift).norm();
      meanShift = meanUpdated;
      svd = Eigen::JacobiSVD<Eigen::Matrix3f>(cov);
//      stdDevHist = svd.singularValues().maxCoeff() / dataTemp.size();
      stdDevHist = svd.singularValues().maxCoeff() / sqrt((double) dataTemp.size());

      iteration_counter++;
    }
  //  std::cout << "Number of iterations: " << iteration_counter << " shift " << shift
  //            << " size " << (float)dataTemp.size() / size << " in " << clock.Tac() * 1e3 << " ms." << std::endl;

  //  stdDevHist = calcStdDev(data, meanShift);

    Eigen::Vector4f dominantColor;
    dominantColor.head(3) = meanShift;
    float averageIntensity = 0;
    for(unsigned i=0; i < dataTemp.size(); i++)
      averageIntensity += dataTemp[i][3];
    averageIntensity /= dataTemp.size();
    dominantColor(3) = averageIntensity;

//    concentration = float(dataTemp.size()) / size;
    int countFringe05 = 0;
    for(typename std::vector<Eigen::Vector4f>::iterator it=data.begin(); it != data.end(); it++)
        if((it->head(3) - meanShift).norm() < 0.05 ) //&& *it(3) - averageIntensity < 0.3)
            ++countFringe05;
    concentration = static_cast<dataType>(countFringe05) / data.size();

    return dominantColor;
  }

  // Gets the center of a single-mode distribution, it performs variable mean shift
  template<typename dataType>
  dataType getHistogramMeanShift(std::vector<dataType> &data, double range, dataType &stdDevHist_out)//, dataType &concentration05)
  {
//    cout << "Do meanShift\n";
  //  mrpt::utils::CTicTac clock;
  //  clock.Tic();
    size_t size = data.size();
    std::vector<dataType> dataTemp = data;

    dataType sum = 0;
    for(size_t i=0; i < data.size(); i++){
      sum += data[i];}
    dataType meanShift =sum/size;
    dataType stdDevHist = mrpt::math::stddev(data);

////    dataType meanShift;
//    double meanShift, stdDevHist;
//    mrpt::math::meanAndStd(data,meanShift,stdDevHist);
//    double sum = meanShift*data.size();
//cout << "mean " << meanShift << endl;

    //dataType step = 1;
    double shift = 1000;
    int iteration_counter = 0;
    double convergence = range * 0.001;
    while(2*dataTemp.size() > size && shift > convergence)
    {
//  std::cout << "iteration " << iteration_counter << " Std " << stdDevHist << std::endl;
      for(typename std::vector<dataType>::iterator it=dataTemp.begin(); it != dataTemp.end(); )
      {
//        cout << "CHeck\n";
        if(fabs(*it - meanShift) > stdDevHist)
        {
          sum -= *it;
          dataTemp.erase(it);
        }
        else
          it++;
      }
//    cout << "sum " << sum << " newdatasize " << dataTemp.size() << endl;
      double meanUpdated = sum / dataTemp.size();
      shift = fabs(meanUpdated - meanShift);
      meanShift = meanUpdated;
      stdDevHist = mrpt::math::stddev(dataTemp);

      iteration_counter++;
    }
  //  std::cout << "Number of iterations: " << iteration_counter << " shift " << shift
  //            << " size " << (float)dataTemp.size() / size << " in " << clock.Tac() * 1e3 << " ms." << std::endl;

  //  stdDevHist = calcStdDev(data, meanShift);

//    // Calculate concentration05
////    stdDevHist_out = float(dataTemp.size()) / size;
//    int countFringe05 = 0;
//    for(typename std::vector<dataType>::iterator it=data.begin(); it != data.end(); it++)
//        if(fabs(*it - meanShift) < 0.05)
//            ++countFringe05;
//    concentration05 = static_cast<dataType>(countFringe05) / data.size();

    return static_cast<dataType>(meanShift);
  }

//  // Bhattacharyya histogram distance function
//  double PBMAP_IMPEXP BhattacharyyaDist(std::vector<float> &hist1, std::vector<float> &hist2)
//  {
//    assert(hist1.size() == hist2.size());
//    double BhattachDist;
//    double BhattachDist_aux = 0.0;
//    for(unsigned i=0; i < hist1.size(); i++)
//      BhattachDist_aux += sqrt(hist1[i]*hist2[i]);
//
//    BhattachDist = sqrt(1 - BhattachDist_aux);
//
//    return BhattachDist;
//  }

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
