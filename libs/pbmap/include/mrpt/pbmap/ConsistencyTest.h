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

#ifndef __CONSISTENCYTEST_H
#define __CONSISTENCYTEST_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/types_math.h> // Eigen
#include <mrpt/pbmap/link_pragmas.h>
#include <mrpt/pbmap/PbMap.h>

namespace mrpt {
namespace pbmap {

  /*! This class computes the rigid transformation between two sets of matched planes,
   *  and provides a measure of their rigid adjustment
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PBMAP_IMPEXP ConsistencyTest
  {
   public:

    /*!Constructor */
    ConsistencyTest(PbMap &PBM_source, PbMap &PBM_target);

  //  /**! Get diamond of points around the center. This is used to calculate the adjustment error with a model plane */
  //  void calcDiamondPlane(Plane& plane);

    /*!Calculate the alignment error between two sets of matched planes.
      The input rigid transformation "se3rigidTransfInv" is used to project the centroids of one set of planes into their
      matched planes and returns the sum of cuadratic distances */
    double calcAlignmentError( std::map<unsigned, unsigned> &matched_planes, Eigen::Matrix4f &rigidTransf );

    /*!Return an initial guess for the rigid transformation which aligns two matched places.
    The translation is calculated from the planes centroids and the rotation from the alignment of the plane's normals.*/
    Eigen::Matrix4f initPose( std::map<unsigned, unsigned> &matched_planes);
    Eigen::Matrix4f estimatePose( std::map<unsigned, unsigned> &matched_planes ); // Weighted with the area
    bool estimatePoseWithCovariance(std::map<unsigned, unsigned> &matched_planes, Eigen::Matrix4f &rigidTransf, Eigen::Matrix<float,6,6> &covarianceM);

    /*!Return an initial guess for the rigid transformation which aligns two matched places.
    The translation is calculated from the planes centroids and the rotation from the alignment of the plane's normals.
    A planar movement is assumed (wheeled robot)*/
    Eigen::Matrix4f initPose2D( std::map<unsigned, unsigned> &matched_planes);

    /*!Return the estimated rigid transformation which aligns two matched subgraphs (i.e. neighborhoods of planes).
    This function iteratively minimizes the alignment error of the matched planes wrt the rigid transformation.*/
    Eigen::Matrix4f getRTwithModel( std::map<unsigned, unsigned> &matched_planes );


//Eigen::Matrix4f getAlignment( const mrpt::math::CMatrixFixedNumeric<float,3,8> &matched_planes );

Eigen::Matrix4f estimatePoseRANSAC( std::map<unsigned, unsigned> &matched_planes );

   private:

    /*!One of the subgraphs matched by SubgraphMatcher.*/
    PbMap &PBMSource;

    /*!The other subgraph matched by SubgraphMatcher.*/
    PbMap &PBMTarget;

    /*!List of pairs of matched planes from the PbMaps PBMSource with those from PBMTarget*/
    std::map<unsigned, unsigned> matched_planes;

//// Ransac functions to detect outliers in the plane matching
//void ransacPlaneAlignment_fit( const mrpt::math::CMatrixFloat &planeCorresp,
//                                const vector_size_t  &useIndices,
////                                vector< Eigen::Matrix4f > &fitModels );
//                                vector< mrpt::math::CMatrixFloat44 > &fitModels );
//
//void ransac3Dplane_distance( const mrpt::math::CMatrixFloat &planeCorresp,
//                              const vector< Eigen::Matrix4f > & testModels,
//                              const double distanceThreshold,
//                              unsigned int & out_bestModelIndex,
//                              vector_size_t & out_inlierIndices );
//
//bool ransac3Dplane_degenerate( const mrpt::math::CMatrixFloat &planeCorresp,
//                                const mrpt::vector_size_t &useIndices );
//
//void TestRANSAC();

  };

} } // End of namespaces

#endif
#endif
