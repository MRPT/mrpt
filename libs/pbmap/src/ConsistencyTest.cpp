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

#include <mrpt/math/CArrayNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/ransac.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/pbmap/ConsistencyTest.h>
#include <mrpt/pbmap/SubgraphMatcher.h>
#include <mrpt/pbmap/PbMapLocaliser.h>

using namespace std;
using namespace Eigen;
using namespace mrpt::pbmap;
using namespace mrpt::math; // CMatrix*
using namespace mrpt;

ConsistencyTest::ConsistencyTest(PbMap &PBM_source, PbMap &PBM_target) :
    PBMSource(PBM_source),
    PBMTarget(PBM_target)
{}

double ConsistencyTest::calcAlignmentError( std::map<unsigned, unsigned> &matched_planes, Eigen::Matrix4f &rigidTransf )
{
  double sum_depth_errors2 = 0;
  double sum_areas = 0;
//  unsigned count_pts = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
      sum_depth_errors2 += (PBMSource.vPlanes[it->first].areaVoxels + PBMTarget.vPlanes[it->second].areaVoxels) *
                            pow(PBMTarget.vPlanes[it->second].v3normal.dot(compose(rigidTransf, PBMSource.vPlanes[it->first].v3center) - PBMTarget.vPlanes[it->second].v3center), 2);
      sum_areas += PBMSource.vPlanes[it->first].areaVoxels + PBMTarget.vPlanes[it->second].areaVoxels;
  }
  double avError2 = sum_depth_errors2 / sum_areas;
  return sqrt(avError2);
}

// Transformation from Source to Target
Eigen::Matrix4f ConsistencyTest::initPose( std::map<unsigned, unsigned> &matched_planes )
{
//  assert(matched_planes.size() >= 3);
  if(matched_planes.size() < 3)
  {
    cout << "Insuficient matched planes " << matched_planes.size() << endl;
    return Eigen::Matrix4f::Identity();
  }

  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
  normalCovariances(0,0) = 1;  // Limit rotation on y/z (horizontal) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    normalCovariances += PBMTarget.vPlanes[it->second].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixV() * svd.matrixU().transpose();

  // Check consitioning. 3 non-parallel planes are required in 6DoF, and only two for planar movement (3DoF)
  bool bPlanar_cond = false;
  for(map<unsigned, unsigned>::iterator it1 = matched_planes.begin(); it1 != matched_planes.end() && !bPlanar_cond; it1++)
  {
    map<unsigned, unsigned>::iterator it2 = it1; it2++;
    for(; it2 != matched_planes.end() && !bPlanar_cond; it2++)
    {
      Eigen::Vector3f planar_conditioning = PBMSource.vPlanes[it1->first].v3normal .cross (PBMSource.vPlanes[it2->first].v3normal);
      if(fabs(planar_conditioning(0)) > 0.33)
        bPlanar_cond = true;
    }
  }
//  float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//  if(conditioning > 100) // ^Dof
  if(!bPlanar_cond) // ^Dof
  {
//    cout << " ConsistencyTest::initPose -> Bad conditioning: " << conditioning << " -> Returning the identity\n";
    return Eigen::Matrix4f::Identity();
  }

  double det = Rotation.determinant();
  if(det != 1)
  {
    Eigen::Matrix3f aux;
    aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Rotation = svd.matrixV() * aux * svd.matrixU().transpose();
  }
//  if(Rotation.determinant() < 0)
//    Rotation.row(2) *= -1;
//  float det = Rotation.determinant();
//cout << "Rotation det " << det << endl;

//cout << "Rotation\n" << Rotation << endl;

//  // Evaluate error of each match looking for outliers
//  float sumError = 0;
//  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
//  {
//    float error = (PBMSource.vPlanes[it->first].v3normal .cross (Rotation * PBMTarget.vPlanes[it->second].v3normal ) ).norm();
//    sumError += error;
//    cout << "errorRot " << it->first << " " << it->second << " is " << error << endl;
//  }
//  cout << "Average rotation error " << sumError / matched_planes.size() << endl;

  // Calculate translation
  Vector3f translation;
  Matrix3f hessian = Matrix3f::Zero();
  Vector3f gradient = Vector3f::Zero();
  float accum_error2 = 0.0;
  hessian(0,0) = 1; // Limit movement on x (vertical) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
    float trans_error = (PBMSource.vPlanes[it->first].d - PBMTarget.vPlanes[it->second].d); //+n*t

    accum_error2 += trans_error * trans_error;
//    hessian += PBMTarget.vPlanes[it->second].v3normal * PBMTarget.vPlanes[it->second].v3normal.transpose();
//    gradient += -PBMTarget.vPlanes[it->second].v3normal * trans_error;
    hessian += PBMSource.vPlanes[it->first].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();
    gradient += PBMSource.vPlanes[it->first].v3normal * trans_error;
  }
  translation = -hessian.inverse() * gradient;
//cout << "Previous average translation error " << sumError / matched_planes.size() << endl;

//  // Evaluate error of each match looking for outliers
//  sumError = 0;
//  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
//  {
////    float trans_error = fabs(-PBMTarget.vPlanes[it->second].d + translation.dot(PBMTarget.vPlanes[it->second].v3normal) + PBMSource.vPlanes[it->first].d);
//    float trans_error = fabs((PBMTarget.vPlanes[it->second].d - translation.dot(PBMSource.vPlanes[it->first].v3normal)) - PBMSource.vPlanes[it->first].d);
//    sumError += trans_error;
//    cout << "errorTrans " << it->first << " " << it->second << " is " << trans_error << endl;
//  }
//cout << "Average translation error " << sumError / matched_planes.size() << endl;

  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
  Eigen::Matrix4f rigidTransf;
  rigidTransf.block(0,0,3,3) = Rotation;
  rigidTransf.block(0,3,3,1) = translation;
  rigidTransf.row(3) << 0,0,0,1;
  return rigidTransf;
}

// Transformation from Source to Target
Eigen::Matrix4f ConsistencyTest::estimatePose( std::map<unsigned, unsigned> &matched_planes )
{
  if(matched_planes.size() < 3)
  {
    cout << "Insuficient matched planes " << matched_planes.size() << endl;
    return Eigen::Matrix4f::Identity();
  }

  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
//  normalCovariances(0,0) = 1;  // Limit rotation on y/z (horizontal) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    normalCovariances += (PBMTarget.vPlanes[it->second].areaHull/PBMTarget.vPlanes[it->second].d) * PBMTarget.vPlanes[it->second].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();
//    normalCovariances += PBMTarget.vPlanes[it->second].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();

  // Introduce the virtual matching of two vertical planes n=(1,0,0)
  for(unsigned r=0; r<3; r++)
    for(unsigned c=0; c<3; c++)
      normalCovariances(0,0) += normalCovariances(r,c);

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixV() * svd.matrixU().transpose();

  // Check consitioning. 3 non-parallel planes are required in 6DoF, and only two for planar movement (3DoF)
  bool bPlanar_cond = false;
  for(map<unsigned, unsigned>::iterator it1 = matched_planes.begin(); it1 != matched_planes.end() && !bPlanar_cond; it1++)
  {
    map<unsigned, unsigned>::iterator it2 = it1; it2++;
    for(; it2 != matched_planes.end() && !bPlanar_cond; it2++)
    {
      Eigen::Vector3f planar_conditioning = PBMSource.vPlanes[it1->first].v3normal .cross (PBMSource.vPlanes[it2->first].v3normal);
      if(fabs(planar_conditioning(0)) > 0.33)
        bPlanar_cond = true;
    }
  }
//  float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//  if(conditioning > 100) // ^Dof
  if(!bPlanar_cond) // ^Dof
  {
//    cout << " ConsistencyTest::initPose -> Bad conditioning: " << conditioning << " -> Returning the identity\n";
    return Eigen::Matrix4f::Identity();
  }

  double det = Rotation.determinant();
  if(det != 1)
  {
    Eigen::Matrix3f aux;
    aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Rotation = svd.matrixV() * aux * svd.matrixU().transpose();
  }
//cout << "Rotation\n" << Rotation << endl;

//  // Evaluate error of each match looking for outliers
//  float sumError = 0;
//  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
//  {
//    float error = (PBMSource.vPlanes[it->first].v3normal .cross (Rotation * PBMTarget.vPlanes[it->second].v3normal ) ).norm();
//    sumError += error;
//    cout << "errorRot " << it->first << " " << it->second << " is " << error << endl;
//  }
//  cout << "Average rotation error " << sumError / matched_planes.size() << endl;

  // Calculate translation
  Vector3f translation;
  Matrix3f hessian = Matrix3f::Zero();
  Vector3f gradient = Vector3f::Zero();
//  float accum_error2 = 0.0;
//  hessian(0,0) = 1; // Limit movement on x (vertical) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
    float trans_error = (PBMSource.vPlanes[it->first].d - PBMTarget.vPlanes[it->second].d); //+n*t
//  cout << it->first << " area " << PBMSource.vPlanes[it->first].areaHull << endl;
//    accum_error2 += trans_error * trans_error;
//    hessian += PBMTarget.vPlanes[it->second].v3normal * PBMTarget.vPlanes[it->second].v3normal.transpose();
//    gradient += -PBMTarget.vPlanes[it->second].v3normal * trans_error;
    hessian += (PBMSource.vPlanes[it->first].areaHull / PBMSource.vPlanes[it->first].d) * PBMSource.vPlanes[it->first].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();
    gradient += (PBMSource.vPlanes[it->first].areaHull / PBMSource.vPlanes[it->first].d) * PBMSource.vPlanes[it->first].v3normal * trans_error;
  }

  // Introduce the virtual matching of two vertical planes n=(1,0,0)
  for(unsigned r=0; r<3; r++)
    for(unsigned c=0; c<3; c++)
      hessian(0,0) += hessian(r,c);

  translation = -hessian.inverse() * gradient;
//cout << "Previous average translation error " << sumError / matched_planes.size() << endl;

//  // Evaluate error of each match looking for outliers
//  sumError = 0;
//  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
//  {
////    float trans_error = fabs(-PBMTarget.vPlanes[it->second].d + translation.dot(PBMTarget.vPlanes[it->second].v3normal) + PBMSource.vPlanes[it->first].d);
//    float trans_error = fabs((PBMTarget.vPlanes[it->second].d - translation.dot(PBMSource.vPlanes[it->first].v3normal)) - PBMSource.vPlanes[it->first].d);
//    sumError += trans_error;
//    cout << "errorTrans " << it->first << " " << it->second << " is " << trans_error << endl;
//  }
//cout << "Average translation error " << sumError / matched_planes.size() << endl;

  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
  Eigen::Matrix4f rigidTransf;
  rigidTransf.block(0,0,3,3) = Rotation;
  rigidTransf.block(0,3,3,1) = translation;
  rigidTransf.row(3) << 0,0,0,1;
  return rigidTransf;
}

bool ConsistencyTest::estimatePoseWithCovariance( std::map<unsigned, unsigned> &matched_planes,
                                                 Eigen::Matrix4f &rigidTransf,
                                                 Eigen::Matrix<float,6,6> &covarianceM)
{
  if(matched_planes.size() < 3)
  {
    cout << "Insuficient matched planes " << matched_planes.size() << endl;
    return false;
  }

  unsigned col = 0;
  MatrixXf normalVectors(3,matched_planes.size());
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++, col++)
    normalVectors.col(col) = PBMTarget.vPlanes[it->first].v3normal;
  JacobiSVD<MatrixXf> svd_cond(normalVectors, ComputeThinU | ComputeThinV);
//  cout << "SV " << svd_cond.singularValues().transpose() << endl;
  if(svd_cond.singularValues()[0] / svd_cond.singularValues()[1] > 10)
    return false;

  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
//  normalCovariances(0,0) = 1;  // Limit rotation on y/z (horizontal) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    normalCovariances += (PBMSource.vPlanes[it->first].areaHull / PBMSource.vPlanes[it->first].d) * PBMTarget.vPlanes[it->second].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();

  // Limit the rotation to the X (vertical) axis by introducing the virtual matching of two large horizontal planes n=(1,0,0)
  for(unsigned r=0; r<3; r++)
    for(unsigned c=0; c<3; c++)
      normalCovariances(0,0) += fabs(normalCovariances(r,c));

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);

  Matrix3f Rotation = svd.matrixV() * svd.matrixU().transpose();
  double det = Rotation.determinant();
  if(det != 1)
  {
    Eigen::Matrix3f aux;
    aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Rotation = svd.matrixV() * aux * svd.matrixU().transpose();
  }

  // Calculate translation
  Vector3f translation;
  Matrix3f hessian = Matrix3f::Zero();
  Vector3f gradient = Vector3f::Zero();
//  hessian(0,0) = 1; // Limit movement on x (vertical) axis
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
    float trans_error = (PBMSource.vPlanes[it->first].d - PBMTarget.vPlanes[it->second].d); //+n*t
    hessian += (PBMSource.vPlanes[it->first].areaHull / PBMSource.vPlanes[it->first].d) * PBMSource.vPlanes[it->first].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();
    gradient += (PBMSource.vPlanes[it->first].areaHull / PBMSource.vPlanes[it->first].d) * PBMSource.vPlanes[it->first].v3normal * trans_error;
  }

  // Introduce the virtual matching of a vertical plane n=(1,0,0)
  for(unsigned r=0; r<3; r++)
    for(unsigned c=0; c<3; c++)
      hessian(0,0) += fabs(hessian(r,c));

  translation = -hessian.inverse() * gradient;

  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
//  Eigen::Matrix4f rigidTransf;
  rigidTransf.block(0,0,3,3) = Rotation;
  rigidTransf.block(0,3,3,1) = translation;
  rigidTransf.row(3) << 0,0,0,1;

//  Eigen::Matrix<float,6,6> covarianceM = Eigen::Matrix<float,6,6>::Zero();
  covarianceM.block(0,0,3,3) = hessian; // The first diagonal 3x3 block corresponds to the translation part
  covarianceM.block(3,3,3,3) = normalCovariances; // Rotation block

  return true;
}

Eigen::Matrix4f ConsistencyTest::initPose2D( std::map<unsigned, unsigned> &matched_planes )
{
  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    normalCovariances += PBMTarget.vPlanes[it->second].v3normal * PBMSource.vPlanes[it->first].v3normal.transpose();
  normalCovariances(1,1) += 100; // Rotation "restricted" to the y axis

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixU() * svd.matrixV().transpose();

  if(Rotation.determinant() < 0)
//    Rotation.row(2) *= -1;
    Rotation = -Rotation;

  // Calculate translation
  Vector3f translation;
  Vector3f center_data = Vector3f::Zero(), center_model = Vector3f::Zero();
  Vector3f centerFull_data = Vector3f::Zero(), centerFull_model = Vector3f::Zero();
  unsigned numFull = 0, numNonStruct = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
    if(PBMSource.vPlanes[it->first].bFromStructure) // The certainty in center of structural planes is too low
      continue;

    ++numNonStruct;
    center_data += PBMSource.vPlanes[it->first].v3center;
    center_model += PBMTarget.vPlanes[it->second].v3center;
    if(PBMSource.vPlanes[it->first].bFullExtent)
    {
      centerFull_data += PBMSource.vPlanes[it->first].v3center;
      centerFull_model += PBMTarget.vPlanes[it->second].v3center;
      ++numFull;
    }
  }
  if(numFull > 0)
  {
    translation = (-centerFull_model + Rotation * centerFull_data) / numFull;
  }
  else
  {
    translation = (-center_model + Rotation * center_data) / numNonStruct;
  }

  translation[1] = 0; // Restrict no translation in the y axis

  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
  Eigen::Matrix4f rigidTransf;
  rigidTransf.block(0,0,3,3) = Rotation;
  rigidTransf.block(0,3,3,1) = translation;
  rigidTransf.row(3) << 0,0,0,1;
  return rigidTransf;
}

Eigen::Matrix4f ConsistencyTest::getRTwithModel( std::map<unsigned, unsigned> &matched_planes )
{
  assert(matched_planes.size() >= 3);
  Eigen::Matrix4f rigidTransf = initPose( matched_planes ); // Inverse-Pose which maps from model to data

//  std::map<unsigned, unsigned> surrounding_planes = matched_planes;
//  surrounding_planes.insert(...);
  double alignmentError = calcAlignmentError( matched_planes, rigidTransf );

  #ifdef _VERBOSE
    cout << "INITIALIZATION POSE \n" << rigidTransf << endl;
    cout << "Alignment error " << alignmentError << endl;
  #endif

  unsigned nIter = 0;
  double improveRate = 0;
  while( nIter < 10 && improveRate < 0.999 )
  {
    // Find the rigid transformation which minimizes the distance to the corresponding planes in the model
    Vector3f ptInModelRef;
    Eigen::Matrix<float,6,1> v6JacDepthPlane;
    Eigen::Matrix<float,6,1> v6Error = Eigen::Matrix<float,6,1>::Zero();
    Eigen::Matrix<float,6,6> m6Hessian = Eigen::Matrix<float,6,6>::Zero();
    double depthError;
    for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    {
      ptInModelRef = compose(rigidTransf, PBMSource.vPlanes[it->first].v3center);
      depthError = PBMTarget.vPlanes[it->second].v3normal.dot(ptInModelRef - PBMTarget.vPlanes[it->second].v3center);

      v6JacDepthPlane.head(3) = PBMTarget.vPlanes[it->second].v3normal;
      v6JacDepthPlane(3) = -PBMTarget.vPlanes[it->second].v3normal(1)*ptInModelRef(2) + PBMTarget.vPlanes[it->second].v3normal(2)*ptInModelRef(1);
      v6JacDepthPlane(4) =  PBMTarget.vPlanes[it->second].v3normal(0)*ptInModelRef(2) - PBMTarget.vPlanes[it->second].v3normal(2)*ptInModelRef(0);
      v6JacDepthPlane(5) = -PBMTarget.vPlanes[it->second].v3normal(0)*ptInModelRef(1) + PBMTarget.vPlanes[it->second].v3normal(1)*ptInModelRef(0);
      m6Hessian += v6JacDepthPlane * v6JacDepthPlane.transpose();
      v6Error += v6JacDepthPlane * depthError;
    }

    Eigen::Matrix<float,6,1> updatedSE3 = (m6Hessian.inverse() * v6Error).transpose();
    mrpt::math::CArrayNumeric<double, 6> _updatedSE3;
    _updatedSE3(0) = updatedSE3(0);
    _updatedSE3(1) = updatedSE3(1);
    _updatedSE3(2) = updatedSE3(2);
    _updatedSE3(3) = updatedSE3(3);
    _updatedSE3(4) = updatedSE3(4);
    _updatedSE3(5) = updatedSE3(5);
    mrpt::math::CMatrixDouble44 CMatUpdate; mrpt::poses::CPose3D::exp(_updatedSE3).getHomogeneousMatrix(CMatUpdate);
    Eigen::Matrix4f updatePose;
    updatePose << CMatUpdate(0,0), CMatUpdate(0,1), CMatUpdate(0,2), CMatUpdate(0,3),
                  CMatUpdate(1,0), CMatUpdate(1,1), CMatUpdate(1,2), CMatUpdate(1,3),
                  CMatUpdate(2,0), CMatUpdate(2,1), CMatUpdate(2,2), CMatUpdate(2,3),
                  0,0,0,1;
    Eigen::Matrix4f tempPose = compose(updatePose, rigidTransf);
    double newError = calcAlignmentError( matched_planes, tempPose );
  #ifdef _VERBOSE
    cout << "New alignment error " << newError << endl;
  #endif

    if(newError < alignmentError)
    {
      improveRate = newError / alignmentError;
      alignmentError = newError;
      rigidTransf = tempPose;
    }
    else
    {
      #ifdef _VERBOSE
        cout << "Not converging in iteration " << nIter << endl;
      #endif
      break;
    }

    ++nIter;
  }

  #ifdef _VERBOSE
    cout << "Consistency test converged after " << nIter << endl;
  #endif

  return rigidTransf;
}


// Obtain the rigid transformation from 3 matched planes
CMatrixDouble getAlignment( const CMatrixDouble &matched_planes )
{
  assert(size(matched_planes,1) == 8 && size(matched_planes,2) == 3);

  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
  normalCovariances(0,0) = 1;
  for(unsigned i=0; i<3; i++)
  {
    Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
    Vector3f n_ii = Vector3f(matched_planes(4,i), matched_planes(5,i), matched_planes(6,i));
    normalCovariances += n_i * n_ii.transpose();
//    normalCovariances += matched_planes.block(i,0,1,3) * matched_planes.block(i,4,1,3).transpose();
  }

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixV() * svd.matrixU().transpose();

//  float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//  if(conditioning > 100)
//  {
//    cout << " ConsistencyTest::initPose -> Bad conditioning: " << conditioning << " -> Returning the identity\n";
//    return Eigen::Matrix4f::Identity();
//  }

  double det = Rotation.determinant();
  if(det != 1)
  {
    Eigen::Matrix3f aux;
    aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Rotation = svd.matrixV() * aux * svd.matrixU().transpose();
  }


  // Calculate translation
  Vector3f translation;
  Matrix3f hessian = Matrix3f::Zero();
  Vector3f gradient = Vector3f::Zero();
  hessian(0,0) = 1;
  for(unsigned i=0; i<3; i++)
  {
    float trans_error = (matched_planes(3,i) - matched_planes(7,i)); //+n*t
//    hessian += matched_planes.block(i,0,1,3) * matched_planes.block(i,0,1,3).transpose();
//    gradient += matched_planes.block(i,0,1,3) * trans_error;
    Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
    hessian += n_i * n_i.transpose();
    gradient += n_i * trans_error;
  }
  translation = -hessian.inverse() * gradient;
//cout << "Previous average translation error " << sumError / matched_planes.size() << endl;

//  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
//  Eigen::Matrix4f rigidTransf;
//  rigidTransf.block(0,0,3,3) = Rotation;
//  rigidTransf.block(0,3,3,1) = translation;
//  rigidTransf.row(3) << 0,0,0,1;

  CMatrixDouble rigidTransf(4,4);
  rigidTransf(0,0) = Rotation(0,0);
  rigidTransf(0,1) = Rotation(0,1);
  rigidTransf(0,2) = Rotation(0,2);
  rigidTransf(1,0) = Rotation(1,0);
  rigidTransf(1,1) = Rotation(1,1);
  rigidTransf(1,2) = Rotation(1,2);
  rigidTransf(2,0) = Rotation(2,0);
  rigidTransf(2,1) = Rotation(2,1);
  rigidTransf(2,2) = Rotation(2,2);
  rigidTransf(0,3) = translation(0);
  rigidTransf(1,3) = translation(1);
  rigidTransf(2,3) = translation(2);
  rigidTransf(3,0) = 0;
  rigidTransf(3,1) = 0;
  rigidTransf(3,2) = 0;
  rigidTransf(3,3) = 1;

  return rigidTransf;
}

// Ransac functions to detect outliers in the plane matching
void ransacPlaneAlignment_fit(
        const CMatrixDouble &planeCorresp,
        const mrpt::vector_size_t  &useIndices,
        vector< CMatrixDouble > &fitModels )
//        vector< Eigen::Matrix4f > &fitModels )
{
  ASSERT_(useIndices.size()==3);

  try
  {
    CMatrixDouble corresp(8,3);

//  cout << "Size planeCorresp: " << endl;
//  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
    for(unsigned i=0; i<3; i++)
      corresp.col(i) = planeCorresp.col(useIndices[i]);

    fitModels.resize(1);
//    Eigen::Matrix4f &M = fitModels[0];
    CMatrixDouble &M = fitModels[0];
    M = getAlignment(corresp);
  }
  catch(exception &)
  {
    fitModels.clear();
    return;
  }
}

void ransac3Dplane_distance(
        const CMatrixDouble &planeCorresp,
        const vector< CMatrixDouble > & testModels,
        const double distanceThreshold,
        unsigned int & out_bestModelIndex,
        mrpt::vector_size_t & out_inlierIndices )
{
  ASSERT_( testModels.size()==1 )
  out_bestModelIndex = 0;
  const CMatrixDouble &M = testModels[0];

  Eigen::Matrix3f Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
  Eigen::Vector3f translation; translation << M(0,3), M(1,3), M(2,3);

	ASSERT_( size(M,1)==4 && size(M,2)==4 )

  const size_t N = size(planeCorresp,2);
  out_inlierIndices.clear();
  out_inlierIndices.reserve(100);
  for (size_t i=0;i<N;i++)
  {
    const Eigen::Vector3f n_i = Eigen::Vector3f(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
    const Eigen::Vector3f n_ii = Rotation * Eigen::Vector3f(planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
    const float d_error = fabs((planeCorresp(7,i) - translation.dot(n_i)) - planeCorresp(3,i));
    const float angle_error = (n_i .cross (n_ii )).norm();

    if (d_error < distanceThreshold)
     if (angle_error < distanceThreshold) // Warning: this threshold has a different dimension
      out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac3Dplane_degenerate(
        const CMatrixDouble &planeCorresp,
        const mrpt::vector_size_t &useIndices )
{
  ASSERT_( useIndices.size()==3 )

  const Eigen::Vector3f n_1 = Eigen::Vector3f(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
  const Eigen::Vector3f n_2 = Eigen::Vector3f(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
  const Eigen::Vector3f n_3 = Eigen::Vector3f(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
//cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
    return true;

  return false;
}


// ------------------------------------------------------
//                                TestRANSAC
// ------------------------------------------------------
Eigen::Matrix4f ConsistencyTest::estimatePoseRANSAC( std::map<unsigned, unsigned> &matched_planes )
{
//  assert(matched_planes.size() >= 3);
//  CTicTac tictac;

  if(matched_planes.size() <= 3)
  {
    cout << "Insuficient matched planes " << matched_planes.size() << endl;
    return Eigen::Matrix4f::Identity();
  }

  CMatrixDouble planeCorresp(8, matched_planes.size());
  unsigned col = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++, col++)
  {
    planeCorresp(0,col) = PBMSource.vPlanes[it->first].v3normal(0);
    planeCorresp(1,col) = PBMSource.vPlanes[it->first].v3normal(1);
    planeCorresp(2,col) = PBMSource.vPlanes[it->first].v3normal(2);
    planeCorresp(3,col) = PBMSource.vPlanes[it->first].d;
    planeCorresp(4,col) = PBMTarget.vPlanes[it->second].v3normal(0);
    planeCorresp(5,col) = PBMTarget.vPlanes[it->second].v3normal(1);
    planeCorresp(6,col) = PBMTarget.vPlanes[it->second].v3normal(2);
    planeCorresp(7,col) = PBMTarget.vPlanes[it->second].d;
  }
//  cout << "Size " << matched_planes.size() << " " << size(1) << endl;

  mrpt::vector_size_t inliers;
//  Eigen::Matrix4f best_model;
  CMatrixDouble best_model;

  math::RANSAC ransac_executer;
  ransac_executer.execute(planeCorresp,
                        ransacPlaneAlignment_fit,
                        ransac3Dplane_distance,
                        ransac3Dplane_degenerate,
                        0.2,
                        3,  // Minimum set of points
                        inliers,
                        best_model,
                        true,   // Verbose
                        0.99999
                        );

//  cout << "Computation time: " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

  cout << "Size planeCorresp: " << size(planeCorresp,2) << endl;
  cout << "RANSAC finished: " << inliers.size() << " inliers: " << inliers << " . \nBest model: \n" << best_model << endl;
//        cout << "Best inliers: " << best_inliers << endl;

  Eigen::Matrix4f rigidTransf; rigidTransf << best_model(0,0), best_model(0,1), best_model(0,2), best_model(0,3), best_model(1,0), best_model(1,1), best_model(1,2), best_model(1,3), best_model(2,0), best_model(2,1), best_model(2,2), best_model(2,3), 0, 0, 0, 1;

//  return best_model;
  return rigidTransf;
}


//using namespace mrpt;
//using namespace mrpt::utils;
////using namespace mrpt::gui;
//using namespace mrpt::math;
//using namespace mrpt::random;
//using namespace std;
//
//void  ransac3Dplane_fit(
//	const CMatrixDouble  &allData,
//	const vector_size_t  &useIndices,
//	vector< CMatrixDouble > &fitModels )
//{
//	ASSERT_(useIndices.size()==3);
//
//	TPoint3D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]),allData(2,useIndices[0]) );
//	TPoint3D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]),allData(2,useIndices[1]) );
//	TPoint3D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]),allData(2,useIndices[2]) );
//
//	try
//	{
//		TPlane  plane( p1,p2,p3 );
//		fitModels.resize(1);
//		CMatrixDouble &M = fitModels[0];
//
//		M.setSize(1,4);
//		for (size_t i=0;i<4;i++)
//			M(0,i)=plane.coefs[i];
//	}
//	catch(exception &)
//	{
//		fitModels.clear();
//		return;
//	}
//
//
//
//}
//
//void ransac3Dplane_distance(
//	const CMatrixDouble &allData,
//	const vector< CMatrixDouble > & testModels,
//	const double distanceThreshold,
//	unsigned int & out_bestModelIndex,
//	vector_size_t & out_inlierIndices )
//{
//	ASSERT_( testModels.size()==1 )
//	out_bestModelIndex = 0;
//	const CMatrixDouble &M = testModels[0];
//
//	ASSERT_( size(M,1)==1 && size(M,2)==4 )
//
//	TPlane  plane;
//	plane.coefs[0] = M(0,0);
//	plane.coefs[1] = M(0,1);
//	plane.coefs[2] = M(0,2);
//	plane.coefs[3] = M(0,3);
//
//	const size_t N = size(allData,2);
//	out_inlierIndices.clear();
//	out_inlierIndices.reserve(100);
//	for (size_t i=0;i<N;i++)
//	{
//		const double d = plane.distance( TPoint3D( allData.get_unsafe(0,i),allData.get_unsafe(1,i),allData.get_unsafe(2,i) ) );
//		if (d<distanceThreshold)
//			out_inlierIndices.push_back(i);
//	}
//}
//
///** Return "true" if the selected points are a degenerate (invalid) case.
//  */
//bool ransac3Dplane_degenerate(
//	const CMatrixDouble &allData,
//	const mrpt::vector_size_t &useIndices )
//{
//	return false;
//}
//
//
//// ------------------------------------------------------
////				TestRANSAC
//// ------------------------------------------------------
//void ConsistencyTest::TestRANSAC()
//{
//	randomGenerator.randomize();
//
//	// Generate random points:
//	// ------------------------------------
//	const size_t N_plane = 300;
//	const size_t N_noise = 100;
//
//	const double PLANE_EQ[4]={ 1,-1,1, -2 };
//
//	CMatrixDouble data(3,N_plane+N_noise);
//	for (size_t i=0;i<N_plane;i++)
//	{
//		const double xx = randomGenerator.drawUniform(-3,3);
//		const double yy = randomGenerator.drawUniform(-3,3);
//		const double zz = -(PLANE_EQ[3]+PLANE_EQ[0]*xx+PLANE_EQ[1]*yy)/PLANE_EQ[2];
//		data(0,i) = xx;
//		data(1,i) = yy;
//		data(2,i) = zz;
//	}
//
//	for (size_t i=0;i<N_noise;i++)
//	{
//		data(0,i+N_plane) = randomGenerator.drawUniform(-4,4);
//		data(1,i+N_plane) = randomGenerator.drawUniform(-4,4);
//		data(2,i+N_plane) = randomGenerator.drawUniform(-4,4);
//	}
//
//
//	// Run RANSAC
//	// ------------------------------------
//	CMatrixDouble best_model;
//	vector_size_t best_inliers;
//	const double DIST_THRESHOLD = 0.2;
//
//
//	CTicTac	tictac;
//	const size_t TIMES=100;
//
//	for (size_t iters=0;iters<TIMES;iters++)
//		math::RANSAC::execute(
//			data,
//			ransac3Dplane_fit,
//			ransac3Dplane_distance,
//			ransac3Dplane_degenerate,
//			DIST_THRESHOLD,
//			3,  // Minimum set of points
//			best_inliers,
//			best_model,
//			iters==0   // Verbose
//			);
//
//	cout << "Computation time: " << tictac.Tac()*1000.0/TIMES << " ms" << endl;
//
//	ASSERT_(size(best_model,1)==1 && size(best_model,2)==4)
//
//	cout << "RANSAC finished: Best model: " << best_model << endl;
////	cout << "Best inliers: " << best_inliers << endl;
//
//	TPlane  plane( best_model(0,0), best_model(0,1),best_model(0,2),best_model(0,3) );
//
//
//}
