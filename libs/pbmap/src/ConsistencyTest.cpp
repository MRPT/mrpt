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

//#include <mrpt/pbmap/Miscellaneous.h>
//#include <mrpt/pbmap/ConsistencyTest.h>

using namespace std;
using namespace Eigen;
using namespace mrpt::pbmap;

ConsistencyTest::ConsistencyTest(PbMap &PBM_source, PbMap &PBM_target, std::map<unsigned, unsigned> &matched_planes) :
    PBMSource(PBM_source),
    PBMTarget(PBM_target),
    matched_planes(matched_planes)
{}

double ConsistencyTest::calcAlignmentError( std::map<unsigned, unsigned> &matched_planes, Eigen::Matrix4f &rigidTransf )
{
  double sum_depth_errors2 = 0;
  double sum_areas = 0;
//  unsigned count_pts = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
  {
      sum_depth_errors2 += (PBMSource.vPlanes[it->first].areaVoxels + PBMTarget.vPlanes[it->second].areaVoxels) *
                            pow(PBMTarget.vPlanes[it->second].v3normal .dot (compose(rigidTransf, PBMSource.vPlanes[it->first].v3center) - PBMTarget.vPlanes[it->second].v3center), 2);
      sum_areas += PBMSource.vPlanes[it->first].areaVoxels + PBMTarget.vPlanes[it->second].areaVoxels;
  }
  double avError2 = sum_depth_errors2 / sum_areas;
  return sqrt(avError2);
}


Eigen::Matrix4f ConsistencyTest::initPose( std::map<unsigned, unsigned> &matched_planes )
{
  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    for(unsigned i=0; i < 3; i++)
      for(unsigned j=0; j < 3; j++)
        normalCovariances(i,j) += PBMTarget.vPlanes[it->second].v3normal[i] * PBMSource.vPlanes[it->first].v3normal[i];

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixU() * svd.matrixV().transpose();

  if(Rotation.determinant() < 0)
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
//    cout << "numFull " << numFull << endl;
    translation = (centerFull_model - Rotation * centerFull_data) / numFull;
  }
  else
  {
//    cout << "No fullPlanes in matched context\n";
    translation = (center_model - Rotation * center_data) / numNonStruct;
  }

  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
  Eigen::Matrix4f rigidTransf;
  rigidTransf.block(0,0,3,3) = Rotation;
  rigidTransf.block(0,3,3,1) = translation;
  rigidTransf.row(3) << 0,0,0,1;
  return rigidTransf;
}

Eigen::Matrix4f ConsistencyTest::getRTwithModel( std::map<unsigned, unsigned> &matched_planes )
{
  assert(matched_planes.size() > 3);
  Eigen::Matrix4f rigidTransf = initPose( matched_planes ); // Inverse-Pose which maps from model to data

  double alignmentError = calcAlignmentError( matched_planes, rigidTransf );

  #ifdef _VERBOSE
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

