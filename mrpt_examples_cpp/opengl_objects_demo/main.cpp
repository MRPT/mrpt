/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPlanarLaserScan.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>

#include <Eigen/Dense>  // transpose()
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt::literals;  // _deg
using namespace mrpt;            // viz::
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std::string_literals;

// ------------------------------------------------------
//				TestOpenGLObjects
// ------------------------------------------------------
void TestOpenGLObjects()
{
  CDisplayWindow3D win("Demo of MRPT's OpenGL objects", 640, 480);

  Scene::Ptr& theScene = win.get3DSceneAndLock();

  auto& rng = mrpt::random::getRandomGenerator();

  // Objects:
  double off_x = 0;
  const double off_y_label = 20;
  const double STEP_X = 25;

  // XY Grid
  /**
   * Define each one of the objects in its own scope and just attach it to the
   * scene by using insert(obj) method call.
   */
  {
    // using mrpt smart pointers so that obj survives outside this scope.
    auto obj = viz::CGridPlaneXY::Create(-7, 7, -7, 7, 0, 1);
    obj->setColor(0.7f, 0.7f, 0.7f);
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    auto obj2 = viz::CGridPlaneXY::Create(-9, 9, -9, 9, 0, 2);
    obj2->setColor(0.3f, 0.3f, 0.3f, 0.99f);
    obj2->setLocation(off_x, 20, 0);
    obj2->enableAntiAliasing();
    theScene->insert(obj2);

    auto gl_txt = viz::CText::Create("CGridPlaneXY");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // XZ Grid
  {
    auto obj = viz::CGridPlaneXZ::Create(-7, 7, -7, 7, 0, 1);
    obj->setColor(0.7f, 0.7f, 0.7f);
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    auto gl_txt = viz::CText::Create("CGridPlaneXZ");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // Arrow
  {
    auto obj = viz::CArrow::Create(
        mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(3, 0, 0), 0.2f, 0.1f, 0.2f);
    obj->setLocation(off_x, 0, 0);
    obj->setColor(1, 0, 0);
    obj->setName("arrow #1");
    obj->enableShowName();
    theScene->insert(obj);

    auto obj2 = viz::CArrow::Create(
        mrpt::math::TPoint3Df(1, 2, 3), mrpt::math::TPoint3Df(6, -3, 0), 0.1f, 0.1f, 0.3f);
    obj2->setLocation(off_x, 0, 0);
    obj2->setColor(0, 0, 1);
    theScene->insert(obj2);

    auto gl_txt = viz::CText::Create("CArrow");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // Axis
  {
    auto obj = viz::CAxis::Create(-6, -6, -6, 6, 6, 6, 2, 2, true);
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    auto gl_txt = viz::CText::Create("CAxis");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // Box
  {
    auto obj = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), true, 3.0);
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    auto obj2 = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
    obj2->setLocation(off_x, 4, 0);
    theScene->insert(obj2);

    auto obj3 = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
    obj3->enableBoxBorder(true);
    obj3->setLineWidth(3);
    obj3->setColor_u8(0xff, 0x00, 0x00, 0xa0);
    obj3->setLocation(off_x, 8, 0);
    theScene->insert(obj3);

    auto obj4 = viz::CBox::Create(TPoint3D(-2.0, -1.0, -0.2), TPoint3D(2.0, 1.0, 0.2), false);
    obj4->setLocation(off_x, 12, 0);
    obj4->enableBoxBorder(true);
    obj4->setLineWidth(3);
    obj4->setColor_u8(0xff, 0x00, 0x00, 0xa0);
    theScene->insert(obj4);

    auto gl_txt = viz::CText::Create("CBox");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // Frustum
  {
    auto obj = viz::CFrustum::Create(1, 5, 60, 45, 1.5f, true, false);
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    auto obj2 = viz::CFrustum::Create(1, 5, 60, 45, 2.5f, true, true);
    obj2->setLocation(off_x, 6, 0);
    obj2->setColor_u8(0xff, 0x00, 0x00, 0x80);
    theScene->insert(obj2);

    // Default camera params:
    mrpt::img::TCamera c;
    c.nrows = 800;
    c.ncols = 600;
    c.setIntrinsicParamsFromValues(700, 700, 400, 300);

    auto obj3 = viz::CFrustum::Create(c);
    obj3->setLocation(off_x, 12, 0);
    obj3->setColor_u8(0xff, 0x00, 0x00, 0x80);
    theScene->insert(obj3);

    auto gl_txt = viz::CText::Create("CFrustum");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // Cylinder
  {
    auto obj = viz::CCylinder::Create(2, 2, 4, 20);
    obj->setLocation(off_x, 0, 0);
    obj->setColor(0, 0, 0.8f);
    theScene->insert(obj);

    auto obj2 = viz::CCylinder::Create(2, 1, 4, 20);
    obj2->setLocation(off_x, 6, 0);
    obj2->setColor(0, 0, 0.8f);
    theScene->insert(obj2);

    auto obj3 = viz::CCylinder::Create(2, 0, 4, 20);
    obj3->setLocation(off_x, -6, 0);
    obj3->setColor(0, 0, 0.8f);
    theScene->insert(obj3);

    auto gl_txt = viz::CText::Create("CCylinder");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CDisk
  {
    {
      auto obj = viz::CDisk::Create(2.0f, 1.8f, 50);
      obj->setLocation(off_x, 0, 0);
      obj->setColor(0.8f, 0, 0);
      theScene->insert(obj);
    }

    {
      auto obj = viz::CDisk::Create(2.0f, 0, 50);
      obj->setLocation(off_x, 5, 0);
      obj->setColor(0.8f, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CDisk");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CEllipsoid3D
  {
    const double cov3d_dat[] = {0.9, 0.7, -0.4, 0.7, 1.6, -0.6, -0.4, -0.6, 1.5};
    const double cov2d_dat[] = {0.9, 0.7, 0.7, 1.6};
    mrpt::math::CMatrixDouble22 cov2d(cov2d_dat);
    mrpt::math::CMatrixDouble33 cov3d(cov3d_dat);

    {
      auto obj = viz::CEllipsoid2D::Create();
      obj->setCovMatrix(cov2d);
      obj->setLocation(off_x, 6, 0);
      obj->setQuantiles(2.0);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CEllipsoid2D::Create();
      obj->setCovMatrix(cov2d);
      obj->setLocation(off_x, 12, 0);
      obj->enableDrawSolid3D(true);
      obj->setQuantiles(2.0);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CEllipsoid3D::Create();
      obj->setCovMatrix(cov3d);
      obj->setQuantiles(2.0);
      obj->enableDrawSolid3D(false);
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CEllipsoid3D::Create();
      obj->setCovMatrix(cov3d);
      obj->setQuantiles(2.0);
      obj->materialShininess(0.99f);
      obj->setName("Ellipsoid shininess=0.99");
      obj->enableShowName();
      obj->enableDrawSolid3D(true);
      obj->setLocation(off_x, -6, 0);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CEllipsoid3D::Create();
      obj->setCovMatrix(cov3d);
      obj->setQuantiles(2.0);
      obj->materialShininess(0.01f);
      obj->setName("Ellipsoid shininess=0.01");
      obj->enableShowName();
      obj->enableDrawSolid3D(true);
      obj->setLocation(off_x, -12, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CEllipsoid3D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CEllipsoidRangeBearing2D
  {  // (range,bearing) -> (x,y)
    const double cov_params_dat[] = {0.2, 0, 0, 0.1};
    const double mean_params_dat[] = {3.0, 0.5};
    mrpt::math::CMatrixFixed<double, 2, 2> cov_params(cov_params_dat);
    mrpt::math::CMatrixFixed<double, 2, 1> mean_params(mean_params_dat);

    {
      auto obj = viz::CEllipsoidRangeBearing2D::Create();
      obj->setCovMatrixAndMean(cov_params, mean_params);
      obj->setLocation(off_x, 6, 0);
      obj->setQuantiles(2.0f);
      // obj->setNumberOfSegments(50);
      theScene->insert(obj);

      auto obj_corner = viz::stock_objects::CornerXYSimple(1, 3);
      obj_corner->setLocation(off_x, 6, 0);
      theScene->insert(obj_corner);
    }
  }
  {  // (range,bearing) -> (x,y)
    const double cov_params_dat[] = {0.2, 0.09, 0.09, 0.1};
    const double mean_params_dat[] = {5.0, -0.5};
    mrpt::math::CMatrixFixed<double, 2, 2> cov_params(cov_params_dat);
    mrpt::math::CMatrixFixed<double, 2, 1> mean_params(mean_params_dat);

    {
      auto obj = viz::CEllipsoidRangeBearing2D::Create();
      obj->setCovMatrixAndMean(cov_params, mean_params);
      obj->setLocation(off_x, 0, 0);
      obj->setQuantiles(2.0f);
      // obj->setNumberOfSegments(50);
      theScene->insert(obj);

      auto obj_corner = viz::stock_objects::CornerXYSimple(1, 3);
      obj_corner->setLocation(off_x, 0, 0);
      theScene->insert(obj_corner);
    }

    auto gl_txt = viz::CText::Create("CEllipsoidRangeBearing2D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CEllipsoidInverseDepth2D
  {  // (inv_range,yaw) -> (x,y)
    // Formula from our book (ch8) for confidence intervals of 3sigmas:
    const double max_dist = 1e4;
    const double min_dist = 1;
    const double rho_mean = 0.5 * (1. / min_dist + 1. / max_dist);
    const double rho_std = (1. / 6.) * (1. / min_dist - 1. / max_dist);

    const double cov_params_dat[] = {square(rho_std), 0, 0, square(2.0_deg)};
    const double mean_params_dat[] = {rho_mean, 70.0_deg};
    mrpt::math::CMatrixFixed<double, 2, 2> cov_params(cov_params_dat);
    mrpt::math::CMatrixFixed<double, 2, 1> mean_params(mean_params_dat);

    {
      auto obj = viz::CEllipsoidInverseDepth2D::Create();
      obj->setCovMatrixAndMean(cov_params, mean_params);
      obj->setLocation(off_x, 6, 0);
      obj->setQuantiles(3.f);
      obj->setNumberOfSegments(100);
      theScene->insert(obj);

      auto obj_corner = viz::stock_objects::CornerXYSimple(1, 3);
      obj_corner->setLocation(off_x, 6, 0);
      theScene->insert(obj_corner);
    }

    auto gl_txt = viz::CText::Create("CEllipsoidInverseDepth2D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CEllipsoidInverseDepth3D
  {  // (inv_range,yaw,pitch) -> (x,y,z)
    // Formula from our book (ch8) for confidence intervals of 3sigmas:
    const double max_dist = 1e2;
    const double min_dist = 1;
    const double rho_mean = 0.5 * (1. / min_dist + 1. / max_dist);
    const double rho_std = (1. / 6.) * (1. / min_dist - 1. / max_dist);

    const double cov_params_dat[] = {square(rho_std), 0, 0, 0, square(2.0_deg), 0, 0, 0,
                                     square(2.0_deg)};
    const double mean_params_dat[] = {rho_mean, 30.0_deg, -45.0_deg};
    mrpt::math::CMatrixFixed<double, 3, 3> cov_params(cov_params_dat);
    mrpt::math::CMatrixFixed<double, 3, 1> mean_params(mean_params_dat);

    {
      auto obj = viz::CEllipsoidInverseDepth3D::Create();
      obj->setCovMatrixAndMean(cov_params, mean_params);
      obj->setLocation(off_x, 0, 0);
      obj->setQuantiles(3.f);
      // obj->setNumberOfSegments(50);
      theScene->insert(obj);

      auto obj_corner = viz::stock_objects::CornerXYZSimple(1, 3);
      obj_corner->setLocation(off_x, 0, 0);
      theScene->insert(obj_corner);
    }

    auto gl_txt = viz::CText::Create("CEllipsoidInverseDepth3D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CMesh3D
  {
    auto obj = viz::CMesh3D::Create();
    obj->enableShowEdges(false);
    obj->enableShowFaces(true);
    obj->enableShowVertices(false);
    obj->setLocation(off_x, 0, 0);

    const unsigned int rows = 200, cols = 200;
    const unsigned int num_verts = (rows + 1) * (cols + 1);
    const unsigned int num_faces = rows * cols;
    int* vert_per_face = new int[num_faces];
    int* face_verts = new int[4 * num_faces];
    float* vert_coords = new float[3 * num_verts];

    // Assign vertices to faces and set them to be Quads
    unsigned int first_ind = 0;
    for (unsigned int u = 0; u < cols; u++)
    {
      for (unsigned int v = 0; v < rows; v++)
      {
        const unsigned int face_ind = v + u * rows;
        vert_per_face[face_ind] = 4;

        face_verts[4 * face_ind] = first_ind;
        face_verts[4 * face_ind + 1] = first_ind + 1;
        face_verts[4 * face_ind + 2] = first_ind + rows + 2;
        face_verts[4 * face_ind + 3] = first_ind + rows + 1;
        first_ind++;
      }

      first_ind++;
    }

    // Create vert coords
    for (unsigned int u = 0; u <= cols; u++)
      for (unsigned int v = 0; v <= rows; v++)
      {
        const unsigned int vert_ind = v + u * (rows + 1);

        vert_coords[3 * vert_ind] =
            (2.f - 0.01f * u) * (2.f + cos(0.01f * M_PI * v)) * cos(0.01f * M_PI * u);
        vert_coords[3 * vert_ind + 1] =
            (2.f - 0.01f * u) * (2.f + cos(0.01f * M_PI * v)) * sin(0.01f * M_PI * u);
        vert_coords[3 * vert_ind + 2] = 3.f * 0.01f * u + (2.f - 0.01f * u) * sin(0.01f * M_PI * v);
      }

    obj->loadMesh(num_verts, num_faces, vert_per_face, face_verts, vert_coords);
    theScene->insert(obj);

    auto gl_txt = viz::CText::Create("CMesh3D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CMeshFast, CMesh:
  {
    viz::CMeshFast::Ptr obj1 = viz::CMeshFast::Create();
    viz::CMeshFast::Ptr obj2 = viz::CMeshFast::Create();
    viz::CMesh::Ptr obj3 = viz::CMesh::Create();
    viz::CMesh::Ptr obj4 = viz::CMesh::Create();
    viz::CMesh::Ptr obj5 = viz::CMesh::Create();

    obj1->setXBounds(-1, 1);
    obj1->setYBounds(-1, 1);

    const int W = 128, H = 128;

    mrpt::math::CMatrixDynamic<float> Z(H, W);

    for (int r = 0; r < H; r++)
      for (int c = 0; c < W; c++) Z(r, c) = sin(0.05 * (c + r) - 0.5) * cos(0.9 - 0.03 * r);

    const std::string texture_file =
        mrpt::system::getShareMRPTDir() + "datasets/sample-texture-terrain.jpg"s;

    const std::string texture_file2 =
        mrpt::system::getShareMRPTDir() + "datasets/sample-texture-corners.jpg"s;

    mrpt::img::CImage im, im2;

    // obj1:
    obj1->setZ(Z);
    obj1->enableColorFromZ(true);
    obj1->setPointSize(2.0);
    obj1->setLocation(off_x, 0, 0);
    theScene->insert(obj1);

    // obj 2:
    if (im.loadFromFile(texture_file))
    {
      obj2->assignImageAndZ(im, Z);
      obj2->setPointSize(2.0);
      obj2->setLocation(off_x, 3, 0);
      theScene->insert(obj2);
    }
    {
      auto gl_txt = viz::CText::Create("CMeshFast");
      gl_txt->setLocation(off_x, off_y_label, 0);
      theScene->insert(gl_txt);
    }

    off_x += STEP_X;

    // obj 3:
    obj3->setZ(Z);
    obj3->enableColorFromZ(true, mrpt::img::cmJET);
    obj3->enableWireFrame(true);
    obj3->setLocation(off_x, 0, 0);
    obj3->cullFaces(mrpt::viz::TCullFace::BACK);
    // obj3->enableTextureMipMap(false);
    theScene->insert(obj3);

    // obj 4:
    if (!im.isEmpty())
    {
      obj4->assignImageAndZ(im, Z);
      obj4->setLocation(off_x, 3, 0);
      obj4->cullFaces(mrpt::viz::TCullFace::BACK);
      // obj4->enableTextureMipMap(false);
      theScene->insert(obj4);
    }
    // obj 5:
    if (!im.isEmpty())
    {
      obj5->assignImageAndZ(im, Z);
      obj5->setMeshTextureExtension(0.25, 0.5);
      obj5->setLocation(off_x + 3, 3, 0);
      // obj5->enableTextureMipMap(false);
      theScene->insert(obj5);
    }

    mrpt::math::CMatrixDynamic<float> Z2(H, W);

    if (im2.loadFromFile(texture_file2))
    {
      im2.getAsMatrix(Z2);

      viz::CMesh::Ptr obj = viz::CMesh::Create(
          false /*transparency*/,  //
          -1, 1,                   // x: Min Max
          1, -1                    // y: Min Max
      );
      obj->assignImageAndZ(im2, Z2);
      obj->setLocation(off_x, 6, 0);
      theScene->insert(obj);
    }
    if (im2.loadFromFile(texture_file2))
    {
      viz::CMesh::Ptr obj = viz::CMesh::Create(
          false /*transparency*/,  //
          1, -1,                   // x: Min Max
          -1, 1                    // y: Min Max
      );
      obj->assignImageAndZ(im2, Z2);
      obj->setLocation(off_x, 9, 0);
      theScene->insert(obj);
    }

    {
      auto gl_txt = viz::CText::Create("CMesh");
      gl_txt->setLocation(off_x, off_y_label, 0);
      theScene->insert(gl_txt);
    }
  }
  off_x += STEP_X;

  // CPointCloud
  {
    auto obj = viz::CPointCloud::Create();
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    obj->setPointSize(1.0);
    obj->enableColorFromY();

    for (int i = 0; i < 100000; i++)
      obj->insertPoint(
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(-5, 5),
          rng.drawUniform<float>(-5, 5));

    auto gl_txt = viz::CText::Create("CPointCloud");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CPointCloudColoured
  {
    auto obj = viz::CPointCloudColoured::Create();
    obj->setLocation(off_x, 0, 0);
    theScene->insert(obj);

    obj->setPointSize(1.0);

    for (int i = 0; i < 200; i++)
      obj->push_back(
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(-5, 5),
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(0, 1), rng.drawUniform<float>(0, 1),
          rng.drawUniform<float>(0, 1));

    auto gl_txt = viz::CText::Create("CPointCloudColoured");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CPolyhedron
  {
    {
      auto obj = viz::CPolyhedron::CreateCuboctahedron(1.0);
      obj->setLocation(off_x, 0, 0);
      obj->setWireframe(true);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CPolyhedron::CreateDodecahedron(1.0);
      obj->setLocation(off_x, -5, 0);
      theScene->insert(obj);
    }
    {
      auto obj = viz::CPolyhedron::CreateIcosahedron(1.0);
      obj->setLocation(off_x, 5, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CPolyhedron");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CSphere
  {
    {
      auto obj = viz::CSphere::Create(3.0);
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CSphere");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CText
  {
    {
      auto obj = viz::CText::Create(
          "This is a CText example! My size is invariant to "
          "eye-distance");
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CText");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CText3D
  {
    {
      auto obj = viz::CText3D::Create("I'm a cool CText3D!");
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    {
      auto obj = viz::CText3D::Create("A rotated CText3D");
      obj->setPose(
          mrpt::poses::CPose3D(off_x, 0, 0, 0, 0, 0) +
          mrpt::poses::CPose3D::FromString("[0 5 0 180 0 90]"));
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CText3D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CColorMap
  {
    {
      auto obj =
          viz::CColorBar::Create(mrpt::img::cmHOT, 0.2, 1.0, 0.0, 1.0, -50.0, 100.0, "%7.02f m/s");
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CColorBar");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CSetOfLines
  {
    for (int rep = 0; rep < 2; rep++)
    {
      auto obj = viz::CSetOfLines::Create();
      obj->setLocation(off_x, rep * 20, 0);

      for (int i = 0; i < 15; i++)
        obj->appendLine(
            rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-5, 5),
            rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-5, 5));

      if (rep == 1) obj->setVerticesPointSize(5.0f);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CSetOfLines");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CSimpleLine
  {
    {
      auto obj = viz::CSimpleLine::Create();
      obj->setLocation(off_x, 0, 0);

      obj->setLineCoords(
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(-5, 5),
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(-5, 5),
          rng.drawUniform<float>(-5, 5), rng.drawUniform<float>(-5, 5));

      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CSimpleLine");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CVectorField2D
  {
    {
      auto obj = viz::CVectorField2D::Create();
      obj->setLocation(off_x, 0, 0);

      CMatrixFloat x(16, 16), y(16, 16);
      for (int i = 0; i < x.rows(); i++)
        for (int j = 0; j < x.cols(); j++)
        {
          x(i, j) = sin(0.3 * i);
          y(i, j) = cos(0.3 * i);
        }
      obj->setVectorField(x, y);
      obj->setPointColor(1, 0.3f, 0);
      obj->setVectorFieldColor(0, 0, 1);
      obj->setPointSize(3.0);
      obj->setLineWidth(2.0);
      obj->setGridCenterAndCellSize(0, 0, 1.2f, 1.2f);
      obj->adjustVectorFieldToGrid();
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CVectorField2D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CVectorField3D
  {
    {
      const unsigned int num = 20;
      const float scale = 0.8 * STEP_X / num;
      auto obj = viz::CVectorField3D::Create();
      obj->setLocation(off_x, -0.5 * scale * num, 0);  //

      CMatrixFloat x(num, num), y(num, num), z(num, num);
      CMatrixFloat vx(num, num), vy(num, num), vz(num, num);
      for (int i = 0; i < x.rows(); i++)
        for (int j = 0; j < x.cols(); j++)
        {
          x(i, j) = (i - 0.5 * num) * scale;
          y(i, j) = j * scale;
          z(i, j) = 3 * sin(0.3 * i) * cos(0.3 * j);
          vx(i, j) = 0.4 * sin(0.3 * i);
          vy(i, j) = 0.8 * cos(0.3 * i);
          vz(i, j) = 0.01 * i * j;
        }
      obj->setPointCoordinates(x, y, z);
      obj->setVectorField(vx, vy, vz);
      obj->setPointColor(1, 0.3f, 0);
      obj->setVectorFieldColor(0, 0, 1);
      obj->setPointSize(3.0);
      obj->setLineWidth(2.0);
      obj->enableColorFromModule();
      obj->setMaxSpeedForColor(3.0);
      obj->setMotionFieldColormap(0, 0, 1, 1, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CVectorField3D");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::BumblebeeCamera
  {
    {
      auto obj = viz::stock_objects::BumblebeeCamera();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::BumblebeeCamera()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::CornerXYSimple
  {
    {
      auto obj = viz::stock_objects::CornerXYSimple(1, 3);
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::CornerXYSimple()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::CornerXYZSimple
  {
    {
      auto obj = viz::stock_objects::CornerXYZSimple(1, 3);
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::CornerXYZSimple()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::CornerXYZ
  {
    {
      auto obj = viz::stock_objects::CornerXYZ();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::CornerXYZ()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CSetOfTriangles: tested via stock_objects:
  // stock_objects::RobotPioneer
  {
    {
      auto obj = viz::stock_objects::RobotPioneer();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::RobotPioneer()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::Hokuyo_URG
  {
    {
      auto obj = viz::stock_objects::Hokuyo_URG();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::Hokuyo_URG()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::Hokuyo_UTM
  {
    {
      auto obj = viz::stock_objects::Hokuyo_UTM();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::Hokuyo_UTM()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::Househam_Sprayer
  {
    {
      auto obj = viz::stock_objects::Househam_Sprayer();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::Househam_Sprayer()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // stock_objects::RobotRhodon
  {
    {
      auto obj = viz::stock_objects::RobotRhodon();
      obj->setLocation(off_x, 0, 0);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("stock_objects::RobotRhodon()");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CPlanarLaserScan
  {
    auto obj = mrpt::viz::CPlanarLaserScan::Create();
    obj->setPose(mrpt::poses::CPose3D(off_x, 0, 0, 90.0_deg, 0, 0));

    mrpt::obs::CObservation2DRangeScan scan;
    mrpt::obs::stock_observations::example2DRangeScan(scan);

    obj->setScan(scan);

    theScene->insert(obj);

    auto gl_txt = viz::CText::Create("CPlanarLaserScan");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // CTexturedPlane
  {
    mrpt::img::CImage pic, picAlpha;
    const size_t W = 256, H = 256;
    pic.resize(W, H, mrpt::img::CH_RGB);
    picAlpha.resize(W, H, mrpt::img::CH_GRAY);
    pic.filledRectangle({0, 0}, {(int)(W - 1), (int)(H - 1)}, mrpt::img::TColor::black());
    pic.filledRectangle({0, 0}, {(int)(W / 4), (int)(H / 2)}, mrpt::img::TColor::white());

    picAlpha.filledRectangle(
        {0, 0}, {(int)(W - 1), (int)(H - 1)}, mrpt::img::TColor(0x55, 0x55, 0x55));
    picAlpha.filledRectangle(
        {0, 0}, {(int)(W / 4), (int)(H / 2)}, mrpt::img::TColor(0xa0, 0xa0, 0xa0));

    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 0, 0, 0, 90.0_deg, 0));
      obj->assignImage(pic);
      theScene->insert(obj);
    }

    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 4.0, 0, 0, 90.0_deg, 0));
      obj->assignImage(pic, picAlpha);
      theScene->insert(obj);
    }

    // a plane w/o a texture is a plain color plane:
    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 8.0, 0, 0, 90.0_deg, 0));
      obj->setColor_u8(0xff, 0x00, 0x00, 0xff);
      theScene->insert(obj);
    }
    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 12.0, 0, 0, 90.0_deg, 0));
      obj->setColor_u8(0xff, 0x00, 0x00, 0x40);
      theScene->insert(obj);
    }

    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 6.0, -3.0, 0, 0.0_deg, 0));
      obj->assignImage(pic);
      obj->enableLighting(true);
      theScene->insert(obj);
    }
    {
      viz::CTexturedPlane::Ptr obj = viz::CTexturedPlane::Create();
      obj->setPose(mrpt::poses::CPose3D(off_x, 3.9, -3.0, 0, 0.0_deg, 0));
      obj->setColor_u8(0xff, 0x00, 0x00, 0xff);
      obj->enableLighting(true);
      theScene->insert(obj);
    }

    auto gl_txt = viz::CText::Create("CTexturedPlane");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // COctoMapVoxels
  {
    mrpt::maps::COctoMap map(0.2);
    // Insert 2D scan:

    mrpt::obs::CObservation2DRangeScan scan1;
    mrpt::obs::stock_observations::example2DRangeScan(scan1);
    map.insertObservation(scan1);

    auto gl_map1 = mrpt::viz::COctoMapVoxels::Create();
    auto gl_map2 = mrpt::viz::COctoMapVoxels::Create();
    auto gl_map3 = mrpt::viz::COctoMapVoxels::Create();

    map.getAsOctoMapVoxels(*gl_map1);

    map.getAsOctoMapVoxels(*gl_map2);

    // map.renderingOptions.generateGridLines = true;
    map.getAsOctoMapVoxels(*gl_map3);

    gl_map1->showGridLines(false);
    gl_map1->showVoxelsAsPoints(true);
    gl_map1->setPointSize(3.0);
    gl_map1->setLocation(off_x, 0, 0);
    theScene->insert(gl_map1);

    gl_map2->showVoxelsAsPoints(false);
    gl_map2->showVoxels(VOXEL_SET_OCCUPIED, true);
    gl_map2->showVoxels(VOXEL_SET_FREESPACE, true);
    gl_map2->setLocation(off_x, 4, 0);
    theScene->insert(gl_map2);

    gl_map3->showVoxelsAsPoints(true);
    gl_map3->showGridLines(true);
    gl_map3->setLocation(off_x, 8, 0);
    theScene->insert(gl_map3);

    auto gl_txt = viz::CText::Create("COctoMapVoxels");
    gl_txt->setLocation(off_x, off_y_label, 0);
    theScene->insert(gl_txt);
  }
  off_x += STEP_X;

  // ground plane (to test shadows):
  // A plane w/o a texture is a plain color plane:
  {
    auto obj = mrpt::viz::CBox::Create();
    obj->setColor_u8(0xa0, 0xa0, 0xa0, 0xff);
    obj->setLocation(0, 0, -5.0f);
    obj->setBoxCorners({-20.0f, -20.0f, .0f}, {off_x + 20.f, 40.0f, -0.1f});
    obj->cullFaces(TCullFace::BACK);  // avoid z-fighting
    theScene->insert(obj);
  }

  // Arrow to show the light direction:
  auto glLightArrow =
      viz::CArrow::Create(mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(1, 0, 0));
  glLightArrow->setLocation(off_x / 5, -4.0, 5.0);
  glLightArrow->setColor(0, 1, 0);
  glLightArrow->setName("Light dir.");
  glLightArrow->enableShowName();
  theScene->insert(glLightArrow);

  // Add image-mode viewport:
  {
    const std::string img_file =
        mrpt::system::getShareMRPTDir() + "datasets/stereo-calib/0_left.jpg"s;

    mrpt::img::CImage im;
    if (im.loadFromFile(img_file))
    {
      auto glView = theScene->createViewport("image1");
      glView->setViewportPosition(0.7, 0, 0.3, 0.3);
      glView->setImageView(im);

      glView->setBorderSize(1);
    }
  }

  // Zoom out:
  win.setCameraZoom(250);

  // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
  win.unlockAccess3DScene();
  win.repaint();

  std::cout << "Close the window to end.\n";

  mrpt::viz::TFontParams fp;
  fp.vfont_scale = 14;  // pixels
  fp.draw_shadow = true;
  win.addTextMessage(5, 5, "", 0 /*id*/, fp);

  auto viewport = theScene->getViewport();

  mrpt::viz::TLightParameters& lights = viewport->lightParameters();

  lights.ambient = 0.2;

  while (win.isOpen())
  {
    // Lights:
    const double t = mrpt::Clock::nowDouble();
    const auto p = glLightArrow->getPose();

    const auto lightDir =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(p.x, p.y, p.z, t * 10.0_deg, 45.0_deg, 0.0_deg);

    glLightArrow->setPose(lightDir);

    lights.direction = lightDir.getRotationMatrix().extractColumn<mrpt::math::TVector3Df>(0);

    if (win.keyHit())
    {
      switch (win.getPushedKey())
      {
        case 'S':
        case 's':
          // toggle shadows:
          viewport->enableShadowCasting(!viewport->isShadowCastingEnabled());
          break;
      };
    }

    win.updateTextMessage(
        0 /*id*/, format(
                      "Render time=%.03fms | Shadows: %s", 1e3 / win.getRenderingFPS(),
                      viewport->isShadowCastingEnabled() ? "On" : "Off"));
    std::this_thread::sleep_for(2ms);
    win.repaint();
  }
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    TestOpenGLObjects();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
