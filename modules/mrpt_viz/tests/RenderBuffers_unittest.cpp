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

/** Checks that every visual object actually fills the CPU-side vertex buffers
 *  that the renderer later uploads to the GPU. An object whose updateBuffers()
 *  leaves them empty is invisible on screen, which is not something the
 *  offscreen-rendering tests can detect on a machine with no GL context.
 */

#include <gtest/gtest.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>

using namespace mrpt::viz;

TEST(RenderBuffers, LineObjectsFillTheLinesBuffer)
{
  {
    auto o = CSetOfLines::Create();
    o->appendLine(0, 0, 0, 1, 1, 1);
    o->appendLine(1, 1, 1, 2, 0, 0);
    o->updateBuffers();
    // Two vertices per segment:
    EXPECT_EQ(o->shaderLinesVertexPointBuffer().size(), 4U);
    EXPECT_EQ(o->shaderLinesVertexColorBuffer().size(), 4U);
    // Vertex dots are off by default for this class:
    EXPECT_TRUE(o->shaderPointsVertexPointBuffer().empty());

    o->setVerticesPointSize(4.0f);
    o->updateBuffers();
    EXPECT_EQ(o->shaderPointsVertexPointBuffer().size(), 4U);

    o->clear();
    o->updateBuffers();
    EXPECT_TRUE(o->shaderLinesVertexPointBuffer().empty());
  }
  {
    auto o = CSimpleLine::Create(0, 0, 0, 1, 2, 3);
    o->updateBuffers();
    ASSERT_EQ(o->shaderLinesVertexPointBuffer().size(), 2U);
    EXPECT_EQ(o->shaderLinesVertexPointBuffer()[1].y, 2.0f);
    EXPECT_EQ(o->shaderLinesVertexColorBuffer().size(), 2U);

    // Both setter overloads must invalidate the cached geometry:
    o->setLineCoords({0, 0, 0}, {4, 5, 6});
    o->updateBuffers();
    EXPECT_EQ(o->shaderLinesVertexPointBuffer()[1].y, 5.0f);
    EXPECT_NEAR(o->getBoundingBoxLocal().max.z, 6.0, 1e-5);
  }
  {
    auto o = CAxis::Create(-1, -1, -1, 1, 1, 1, 1.0f, 2.0f, true);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
  {
    auto o = CGridPlaneXY::Create(-1, 1, -1, 1, 0, 0.5f);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
  {
    auto o = CGridPlaneXZ::Create(-1, 1, -1, 1, 0, 0.5f);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
}

TEST(RenderBuffers, SolidObjectsFillTheTrianglesBuffer)
{
  {
    auto o = CDisk::Create(1.0f, 0.0f, 20);
    o->updateBuffers();
    // A filled disk is a triangle fan: one triangle per slice.
    EXPECT_EQ(o->shaderTrianglesBuffer().size(), 20U);

    // Fewer than 3 slices cannot make a disk, and must be rejected where the
    // mistake is made rather than at render time:
    EXPECT_THROW(o->setSlicesCount(2), std::exception);
    EXPECT_THROW(CDisk::Create(1.0f, 0.0f, 1), std::exception);
  }
  {
    // The ring case takes two triangles per slice:
    auto o = CDisk::Create(1.0f, 0.5f, 20);
    o->updateBuffers();
    EXPECT_EQ(o->shaderTrianglesBuffer().size(), 40U);
  }
  {
    auto o = CSphere::Create(1.0f);
    o->updateBuffers();
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
  {
    auto o = CBox::Create(mrpt::math::TPoint3D(-1, -1, -1), mrpt::math::TPoint3D(1, 1, 1));
    o->updateBuffers();
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
  {
    auto o = CCylinder::Create(0.5f, 0.5f, 2.0f, 12);
    o->updateBuffers();
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
  {
    auto o = CArrow::Create(mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(0, 0, 1));
    o->updateBuffers();
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
}

TEST(RenderBuffers, FrustumHonorsTheDrawModeFlags)
{
  {
    auto o = CFrustum::Create(0.3f, 1.5f, 60.0f, 40.0f, 2.0f, true /*lines*/, true /*planes*/);
    o->updateBuffers();
    // A frustum is a box: 12 edges, each emitted exactly once as a vertex pair
    EXPECT_EQ(o->shaderLinesVertexPointBuffer().size(), 24U);
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
  {
    // Wireframe only (the default for the empty constructor):
    auto o = CFrustum::Create(0.3f, 1.5f, 60.0f, 40.0f, 2.0f, true /*lines*/, false /*planes*/);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
    EXPECT_TRUE(o->shaderTrianglesBuffer().empty());
  }
  {
    auto o = CFrustum::Create(0.3f, 1.5f, 60.0f, 40.0f, 2.0f, false /*lines*/, true /*planes*/);
    o->updateBuffers();
    EXPECT_TRUE(o->shaderLinesVertexPointBuffer().empty());
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
}

TEST(RenderBuffers, PointObjectsFillThePointsBuffer)
{
  {
    auto o = CPointCloud::Create();
    o->insertPoint(0, 0, 0);
    o->insertPoint(1, 1, 1);
    o->updateBuffers();
    EXPECT_EQ(o->shaderPointsVertexPointBuffer().size(), 2U);
  }
  {
    auto o = CPointCloudColoured::Create();
    o->push_back(0, 0, 0, 1, 0, 0);
    o->push_back(1, 1, 1, 0, 1, 0);
    o->updateBuffers();
    EXPECT_EQ(o->shaderPointsVertexPointBuffer().size(), 2U);
    EXPECT_EQ(o->shaderPointsVertexColorBuffer().size(), 2U);
  }
}

TEST(RenderBuffers, MeshObjectsFillTheirBuffers)
{
  mrpt::math::CMatrixFloat Z(4, 5);
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 5; c++)
    {
      Z(r, c) = 0.1f * static_cast<float>(r * c);
    }
  }

  {
    auto o = CMesh::Create(false, -1, 1, -1, 1);
    o->setZ(Z);
    o->updateBuffers();
    // (rows-1) x (cols-1) quads, two triangles each:
    EXPECT_EQ(o->shaderTexturedTrianglesBuffer().size(), 2U * 3U * 4U);
  }
  {
    auto o = CMeshFast::Create(false, -1, 1, -1, 1);
    o->setZ(Z);
    o->updateBuffers();
    EXPECT_EQ(o->shaderPointsVertexPointBuffer().size(), 20U);
  }
  {
    // A single quad face:
    auto o = CMesh3D::Create();
    std::vector<int> vertsPerFace = {4};
    std::vector<int> faceVerts = {0, 1, 2, 3};
    std::vector<float> coords = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0};
    o->loadMesh(4, 1, vertsPerFace.data(), faceVerts.data(), coords.data());
    o->updateBuffers();
    // A quad is split into two triangles, and its 4 edges into 4 line pairs:
    EXPECT_EQ(o->shaderTrianglesBuffer().size(), 2U);
    EXPECT_EQ(o->shaderLinesVertexPointBuffer().size(), 8U);

    o->enableShowFaces(false);
    o->enableShowEdges(false);
    o->updateBuffers();
    EXPECT_TRUE(o->shaderTrianglesBuffer().empty());
    EXPECT_TRUE(o->shaderLinesVertexPointBuffer().empty());
  }
}

TEST(RenderBuffers, VectorFieldsFillTheirBuffers)
{
  {
    auto o = CVectorField2D::Create();
    mrpt::math::CMatrixFloat vx(3, 3);
    mrpt::math::CMatrixFloat vy(3, 3);
    vx.fill(0.1f);
    vy.fill(0.2f);
    o->setVectorField(vx, vy);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
    EXPECT_GT(o->shaderPointsVertexPointBuffer().size(), 0U);
  }
  {
    auto o = CVectorField3D::Create();
    mrpt::math::CMatrixFloat m(2, 2);
    m.fill(0.5f);
    o->setPointCoordinates(m, m, m);
    o->setVectorField(m, m, m);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
}

TEST(RenderBuffers, OtherObjectsFillTheirBuffers)
{
  {
    auto o = CTexturedPlane::Create(-1, 1, -1, 1);
    o->updateBuffers();
    EXPECT_GT(o->shaderTexturedTrianglesBuffer().size(), 0U);
  }
  {
    auto o = CColorBar::Create(mrpt::img::cmJET, 0.3, 1.2, 0.0f, 1.0f, 0.0f, 1.0f, "%4.1f", 0.08f);
    o->updateBuffers();
    EXPECT_GT(o->shaderTrianglesBuffer().size(), 0U);
  }
  {
    auto o = CEllipsoid2D::Create();
    mrpt::math::CMatrixDouble22 cov;
    cov.setIdentity();
    o->setCovMatrix(cov);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
  {
    // 3D ellipsoids are drawn as a wireframe mesh:
    auto o = CEllipsoid3D::Create();
    mrpt::math::CMatrixDouble33 cov;
    cov.setIdentity();
    o->setCovMatrix(cov);
    o->updateBuffers();
    EXPECT_GT(o->shaderLinesVertexPointBuffer().size(), 0U);
  }
  {
    auto o = CPolyhedron::CreateHexahedron(1.0);
    o->updateBuffers();
    // A cube: 6 quad faces, 2 triangles each, and no edges unless asked for:
    EXPECT_EQ(o->shaderTrianglesBuffer().size(), 12U);
    EXPECT_TRUE(o->shaderLinesVertexPointBuffer().empty());

    o->setWireframe(true);
    o->updateBuffers();
    EXPECT_EQ(o->shaderLinesVertexPointBuffer().size(), 24U);
  }
}
