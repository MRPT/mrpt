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

/** Drives the backwards-compatibility branches of the mrpt::viz classes'
 *  serializeFrom(), which are only reachable by streams written by older MRPT
 *  versions. See tests/legacy_serialization.h for how the frames are built.
 */

#include "legacy_serialization.h"

#include <gtest/gtest.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Viewport.h>

using namespace mrpt::viz;
using mrpt_test::writeLegacyObjectFrame;
using mrpt_test::writeLegacyRenderHeader;

namespace
{
/** Reads a legacy frame back into `obj`. */
template <class T>
void readLegacy(
    T& obj,
    const std::string& className,
    uint8_t version,
    const std::function<void(mrpt::serialization::CArchive&)>& writePayload)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, className, version, writePayload);
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch.ReadObject(&obj);
}
}  // namespace

// ------------------------------------------- the shared render header itself

TEST(VizLegacySerialization, RenderHeaderVersions)
{
  // Every viz class starts with CVisualObject::writeToStreamRender(), which
  // carries its own version, independent of the class's. Reading each of them
  // must leave the fields the older format lacked at their documented
  // defaults.
  for (uint8_t hdrVer = 0; hdrVer <= 4; hdrVer++)
  {
    CSphere o;
    readLegacy(
        o, "mrpt::viz::CSphere", 3,
        [hdrVer](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a, hdrVer, "sph", mrpt::img::TColor(10, 20, 30, 40));
          a << 2.5f;                // m_radius
          a.WriteAs<uint32_t>(16);  // m_nDivs
        });

    EXPECT_EQ(o.getName(), "sph") << "header v" << int(hdrVer);
    EXPECT_EQ(o.getColor_u8().R, 10) << "header v" << int(hdrVer);
    EXPECT_FLOAT_EQ(o.getRadius(), 2.5f) << "header v" << int(hdrVer);

    if (hdrVer < 1)
    {
      EXPECT_EQ(o.getLocalRepresentativePoint(), mrpt::math::TPoint3Df(0, 0, 0));
    }
    if (hdrVer < 2)
    {
      EXPECT_FLOAT_EQ(o.materialShininess(), 0.2f);
      EXPECT_TRUE(o.castShadows());
    }
    if (hdrVer < 3)
    {
      EXPECT_FLOAT_EQ(o.materialSpecularExponent(), 32.0f);
    }
    if (hdrVer < 4)
    {
      EXPECT_FLOAT_EQ(o.materialEmissive().R, 0.0f);
    }
  }
}

TEST(VizLegacySerialization, RenderHeaderUnknownVersionThrows)
{
  CSphere o;
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::viz::CSphere", 3,
      [](mrpt::serialization::CArchive& a)
      {
        a << static_cast<uint8_t>(0xFF);
        // A render-header version from a hypothetical newer MRPT:
        a << static_cast<uint8_t>(31 | 0xC0);
      });
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(&o), std::exception);
}

TEST(VizLegacySerialization, RenderHeaderTooOldFormatThrows)
{
  // Anything without the 0xFF/bit7 magic is a pre-0.9.5 stream:
  CSphere o;
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::viz::CSphere", 3,
      [](mrpt::serialization::CArchive& a)
      {
        a << static_cast<uint8_t>(0x00);
        a << static_cast<uint8_t>(0x00);
      });
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(&o), std::exception);
}

// ------------------------------------------------------------------ CSphere

TEST(VizLegacySerialization, CSphere)
{
  // v0 and v2 carry a now-unused uint32; v1 additionally carries a bool.
  for (uint8_t v = 0; v <= 3; v++)
  {
    CSphere o;
    readLegacy(
        o, "mrpt::viz::CSphere", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          a << 3.5f;
          a.WriteAs<uint32_t>(24);
          if (v < 3)
          {
            a.WriteAs<uint32_t>(0);  // dummy, dropped in v3
          }
          if (v == 1)
          {
            a << true;  // keepRadiusIndependentEyeDistance, dropped after v1
          }
        });

    EXPECT_FLOAT_EQ(o.getRadius(), 3.5f) << "v" << int(v);
    // The division count has no getter; check it reached the geometry:
    o.updateBuffers();
    EXPECT_GT(o.shaderTrianglesBuffer().size(), 0U) << "v" << int(v);
  }
}

// --------------------------------------------------------------------- CBox

TEST(VizLegacySerialization, CBox)
{
  for (uint8_t v = 0; v <= 3; v++)
  {
    CBox o;
    readLegacy(
        o, "mrpt::viz::CBox", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          a << -1.0 << -2.0 << -3.0 << 1.0 << 2.0 << 3.0;  // corners (doubles)
          a << true;                                       // m_wireframe
          if (v >= 1)
          {
            a << true;                           // m_draw_border
            a << mrpt::img::TColor(1, 2, 3, 4);  // m_solidborder_color
          }
          if (v >= 2)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Triangles version
            a << true;              // m_enableLight
            a.WriteAs<uint8_t>(0);  // m_cullface
          }
          if (v >= 3)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Lines version
            a << 1.0f;              // line width
            a << true;              // antialiasing
          }
        });

    mrpt::math::TPoint3D c1;
    mrpt::math::TPoint3D c2;
    o.getBoxCorners(c1, c2);
    EXPECT_NEAR(c1.x, -1.0, 1e-6) << "v" << int(v);
    EXPECT_NEAR(c2.z, 3.0, 1e-6) << "v" << int(v);
    EXPECT_TRUE(o.isWireframe()) << "v" << int(v);
    // The border flag did not exist before v1 and must default to off:
    EXPECT_EQ(o.isBoxBorderEnabled(), v >= 1) << "v" << int(v);
  }
}

// -------------------------------------------------------------- CSetOfLines

TEST(VizLegacySerialization, CSetOfLines)
{
  // v0/v1 stored six parallel coordinate vectors instead of TSegment3D's.
  for (uint8_t v = 0; v <= 1; v++)
  {
    CSetOfLines o;
    readLegacy(
        o, "mrpt::viz::CSetOfLines", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          // Note: a braced list would select CVectorFloat's size ctor.
          auto vec = [](float a0, float a1)
          {
            mrpt::math::CVectorFloat v(2);
            v[0] = a0;
            v[1] = a1;
            return v;
          };
          a << vec(0.f, 1.f) << vec(0.f, 1.f) << vec(0.f, 1.f);
          a << vec(10.f, 11.f) << vec(20.f, 21.f) << vec(30.f, 31.f);
          if (v >= 1)
          {
            a << 3.0f;  // line width
          }
        });

    ASSERT_EQ(o.size(), 2U) << "v" << int(v);
    double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
    o.getLineByIndex(1, x0, y0, z0, x1, y1, z1);
    EXPECT_NEAR(x0, 1.0, 1e-5) << "v" << int(v);
    EXPECT_NEAR(z1, 31.0, 1e-5) << "v" << int(v);
    if (v >= 1)
    {
      EXPECT_FLOAT_EQ(o.getLineWidth(), 3.0f);
    }

    // The restored segments must reach the render buffers:
    o.updateBuffers();
    EXPECT_EQ(o.shaderLinesVertexPointBuffer().size(), 4U) << "v" << int(v);
  }

  // v2..v4 store the segments directly.
  for (uint8_t v = 2; v <= 4; v++)
  {
    CSetOfLines o;
    readLegacy(
        o, "mrpt::viz::CSetOfLines", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          const std::vector<mrpt::math::TSegment3D> segs{
              {{0, 0, 0}, {1, 2, 3}},
              {{1, 2, 3}, {4, 5, 6}}
          };
          a << segs;
          a << 2.5f;  // line width
          if (v >= 3)
          {
            a << true;  // antialiasing
          }
          if (v >= 4)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Points version
            a << 4.0f;              // point size
            a << false;             // variable point size
            a << 0.0f << 0.0f;      // the two K params
          }
        });

    ASSERT_EQ(o.size(), 2U) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getLineWidth(), 2.5f) << "v" << int(v);
    // Vertex dots did not exist before v4, and must stay off:
    EXPECT_FLOAT_EQ(o.getVerticesPointSize(), v >= 4 ? 4.0f : 0.0f) << "v" << int(v);
  }
}

// ------------------------------------------------------------- CPointCloud

TEST(VizLegacySerialization, CPointCloudCoordinateVectorFormats)
{
  // Before v5 the points were three parallel float vectors; from v3 the
  // "color from axis" selector became an int32 instead of a bool.
  for (uint8_t v = 0; v <= 4; v++)
  {
    CPointCloud o;
    readLegacy(
        o, "mrpt::viz::CPointCloud", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          if (v >= 3)
          {
            a << static_cast<int32_t>(1);  // colZ
          }
          else
          {
            a << true;  // colorFromZ
          }
          const std::vector<float> xs{1.f, 2.f, 3.f};
          const std::vector<float> ys{4.f, 5.f, 6.f};
          const std::vector<float> zs{7.f, 8.f, 9.f};
          a << xs << ys << zs;
          if (v >= 1)
          {
            a << 5.0f;  // point size
          }
          if (v >= 2)
          {
            a << 0.0f << 0.0f << 0.0f;   // colorFromDepth_min RGB
            a << 1.0f << 0.5f << 0.25f;  // colorFromDepth_max RGB
          }
          if (v >= 4)
          {
            a << true;  // point smoothing, ignored since v7
          }
        });

    ASSERT_EQ(o.size(), 3U) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getPoint3Df(2).x, 3.0f) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getPoint3Df(0).z, 7.0f) << "v" << int(v);
    if (v >= 1)
    {
      EXPECT_FLOAT_EQ(o.getPointSize(), 5.0f) << "v" << int(v);
    }

    // The gradient endpoints only exist from v2 on:
    o.updateBuffers();
    ASSERT_EQ(o.shaderPointsVertexColorBuffer().size(), 3U) << "v" << int(v);
  }
}

TEST(VizLegacySerialization, CPointCloudPackedPointFormats)
{
  // v5 and v6 store a packed count + TPoint3Df array.
  for (uint8_t v = 5; v <= 7; v++)
  {
    CPointCloud o;
    readLegacy(
        o, "mrpt::viz::CPointCloud", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          a << static_cast<int32_t>(0);  // colNone
          a.WriteAs<uint32_t>(2);
          a << mrpt::math::TPoint3Df(1, 2, 3) << mrpt::math::TPoint3Df(4, 5, 6);
          if (v < 6)
          {
            a << 7.0f;  // point size, moved into the params blob in v6
          }
          a << 0.0f << 0.0f << 0.0f;
          a << 1.0f << 1.0f << 1.0f;
          if (v < 7)
          {
            a << true;  // point smoothing
          }
          if (v >= 6)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Points version
            a << 7.0f;              // point size
            a << false;
            a << 0.0f << 0.0f;
          }
        });

    ASSERT_EQ(o.size(), 2U) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getPoint3Df(1).y, 5.0f) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getPointSize(), 7.0f) << "v" << int(v);
  }
}

TEST(VizLegacySerialization, UnknownFutureVersionThrows)
{
  CSphere o;
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "mrpt::viz::CSphere", 99,
      [](mrpt::serialization::CArchive& a) { writeLegacyRenderHeader(a); });
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(&o), std::exception);
}

// ----------------------------------------------------------------- Viewport

namespace
{
/** Writes a Viewport payload as MRPT <= `version` wrote it. */
void writeLegacyViewportPayload(mrpt::serialization::CArchive& a, uint8_t version)
{
  CCamera cam;
  cam.setZoomDistance(17.0f);
  a << cam;

  a << false;                     // m_isCloned
  a << false;                     // m_isClonedCamera
  a << std::string("");           // m_clonedViewport
  a << std::string("legacy_vp");  // m_name
  a << true;                      // m_isTransparent
  a << static_cast<uint32_t>(3);  // m_borderWidth
  a << 0.1 << 0.2 << 0.7 << 0.8;  // view x,y,w,h

  if (version >= 1)
  {
    if (version < 7)
    {
      a << true;  // the "has custom background color" flag, removed in v7
    }
    a << 0.25f << 0.5f << 0.75f << 1.0f;  // background RGBA
  }

  a.WriteAs<uint32_t>(0);  // no child objects

  if (version >= 2)
  {
    a << true;  // m_OpenGL_enablePolygonNicest
  }
  if (version >= 3)
  {
    a << TLightParameters();
  }
  if (version >= 4)
  {
    a.WriteAs<uint32_t>(0);  // no 2D text messages
  }
  if (version >= 5)
  {
    a.WriteAs<bool>(false);  // no image-view plane
  }
  if (version >= 6)
  {
    a << std::string("");  // m_clonedCameraViewport
  }
  if (version >= 8)
  {
    a << true;                         // m_shadowsEnabled
    a << static_cast<uint32_t>(1024);  // m_ShadowMapSizeX
    a << static_cast<uint32_t>(512);   // m_ShadowMapSizeY
  }
  if (version >= 9)
  {
    a << 500.0f << 0.05f << 0.06f << 400.0f;  // clip max,min + light shadow min,max
  }
  if (version >= 10)
  {
    a << false;  // m_isViewportVisible
  }
}
}  // namespace

TEST(VizLegacySerialization, Viewport)
{
  for (uint8_t v = 0; v <= 10; v++)
  {
    Viewport o;
    readLegacy(
        o, "mrpt::viz::Viewport", v,
        [v](mrpt::serialization::CArchive& a) { writeLegacyViewportPayload(a, v); });

    EXPECT_EQ(o.getName(), "legacy_vp") << "v" << int(v);
    EXPECT_TRUE(o.isTransparent()) << "v" << int(v);
    EXPECT_EQ(o.getBorderSize(), 3U) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getCamera().getZoomDistance(), 17.0f) << "v" << int(v);

    double x = 0;
    double y = 0;
    double w = 0;
    double h = 0;
    o.getViewportPosition(x, y, w, h);
    EXPECT_NEAR(x, 0.1, 1e-9) << "v" << int(v);
    EXPECT_NEAR(h, 0.8, 1e-9) << "v" << int(v);

    // Fields introduced later must land on their documented defaults:
    if (v >= 8)
    {
      EXPECT_TRUE(o.isShadowCastingEnabled()) << "v" << int(v);
    }
    if (v >= 9)
    {
      float cmin = 0;
      float cmax = 0;
      o.getViewportClipDistances(cmin, cmax);
      EXPECT_FLOAT_EQ(cmin, 0.05f);
      EXPECT_FLOAT_EQ(cmax, 500.0f);
    }
    EXPECT_EQ(o.getViewportVisibility(), v < 10) << "v" << int(v);
  }
}

// ------------------------------------------------------------------ CCamera

TEST(VizLegacySerialization, CCamera)
{
  // Up to v4 CCamera wrote no CVisualObject header at all; v5 added it (and
  // the 6-DOF flag) at the front.
  for (uint8_t v = 1; v <= 5; v++)
  {
    CCamera o;
    readLegacy(
        o, "mrpt::viz::CCamera", v,
        [v](mrpt::serialization::CArchive& a)
        {
          if (v >= 5)
          {
            writeLegacyRenderHeader(a);
          }
          a << 1.0f << 2.0f << 3.0f;  // pointing at x,y,z
          a << 12.0f;                 // eye distance
          a << 30.0f << 40.0f;        // azimuth, elevation
          a << true;                  // projective model
          a << 55.0f;                 // projective FOV
          if (v >= 2)
          {
            const std::optional<mrpt::img::TCamera> noPinhole;
            a << noPinhole;
          }
          if (v >= 3)
          {
            a << false;  // m_useNoProjection
          }
          if (v >= 4)
          {
            a << 9.0f;  // eye roll
          }
          if (v >= 5)
          {
            a << true;  // m_6DOFMode
          }
        });

    EXPECT_FLOAT_EQ(o.getPointingAtY(), 2.0f) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getZoomDistance(), 12.0f) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getProjectiveFOVdeg(), 55.0f) << "v" << int(v);
    EXPECT_TRUE(o.isProjective()) << "v" << int(v);

    // Fields added later must fall back to their defaults:
    EXPECT_EQ(o.hasPinholeModel(), false) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getRollDegrees(), v >= 4 ? 9.0f : 0.0f) << "v" << int(v);
    EXPECT_EQ(o.is6DOFMode(), v >= 5) << "v" << int(v);
  }
}

TEST(VizLegacySerialization, CCameraVersion0)
{
  // v0 carried only the six orbit parameters.
  CCamera o;
  readLegacy(
      o, "mrpt::viz::CCamera", 0,
      [](mrpt::serialization::CArchive& a)
      { a << 1.0f << 2.0f << 3.0f << 12.0f << 30.0f << 40.0f; });

  EXPECT_FLOAT_EQ(o.getPointingAtZ(), 3.0f);
  EXPECT_FLOAT_EQ(o.getAzimuthDegrees(), 30.0f);
  EXPECT_FALSE(o.is6DOFMode());
}

// -------------------------------------------------- CPointCloudColoured

TEST(VizLegacySerialization, CPointCloudColouredDroppedVersions)
{
  // v0..v3 are explicitly no longer readable; that must be a clean throw
  // rather than a stream desync.
  for (uint8_t v = 0; v <= 3; v++)
  {
    CPointCloudColoured o;
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "mrpt::viz::CPointCloudColoured", v,
        [](mrpt::serialization::CArchive& a) { writeLegacyRenderHeader(a); });
    auto arch = mrpt::serialization::archiveFrom(buf);
    EXPECT_THROW(arch.ReadObject(&o), std::exception) << "v" << int(v);
  }
}

TEST(VizLegacySerialization, CPointCloudColouredV4)
{
  CPointCloudColoured o;
  readLegacy(
      o, "mrpt::viz::CPointCloudColoured", 4,
      [](mrpt::serialization::CArchive& a)
      {
        writeLegacyRenderHeader(a);
        const std::vector<mrpt::math::TPoint3Df> pts{
            {1, 2, 3},
            {4, 5, 6}
        };
        const std::vector<mrpt::img::TColor> cols{
            mrpt::img::TColor(255, 0, 0, 255), mrpt::img::TColor(0, 255, 0, 255)};
        a << pts << cols;
        a.WriteAs<uint8_t>(0);  // VisualObjectParams_Points version
        a << 6.0f;              // point size
        a << false;
        a << 0.0f << 0.0f;
      });

  ASSERT_EQ(o.size(), 2U);
  EXPECT_FLOAT_EQ(o.getPoint3Df(1).z, 6.0f);
  EXPECT_EQ(o.getPointColor(0).R, 255);
  EXPECT_FLOAT_EQ(o.getPointSize(), 6.0f);
}

// ------------------------------------------------------------------- CArrow

TEST(VizLegacySerialization, CArrow)
{
  for (uint8_t v = 0; v <= 3; v++)
  {
    CArrow o;
    readLegacy(
        o, "mrpt::viz::CArrow", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          a << 0.0f << 0.0f << 0.0f;   // from
          a << 1.0f << 2.0f << 3.0f;   // to
          a << 0.3f << 0.07f << 0.2f;  // head ratio, small/large radius
          if (v == 1)
          {
            a << 0.0f << 0.0f << 0.0f;  // roll/pitch/yaw, dropped after v1
          }
          if (v >= 2)
          {
            a << static_cast<uint32_t>(15);  // m_slices
          }
          if (v >= 3)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Triangles version
            a << true;              // m_enableLight
            a.WriteAs<uint8_t>(0);  // m_cullface
          }
        });

    const auto bb = o.getBoundingBoxLocal();
    EXPECT_NEAR(bb.max.z, 3.0, 1e-4) << "v" << int(v);
    o.updateBuffers();
    EXPECT_GT(o.shaderTrianglesBuffer().size(), 0U) << "v" << int(v);
  }
}

// -------------------------------------------------------------------- CAxis

TEST(VizLegacySerialization, CAxis)
{
  for (uint8_t v = 0; v <= 3; v++)
  {
    CAxis o;
    readLegacy(
        o, "mrpt::viz::CAxis", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          a << -1.0f << -2.0f << -3.0f;  // min
          a << 1.0f << 2.0f << 3.0f;     // max
          a << 0.5f;                     // frequency
          if (v >= 1)
          {
            a << true << false << true;  // per-axis tick marks
            a << 0.4f;                   // text scale
            for (int i = 0; i < 3; i++)
            {
              for (int j = 0; j < 3; j++)
              {
                a << 0.0f;  // text rotations
              }
            }
          }
          else
          {
            a << true;  // the single pre-v1 "marks" flag
          }
          if (v >= 2)
          {
            a << 0.1f;  // m_markLen
          }
          if (v >= 3)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Lines version
            a << 2.0f;              // line width
            a << true;              // antialiasing
          }
        });

    EXPECT_FLOAT_EQ(o.getFrequency(), 0.5f) << "v" << int(v);
    const auto bb = o.getBoundingBoxLocal();
    EXPECT_NEAR(bb.min.x, -1.0, 1e-4) << "v" << int(v);

    // v0 had a single flag for all three axes, and a fixed text scale:
    EXPECT_FLOAT_EQ(o.getTextScale(), v >= 1 ? 0.4f : 0.25f) << "v" << int(v);

    o.updateBuffers();
    EXPECT_GT(o.shaderLinesVertexPointBuffer().size(), 0U) << "v" << int(v);
  }
}

// ----------------------------------------------------------- COctoMapVoxels

TEST(VizLegacySerialization, COctoMapVoxels)
{
  for (uint8_t v = 0; v <= 4; v++)
  {
    COctoMapVoxels o;
    // A non-default colormap, to prove an old stream resets it:
    o.colorMap(mrpt::img::cmJET);

    readLegacy(
        o, "mrpt::viz::COctoMapVoxels", v,
        [v](mrpt::serialization::CArchive& a)
        {
          writeLegacyRenderHeader(a);
          // Empty STL containers, written the way stl_serialization does:
          // container name, element type name, then the count.
          a << std::string("std::deque") << std::string("COctoMapVoxels::TInfoPerVoxelSet");
          a.WriteAs<uint32_t>(0);  // m_voxel_sets
          a << std::string("std::vector") << std::string("COctoMapVoxels::TGridCube");
          a.WriteAs<uint32_t>(0);  // m_grid_cubes
          a << mrpt::math::TPoint3D(-1, -1, -1) << mrpt::math::TPoint3D(2, 2, 2);
          a << true;   // m_enable_lighting
          a << false;  // m_showVoxelsAsPoints
          a << 3.0f;   // m_showVoxelsAsPointsSize
          a << true;   // m_show_grids
          a << 1.5f;   // m_grid_width
          a << mrpt::img::TColor(9, 8, 7, 255);
          if (v >= 1)
          {
            a << true;  // m_enable_cube_transparency
          }
          if (v >= 2)
          {
            a << static_cast<uint32_t>(0);  // visualization mode
          }
          if (v >= 3)
          {
            a.WriteAs<uint8_t>(0);  // VisualObjectParams_Triangles version
            a << true;
            a.WriteAs<uint8_t>(0);
          }
          if (v >= 4)
          {
            a.WriteAs<uint8_t>(static_cast<uint8_t>(mrpt::img::cmGRAYSCALE));
          }
        });

    EXPECT_TRUE(o.areGridLinesVisible()) << "v" << int(v);
    EXPECT_FLOAT_EQ(o.getGridLinesWidth(), 1.5f) << "v" << int(v);
    EXPECT_EQ(o.getGridLinesColor().R, 9) << "v" << int(v);
    EXPECT_EQ(o.isCubeTransparencyEnabled(), v >= 1) << "v" << int(v);

    const auto bb = o.getBoundingBoxLocal();
    EXPECT_NEAR(bb.max.y, 2.0, 1e-4) << "v" << int(v);

    // The colormap only exists from v4 on; older streams must restore the
    // default rather than keep whatever the object had:
    EXPECT_EQ(o.colorMap(), v >= 4 ? mrpt::img::cmGRAYSCALE : mrpt::img::cmHOT) << "v" << int(v);
  }
}
