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

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/config.h>  // MRPT_HAS_ASSIMP

#include <fstream>

#if MRPT_HAS_ASSIMP

using namespace mrpt::viz;

namespace
{
/** A scratch directory, unique per test, holding the synthetic model files.
 *  Removed by the destructor. */
class TempModelDir
{
 public:
  TempModelDir() : m_dir(mrpt::system::getTempFileName() + "_assimp")
  {
    mrpt::system::createDirectory(m_dir);
  }
  ~TempModelDir() { mrpt::system::deleteFilesInDirectory(m_dir, true); }

  TempModelDir(const TempModelDir&) = delete;
  TempModelDir& operator=(const TempModelDir&) = delete;

  [[nodiscard]] std::string path(const std::string& fileName) const
  {
    return m_dir + std::string("/") + fileName;
  }

  void writeTextFile(const std::string& fileName, const std::string& contents) const
  {
    std::ofstream f(path(fileName));
    f << contents;
  }

 private:
  std::string m_dir;
};

/** A single triangle, no material file at all. */
constexpr const char* OBJ_BARE_TRIANGLE =
    "v 0 0 0\n"
    "v 1 0 0\n"
    "v 0 1 0\n"
    "f 1 2 3\n";

/** Two triangles forming a unit square in the XY plane, with a material that
 *  has a diffuse color and a diffuse texture map. */
constexpr const char* OBJ_TEXTURED_QUAD =
    "mtllib quad.mtl\n"
    "v 0 0 0\n"
    "v 1 0 0\n"
    "v 1 1 0\n"
    "v 0 1 0\n"
    "vt 0 0\n"
    "vt 1 0\n"
    "vt 1 1\n"
    "vt 0 1\n"
    "vn 0 0 1\n"
    "usemtl painted\n"
    "f 1/1/1 2/2/1 3/3/1\n"
    "f 1/1/1 3/3/1 4/4/1\n";

constexpr const char* MTL_TEXTURED =
    "newmtl painted\n"
    "Kd 1.0 0.5 0.25\n"
    "Ks 0.5 0.5 0.5\n"
    "Ke 0.1 0.1 0.1\n"
    "Ns 42.0\n"
    "map_Kd texture.png\n";

/** Same geometry, but the referenced texture file does not exist. */
constexpr const char* MTL_MISSING_TEXTURE =
    "newmtl painted\n"
    "Kd 0.2 0.4 0.6\n"
    "map_Kd this_file_does_not_exist.png\n";

void writeTestTexture(const std::string& fileName)
{
  mrpt::img::CImage img(8, 4, mrpt::img::CH_RGB);
  for (int y = 0; y < 4; y++)
  {
    for (int x = 0; x < 8; x++)
    {
      img.setPixel(
          {x, y}, mrpt::img::TColor(
                      static_cast<uint8_t>(x * 30), static_cast<uint8_t>(y * 60), uint8_t(128)));
    }
  }
  ASSERT_(img.saveToFile(fileName));
}
}  // namespace

TEST(CAssimpModel, EmptyModelDefaults)
{
  auto m = CAssimpModel::Create();
  EXPECT_TRUE(m->getModelPath().empty());
  EXPECT_EQ(m->getModelLoadFlags(), 0U);
  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_EQ(m->getNonTexturedTriangleCount(), 0U);
  EXPECT_EQ(m->getTotalTriangleCount(), 0U);
  EXPECT_EQ(m->getTotalVertexCount(), 0U);
  EXPECT_TRUE(m->getTextureInfo().empty());
  EXPECT_FLOAT_EQ(m->getSplitTrianglesRenderingBBox(), 0.0f);

  // No geometry: ray tracing must simply report "no hit".
  double dist = 0;
  EXPECT_FALSE(m->traceRay(mrpt::poses::CPose3D::Identity(), dist));
}

TEST(CAssimpModel, LoadNonTexturedTriangle)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);

  EXPECT_EQ(m->getModelPath(), dir.path("tri.obj"));
  EXPECT_EQ(m->getModelLoadFlags(), uint32_t(CAssimpModel::LoadFlags::RealTimeFast));

  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_EQ(m->getNonTexturedTriangleCount(), 1U);
  EXPECT_EQ(m->getTotalTriangleCount(), 1U);
  EXPECT_EQ(m->getTotalVertexCount(), 3U);
  EXPECT_TRUE(m->getTextureInfo().empty());

  const auto bb = m->getBoundingBoxLocal();
  EXPECT_NEAR(bb.min.x, 0.0, 1e-4);
  EXPECT_NEAR(bb.min.y, 0.0, 1e-4);
  EXPECT_NEAR(bb.max.x, 1.0, 1e-4);
  EXPECT_NEAR(bb.max.y, 1.0, 1e-4);
}

TEST(CAssimpModel, LoadTexturedQuad)
{
  const TempModelDir dir;
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_TEXTURED);
  writeTestTexture(dir.path("texture.png"));

  auto m = CAssimpModel::Create();
  m->loadScene(
      dir.path("quad.obj"),
      CAssimpModel::LoadFlags::RealTimeQuality | CAssimpModel::LoadFlags::FlipUVs);

  ASSERT_EQ(m->getTexturedMeshCount(), 1U);
  EXPECT_EQ(m->getNonTexturedTriangleCount(), 0U);
  EXPECT_EQ(m->getTotalTriangleCount(), 2U);
  EXPECT_EQ(m->getTotalVertexCount(), 6U);

  const auto texInfo = m->getTextureInfo();
  ASSERT_EQ(texInfo.size(), 1U);
  EXPECT_EQ(texInfo[0].triangleCount, 2U);
  EXPECT_EQ(texInfo[0].width, 8U);
  EXPECT_EQ(texInfo[0].height, 4U);
  EXPECT_FALSE(texInfo[0].hasAlpha);
  EXPECT_EQ(texInfo[0].filepath, dir.path("texture.png"));

  // The diffuse color of the .mtl must reach the triangle vertices:
  const auto obj = m->getByClass<CSetOfTexturedTriangles>(0);
  ASSERT_TRUE(obj);
  ASSERT_EQ(obj->getTrianglesCount(), 2U);
  const auto tri = obj->getTriangle(0);
  EXPECT_EQ(tri.vertices[0].xyzrgba.r, 255);
  EXPECT_NEAR(tri.vertices[0].xyzrgba.g, 127, 2);
  EXPECT_NEAR(tri.vertices[0].xyzrgba.b, 63, 2);
}

TEST(CAssimpModel, IgnoreTexturesFlag)
{
  const TempModelDir dir;
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_TEXTURED);
  writeTestTexture(dir.path("texture.png"));

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("quad.obj"), CAssimpModel::LoadFlags::IgnoreTextures);

  // All geometry must fall back into the single non-textured mesh:
  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_EQ(m->getNonTexturedTriangleCount(), 2U);
}

TEST(CAssimpModel, IgnoreMaterialColorFlag)
{
  const TempModelDir dir;
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_TEXTURED);
  writeTestTexture(dir.path("texture.png"));

  auto m = CAssimpModel::Create();
  m->setColor_u8(mrpt::img::TColor(10, 20, 30, 255));
  m->loadScene(
      dir.path("quad.obj"),
      CAssimpModel::LoadFlags::IgnoreMaterialColor | CAssimpModel::LoadFlags::IgnoreTextures);

  const auto obj = m->getByClass<CSetOfTriangles>(0);
  ASSERT_TRUE(obj);
  ASSERT_GT(obj->getTrianglesCount(), 0U);
  mrpt::viz::TTriangle tri;
  obj->getTriangle(0, tri);
  EXPECT_EQ(tri.vertices[0].xyzrgba.r, 10);
  EXPECT_EQ(tri.vertices[0].xyzrgba.g, 20);
  EXPECT_EQ(tri.vertices[0].xyzrgba.b, 30);
}

TEST(CAssimpModel, MissingTextureFallsBackToNonTextured)
{
  const TempModelDir dir;
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_MISSING_TEXTURE);

  auto m = CAssimpModel::Create();
  // Verbose exercises the diagnostic messages of the "texture not found" path:
  m->loadScene(
      dir.path("quad.obj"),
      CAssimpModel::LoadFlags::RealTimeMaxQuality | CAssimpModel::LoadFlags::Verbose);

  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_EQ(m->getNonTexturedTriangleCount(), 2U);
}

TEST(CAssimpModel, LoadNonExistentFileThrows)
{
  auto m = CAssimpModel::Create();
  EXPECT_THROW(m->loadScene("/nonexistent/path/to/model.obj"), std::exception);
}

TEST(CAssimpModel, ClearResetsState)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  ASSERT_EQ(m->getTotalTriangleCount(), 1U);

  m->clear();
  EXPECT_TRUE(m->getModelPath().empty());
  EXPECT_EQ(m->getModelLoadFlags(), 0U);
  EXPECT_EQ(m->getTotalTriangleCount(), 0U);
  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_TRUE(m->empty());
}

TEST(CAssimpModel, ReloadReplacesPreviousContent)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_TEXTURED);
  writeTestTexture(dir.path("texture.png"));

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("quad.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  ASSERT_EQ(m->getTexturedMeshCount(), 1U);

  m->loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  EXPECT_EQ(m->getTexturedMeshCount(), 0U);
  EXPECT_EQ(m->getTotalTriangleCount(), 1U);
}

TEST(CAssimpModel, SplitTrianglesRenderingBBoxSetter)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);

  auto m = CAssimpModel::Create();
  m->setSplitTrianglesRenderingBBox(2.0f);
  EXPECT_FLOAT_EQ(m->getSplitTrianglesRenderingBBox(), 2.0f);

  // Setting the same value again is a no-op:
  m->setSplitTrianglesRenderingBBox(2.0f);
  EXPECT_FLOAT_EQ(m->getSplitTrianglesRenderingBBox(), 2.0f);

  m->loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  EXPECT_EQ(m->getTotalTriangleCount(), 1U);

  // With content already loaded, this re-applies the (currently no-op) split:
  m->setSplitTrianglesRenderingBBox(1.0f);
  EXPECT_FLOAT_EQ(m->getSplitTrianglesRenderingBBox(), 1.0f);
}

TEST(CAssimpModel, TraceRayHitsLoadedGeometry)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);

  // Ray along +X from behind the triangle plane, looking at it:
  double dist = 0;
  const mrpt::poses::CPose3D rayPose(0.1, 0.1, -1.0, 0, mrpt::DEG2RAD(-90.0), 0);
  EXPECT_TRUE(m->traceRay(rayPose, dist));
  EXPECT_GT(dist, 0.0);
}

TEST(CAssimpModel, SerializationRoundTrip)
{
  const TempModelDir dir;
  dir.writeTextFile("quad.obj", OBJ_TEXTURED_QUAD);
  dir.writeTextFile("quad.mtl", MTL_TEXTURED);
  writeTestTexture(dir.path("texture.png"));

  auto m = CAssimpModel::Create();
  m->loadScene(dir.path("quad.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  m->setSplitTrianglesRenderingBBox(3.0f);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *m;
  buf.Seek(0);

  auto m2 = CAssimpModel::Create();
  arch >> *m2;

  EXPECT_EQ(m2->getModelPath(), m->getModelPath());
  EXPECT_EQ(m2->getModelLoadFlags(), m->getModelLoadFlags());
  EXPECT_FLOAT_EQ(m2->getSplitTrianglesRenderingBBox(), 3.0f);

  // The child objects are restored, and re-indexed into the textured/
  // non-textured pointers:
  EXPECT_EQ(m2->getTexturedMeshCount(), m->getTexturedMeshCount());
  EXPECT_EQ(m2->getTotalTriangleCount(), m->getTotalTriangleCount());
}

TEST(CAssimpModel, MoveSemantics)
{
  const TempModelDir dir;
  dir.writeTextFile("tri.obj", OBJ_BARE_TRIANGLE);

  CAssimpModel a;
  a.loadScene(dir.path("tri.obj"), CAssimpModel::LoadFlags::RealTimeFast);
  const std::string path = a.getModelPath();

  CAssimpModel b(std::move(a));
  EXPECT_EQ(b.getModelPath(), path);
  EXPECT_EQ(b.getTotalTriangleCount(), 1U);

  CAssimpModel c;
  c = std::move(b);
  EXPECT_EQ(c.getModelPath(), path);
  EXPECT_EQ(c.getTotalTriangleCount(), 1U);
}

#else  // MRPT_HAS_ASSIMP

TEST(CAssimpModel, LoadSceneThrowsWithoutAssimp)
{
  auto m = mrpt::viz::CAssimpModel::Create();
  EXPECT_THROW(m->loadScene("whatever.obj"), std::exception);
}

#endif
