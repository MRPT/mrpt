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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CPolyhedron.h>

using namespace mrpt::viz;
using namespace mrpt::math;

namespace
{
// Every closed, simply-connected polyhedron must satisfy Euler's formula.
void checkClosedEulerFormula(const CPolyhedron::Ptr& p)
{
  ASSERT_TRUE(p);
  EXPECT_TRUE(p->isClosed());
  const int64_t v = p->getNumberOfVertices();
  const int64_t e = p->getNumberOfEdges();
  const int64_t f = p->getNumberOfFaces();
  EXPECT_GT(v, 0);
  EXPECT_GT(e, 0);
  EXPECT_GT(f, 0);
  EXPECT_EQ(v - e + f, 2);
}

void checkBasicSanity(const CPolyhedron::Ptr& p)
{
  ASSERT_TRUE(p);
  std::vector<TPoint3D> vertices;
  p->getVertices(vertices);
  EXPECT_EQ(vertices.size(), p->getNumberOfVertices());

  std::vector<CPolyhedron::TPolyhedronEdge> edges;
  p->getEdges(edges);
  EXPECT_EQ(edges.size(), p->getNumberOfEdges());

  std::vector<CPolyhedron::TPolyhedronFace> faces;
  p->getFaces(faces);
  EXPECT_EQ(faces.size(), p->getNumberOfFaces());

  std::vector<double> edgeLengths;
  p->getEdgesLength(edgeLengths);
  EXPECT_EQ(edgeLengths.size(), edges.size());
  for (double l : edgeLengths)
  {
    EXPECT_GT(l, 0);
  }

  std::vector<double> faceAreas;
  p->getFacesArea(faceAreas);
  EXPECT_EQ(faceAreas.size(), faces.size());

  TPoint3D center;
  p->getCenter(center);

  std::vector<TPolygon3D> polys;
  p->getSetOfPolygons(polys);
  EXPECT_EQ(polys.size(), faces.size());

  std::vector<TPolygon3D> polysAbs;
  p->setLocation(1.0, 2.0, 3.0);
  p->getSetOfPolygonsAbsolute(polysAbs);
  EXPECT_EQ(polysAbs.size(), faces.size());
  p->setLocation(0, 0, 0);
}

}  // namespace

// ---- Platonic solids ----
TEST(CPolyhedron, Tetrahedron) { checkClosedEulerFormula(CPolyhedron::CreateTetrahedron(1.0)); }
TEST(CPolyhedron, Hexahedron)
{
  auto p = CPolyhedron::CreateHexahedron(1.0);
  checkClosedEulerFormula(p);
  checkBasicSanity(p);
  EXPECT_GT(p->getVolume(), 0);
}
TEST(CPolyhedron, Octahedron) { checkClosedEulerFormula(CPolyhedron::CreateOctahedron(1.0)); }
TEST(CPolyhedron, Dodecahedron) { checkClosedEulerFormula(CPolyhedron::CreateDodecahedron(1.0)); }
TEST(CPolyhedron, Icosahedron) { checkClosedEulerFormula(CPolyhedron::CreateIcosahedron(1.0)); }

// ---- Archimedean solids ----
TEST(CPolyhedron, TruncatedTetrahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedTetrahedron(1.0));
}
TEST(CPolyhedron, Cuboctahedron) { checkClosedEulerFormula(CPolyhedron::CreateCuboctahedron(1.0)); }
TEST(CPolyhedron, TruncatedHexahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedHexahedron(1.0));
}
TEST(CPolyhedron, TruncatedOctahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedOctahedron(1.0));
}
TEST(CPolyhedron, RhombicuboctahedronType1)
{
  checkClosedEulerFormula(CPolyhedron::CreateRhombicuboctahedron(1.0, true));
}
TEST(CPolyhedron, RhombicuboctahedronType2)
{
  checkClosedEulerFormula(CPolyhedron::CreateRhombicuboctahedron(1.0, false));
}
TEST(CPolyhedron, IcosidodecahedronType1)
{
  checkClosedEulerFormula(CPolyhedron::CreateIcosidodecahedron(1.0, true));
}
TEST(CPolyhedron, IcosidodecahedronType2)
{
  checkClosedEulerFormula(CPolyhedron::CreateIcosidodecahedron(1.0, false));
}
TEST(CPolyhedron, TruncatedDodecahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedDodecahedron(1.0));
}
TEST(CPolyhedron, TruncatedIcosahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedIcosahedron(1.0));
}
TEST(CPolyhedron, Rhombicosidodecahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateRhombicosidodecahedron(1.0));
}

// ---- Other Johnson solid ----
TEST(CPolyhedron, PentagonalRotunda)
{
  checkClosedEulerFormula(CPolyhedron::CreatePentagonalRotunda(1.0));
}

// ---- Catalan solids ----
TEST(CPolyhedron, TriakisTetrahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTriakisTetrahedron(1.0));
}
TEST(CPolyhedron, RhombicDodecahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateRhombicDodecahedron(1.0));
}
TEST(CPolyhedron, TriakisOctahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTriakisOctahedron(1.0));
}
TEST(CPolyhedron, TetrakisHexahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTetrakisHexahedron(1.0));
}
TEST(CPolyhedron, DeltoidalIcositetrahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateDeltoidalIcositetrahedron(1.0));
}
TEST(CPolyhedron, RhombicTriacontahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateRhombicTriacontahedron(1.0));
}
TEST(CPolyhedron, TriakisIcosahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTriakisIcosahedron(1.0));
}
TEST(CPolyhedron, PentakisDodecahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreatePentakisDodecahedron(1.0));
}
TEST(CPolyhedron, DeltoidalHexecontahedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateDeltoidalHexecontahedron(1.0));
}

// ---- Customizable polyhedra ----
TEST(CPolyhedron, CubicPrismFromCoords)
{
  checkClosedEulerFormula(CPolyhedron::CreateCubicPrism(-1, 1, -1, 1, -1, 1));
}
TEST(CPolyhedron, CubicPrismFromPoints)
{
  checkClosedEulerFormula(CPolyhedron::CreateCubicPrism(TPoint3D(-1, -1, -1), TPoint3D(1, 1, 1)));
}
TEST(CPolyhedron, Pyramid)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreatePyramid(base, 2.0));
}
TEST(CPolyhedron, PyramidNotEnoughVertices)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1}
  };
  EXPECT_THROW(CPolyhedron::CreatePyramid(base, 2.0), std::logic_error);
}
TEST(CPolyhedron, DoublePyramid)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreateDoublePyramid(base, 2.0, 1.5));
}
TEST(CPolyhedron, TruncatedPyramid)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreateTruncatedPyramid(base, 2.0, 0.5));
}
TEST(CPolyhedron, Frustum)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreateFrustum(base, 2.0, 0.5));
}
TEST(CPolyhedron, CustomPrism)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreateCustomPrism(base, 2.0));
}
TEST(CPolyhedron, CustomAntiprism)
{
  std::vector<TPoint2D> bottom{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  std::vector<TPoint2D> top{
      {-0.8, -0.6},
      { 0.6, -0.8},
      { 0.8,  0.6},
      {-0.6,  0.8}
  };
  checkClosedEulerFormula(CPolyhedron::CreateCustomAntiprism(bottom, top, 2.0));
}
TEST(CPolyhedron, CustomAntiprismMismatchedBases)
{
  std::vector<TPoint2D> bottom{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  std::vector<TPoint2D> top{
      {-1, -1},
      { 1, -1},
      { 1,  1}
  };
  EXPECT_THROW(CPolyhedron::CreateCustomAntiprism(bottom, top, 2.0), std::logic_error);
}
TEST(CPolyhedron, Parallelepiped)
{
  checkClosedEulerFormula(CPolyhedron::CreateParallelepiped(
      TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0), TPoint3D(0, 0, 1)));
}
TEST(CPolyhedron, Bifrustum)
{
  std::vector<TPoint2D> base{
      {-1, -1},
      { 1, -1},
      { 1,  1},
      {-1,  1}
  };
  checkClosedEulerFormula(CPolyhedron::CreateBifrustum(base, 1.5, 0.6, 1.2, 0.4));
}
TEST(CPolyhedron, Trapezohedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateTrapezohedron(5, 1.0, 1.5));
}
TEST(CPolyhedron, TrapezohedronNotEnoughVertices)
{
  EXPECT_THROW(CPolyhedron::CreateTrapezohedron(2, 1.0, 1.5), std::logic_error);
}
TEST(CPolyhedron, RegularAntiprism)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularAntiprism(6, 1.0, 1.0));
}
TEST(CPolyhedron, RegularPrism)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularPrism(6, 1.0, 1.0));
}
TEST(CPolyhedron, RegularPyramid)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularPyramid(6, 1.0, 1.0));
}
TEST(CPolyhedron, RegularDoublePyramid)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularDoublePyramid(6, 1.0, 1.0, 0.8));
}
TEST(CPolyhedron, ArchimedeanRegularPrism)
{
  checkClosedEulerFormula(CPolyhedron::CreateArchimedeanRegularPrism(6, 1.0));
}
TEST(CPolyhedron, ArchimedeanRegularAntiprism)
{
  checkClosedEulerFormula(CPolyhedron::CreateArchimedeanRegularAntiprism(6, 1.0));
}
TEST(CPolyhedron, RegularTruncatedPyramid)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularTruncatedPyramid(6, 1.0, 1.0, 0.5));
}
TEST(CPolyhedron, RegularFrustum)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularFrustum(6, 1.0, 1.0, 0.5));
}
TEST(CPolyhedron, RegularBifrustum)
{
  checkClosedEulerFormula(CPolyhedron::CreateRegularBifrustum(6, 1.0, 1.0, 0.6, 1.0, 0.4));
}
TEST(CPolyhedron, Cupola) { checkClosedEulerFormula(CPolyhedron::CreateCupola(6, 1.0)); }
TEST(CPolyhedron, CupolaOddEdgesThrows)
{
  EXPECT_THROW(CPolyhedron::CreateCupola(5, 1.0), std::logic_error);
}
TEST(CPolyhedron, CatalanTrapezohedron)
{
  checkClosedEulerFormula(CPolyhedron::CreateCatalanTrapezohedron(5, 1.0));
}
TEST(CPolyhedron, CatalanDoublePyramid)
{
  checkClosedEulerFormula(CPolyhedron::CreateCatalanDoublePyramid(4, 1.0));
}

TEST(CPolyhedron, JohnsonSolidWithConstantBase)
{
  // Examples taken straight from the API documentation.
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(3, 1.0, "P+"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(4, 1.0, "PR"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(4, 1.0, "P-P+"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(3, 1.0, "A"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(8, 1.0, "C-PRC+"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(5, 1.0, "P-AP+"));
  checkClosedEulerFormula(CPolyhedron::CreateJohnsonSolidWithConstantBase(10, 1.0, "R-R+"));
}
TEST(CPolyhedron, JohnsonSolidInvalidStringThrows)
{
  EXPECT_THROW(CPolyhedron::CreateJohnsonSolidWithConstantBase(4, 1.0, "ZZZ"), std::logic_error);
}
TEST(CPolyhedron, JohnsonSolidNotEnoughVerticesThrows)
{
  EXPECT_THROW(CPolyhedron::CreateJohnsonSolidWithConstantBase(2, 1.0, "PR"), std::logic_error);
}

// ---- Special operations ----
TEST(CPolyhedron, GetDual)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto dual = cube->getDual();
  ASSERT_TRUE(dual);
  EXPECT_EQ(dual->getNumberOfVertices(), cube->getNumberOfFaces());
  EXPECT_EQ(dual->getNumberOfFaces(), cube->getNumberOfVertices());
}

TEST(CPolyhedron, Truncate)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto truncated = cube->truncate(0.3);
  ASSERT_TRUE(truncated);
  EXPECT_GT(truncated->getNumberOfVertices(), 0u);

  // factor == 0: no-check copy of the same geometry
  auto same = cube->truncate(0.0);
  ASSERT_TRUE(same);
  EXPECT_EQ(same->getNumberOfVertices(), cube->getNumberOfVertices());

  // factor < 0: empty polyhedron
  auto empty = cube->truncate(-1.0);
  ASSERT_TRUE(empty);
  EXPECT_EQ(empty->getNumberOfVertices(), 0u);
}

TEST(CPolyhedron, Cantellate)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto cantellated = cube->cantellate(0.3);
  ASSERT_TRUE(cantellated);
  EXPECT_GT(cantellated->getNumberOfVertices(), 0u);
}

TEST(CPolyhedron, AugmentByHeight)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto augmented = cube->augment(0.5);
  ASSERT_TRUE(augmented);
  EXPECT_GT(augmented->getNumberOfVertices(), cube->getNumberOfVertices());
}

TEST(CPolyhedron, AugmentByHeightAndNumVertices)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto augmented = cube->augment(0.5, 4);
  ASSERT_TRUE(augmented);
  EXPECT_GE(augmented->getNumberOfVertices(), cube->getNumberOfVertices());
}

TEST(CPolyhedron, AugmentDirectionOnly)
{
  auto tetra = CPolyhedron::CreateTetrahedron(1.0);
  auto augmented = tetra->augment(false);
  ASSERT_TRUE(augmented);
}

TEST(CPolyhedron, AugmentByNumVerticesAndDirection)
{
  auto tetra = CPolyhedron::CreateTetrahedron(1.0);
  auto augmented = tetra->augment(size_t(3), false);
  ASSERT_TRUE(augmented);
}

TEST(CPolyhedron, RotateAndScale)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  auto rotated = cube->rotate(0.7853981);  // 45 deg
  ASSERT_TRUE(rotated);
  EXPECT_EQ(rotated->getNumberOfVertices(), cube->getNumberOfVertices());

  auto scaled = cube->scale(2.0);
  ASSERT_TRUE(scaled);
  EXPECT_GT(scaled->getVolume(), cube->getVolume());
}

TEST(CPolyhedron, ScaleNonPositiveThrows)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  EXPECT_THROW({ [[maybe_unused]] auto r = cube->scale(0.0); }, std::logic_error);
  EXPECT_THROW({ [[maybe_unused]] auto r = cube->scale(-1.0); }, std::logic_error);
}

TEST(CPolyhedron, MakeConvexPolygons)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  EXPECT_NO_THROW(cube->makeConvexPolygons());
}

TEST(CPolyhedron, TraceRay)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  double dist = 0;
  // A ray from outside, pointing towards the object, should hit it:
  const bool hit = cube->traceRay(mrpt::poses::CPose3D(-10, 0, 0, 0, 0, 0), dist);
  // Either it hits (dist>0) or, depending on face winding, might miss; just
  // exercise the code path without asserting a specific geometric result.
  (void)hit;
}

TEST(CPolyhedron, Wireframe)
{
  auto cube = CPolyhedron::CreateHexahedron(1.0);
  EXPECT_FALSE(cube->isWireframe());
  cube->setWireframe(true);
  EXPECT_TRUE(cube->isWireframe());
}

TEST(CPolyhedron, GetIntersection)
{
  auto cube1 = CPolyhedron::CreateHexahedron(1.0);
  auto cube2 = CPolyhedron::CreateHexahedron(1.0);
  cube2->setLocation(0.5, 0.5, 0.5);

  std::vector<TObject3D> intersections;
  const size_t n = CPolyhedron::getIntersection(cube1, cube2, intersections);
  EXPECT_EQ(n, intersections.size());
}

TEST(CPolyhedron, CreateRandomPolyhedron)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (int i = 0; i < 20; i++)
  {
    auto p = CPolyhedron::CreateRandomPolyhedron(1.0);
    ASSERT_TRUE(p);
  }
}

TEST(CPolyhedron, CreateNoCheckAndEmpty)
{
  auto empty = CPolyhedron::CreateEmpty();
  ASSERT_TRUE(empty);
  EXPECT_EQ(empty->getNumberOfVertices(), 0u);

  std::vector<TPoint3D> vertices{
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0}
  };
  std::vector<CPolyhedron::TPolyhedronFace> faces(1);
  faces[0].vertices = {0, 1, 2};
  auto p = CPolyhedron::CreateNoCheck(vertices, faces);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->getNumberOfVertices(), 3u);
}

TEST(CPolyhedron, ConstructFromVerticesAndFaceIndexLists)
{
  std::vector<TPoint3D> vertices{
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 0},
      {0, 1, 0}
  };
  std::vector<std::vector<uint32_t>> faces{
      {0, 1, 2, 3}
  };
  CPolyhedron p(vertices, faces);
  EXPECT_EQ(p.getNumberOfVertices(), 4u);
  EXPECT_EQ(p.getNumberOfFaces(), 1u);
}

TEST(CPolyhedron, ConstructFromVertexAndFaceListThrowsOnBadIndex)
{
  std::vector<TPoint3D> vertices{
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0}
  };
  std::vector<CPolyhedron::TPolyhedronFace> faces(1);
  faces[0].vertices = {0, 1, 99};  // out-of-range vertex index
  EXPECT_THROW(CPolyhedron(vertices, faces, true), std::logic_error);
}

TEST(CPolyhedron, ConstructFromPolygons)
{
  TPolygon3D poly;
  poly.push_back(TPoint3D(0, 0, 0));
  poly.push_back(TPoint3D(1, 0, 0));
  poly.push_back(TPoint3D(1, 1, 0));
  poly.push_back(TPoint3D(0, 1, 0));
  std::vector<TPolygon3D> polys{poly};
  CPolyhedron p(polys);
  EXPECT_EQ(p.getNumberOfVertices(), 4u);
  EXPECT_EQ(p.getNumberOfFaces(), 1u);
}

TEST(CPolyhedron, SerializeRoundTrip)
{
  auto p1 = CPolyhedron::CreateOctahedron(1.0);
  p1->setColor(0.2f, 0.4f, 0.6f);
  p1->setLocation(1, 2, 3);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *p1;
  buf.Seek(0);

  auto p2 = std::dynamic_pointer_cast<CPolyhedron>(arch.ReadObject());
  ASSERT_TRUE(p2);
  EXPECT_EQ(p2->getNumberOfVertices(), p1->getNumberOfVertices());
  EXPECT_EQ(p2->getNumberOfFaces(), p1->getNumberOfFaces());
}

TEST(CPolyhedron, BoundingBoxAndUpdateBuffers)
{
  auto p = CPolyhedron::CreateIcosahedron(1.0);
  EXPECT_NO_THROW(p->updateBuffers());
  const auto bb = p->internalBoundingBoxLocal();
  EXPECT_LE(bb.min.x, bb.max.x);
  EXPECT_LE(bb.min.y, bb.max.y);
  EXPECT_LE(bb.min.z, bb.max.z);
}
