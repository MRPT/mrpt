/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR
#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>
//
#include <mrpt/config.h>
#ifndef MRPT_BUILT_AS_DLL
#include <mrpt/viz/registerAllClasses.h>
#endif

// My classes:
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGeneralizedEllipsoidTemplate.h>
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
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTextMessageCapable.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CUBE_TEXTURE_FACE.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/TTriangle.h>
#include <mrpt/viz/Viewport.h>
#include <mrpt/viz/Visualizable.h>
#include <mrpt/viz/registerAllClasses.h>
#include <mrpt/viz/stock_objects.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::serialization;
using namespace std;

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(SerializeTestOpenGL, WriteReadToMem)
{
#ifndef MRPT_BUILT_AS_DLL
  mrpt::viz::registerAllClasses_mrpt_viz();
#endif

  const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
      CLASS_ID(CAxis),
      CLASS_ID(CBox),
      CLASS_ID(CFrustum),
      CLASS_ID(CDisk),
      CLASS_ID(CGridPlaneXY),
      CLASS_ID(CMesh),
      CLASS_ID(CTexturedPlane),
      CLASS_ID(CSkyBox),
      CLASS_ID(Viewport),
      CLASS_ID(CPointCloud),
      CLASS_ID(CPointCloudColoured),
      CLASS_ID(CSetOfLines),
      CLASS_ID(CSetOfTriangles),
      CLASS_ID(CSphere),
      CLASS_ID(CCylinder),
      CLASS_ID(CPolyhedron),
      CLASS_ID(CArrow),
      CLASS_ID(CCamera),
      CLASS_ID(CEllipsoid3D),
      CLASS_ID(CGridPlaneXZ),
      CLASS_ID(Scene),
      CLASS_ID(CSetOfObjects),
      CLASS_ID(CSimpleLine),
      CLASS_ID(CText),
      CLASS_ID(CText3D),
      CLASS_ID(CEllipsoidInverseDepth2D),
      CLASS_ID(CEllipsoidInverseDepth3D),
      CLASS_ID(CEllipsoidRangeBearing2D),
      CLASS_ID(COctoMapVoxels)};

  for (auto& cl : lstClasses)
  {
    try
    {
      mrpt::io::CMemoryStream buf;
      {
        auto o = mrpt::ptr_cast<CSerializable>::from(cl->createObject());
        mrpt::serialization::archiveFrom(buf) << *o;
        o.reset();
      }

      CSerializable::Ptr recons;
      buf.Seek(0);
      mrpt::serialization::archiveFrom(buf) >> recons;
    }
    catch (const std::exception& e)
    {
      GTEST_FAIL() << "Exception during serialization test for class '" << cl->className << "':\n"
                   << e.what() << endl;
    }
  }
}

TEST(SerializeTestOpenGL, PredefinedSceneFile)
{
  using namespace std::string_literals;

  //! JS_PRELOAD_FILE <tests/default-scene.3Dscene>
  const std::string fil = mrpt::UNITTEST_BASEDIR() + "/tests/default-scene.3Dscene"s;

  mrpt::viz::Scene scene;

  ASSERT_FILE_EXISTS_(fil);
  bool readOk = scene.loadFromFile(fil);
  EXPECT_TRUE(readOk);

  // scene.asYAML().printAsYAML();

  EXPECT_EQ(scene.viewportsCount(), 2U);
  size_t count = 0;
  for (const auto& obj : *scene.getViewport())
  {
    (void)obj;
    count++;
  }

  EXPECT_EQ(count, 2U);
}
