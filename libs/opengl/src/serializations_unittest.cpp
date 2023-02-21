/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR
#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/opengl.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>
//
#include <mrpt/config.h>
#ifndef MRPT_BUILT_AS_DLL
#include <mrpt/opengl/registerAllClasses.h>
#endif

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::serialization;
using namespace std;

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(SerializeTestOpenGL, WriteReadToMem)
{
#ifndef MRPT_BUILT_AS_DLL
	mrpt::opengl::registerAllClasses_mrpt_opengl();
#endif

	const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
		CLASS_ID(CAxis),
		CLASS_ID(CBox),
		CLASS_ID(CFrustum),
		CLASS_ID(CDisk),
		CLASS_ID(CGridPlaneXY),
#if MRPT_HAS_OPENCV	 // These classes need CImage serialization
		CLASS_ID(CMesh),
		CLASS_ID(CTexturedPlane),
#endif
		CLASS_ID(COpenGLViewport),
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
		CLASS_ID(COpenGLScene),
		CLASS_ID(CSetOfObjects),
		CLASS_ID(CSimpleLine),
		CLASS_ID(CText),
		CLASS_ID(CText3D),
		CLASS_ID(CEllipsoidInverseDepth2D),
		CLASS_ID(CEllipsoidInverseDepth3D),
		CLASS_ID(CEllipsoidRangeBearing2D),
		CLASS_ID(COctoMapVoxels),
		CLASS_ID(CSkyBox)
	};

	for (auto& cl : lstClasses)
	{
		try
		{
			mrpt::io::CMemoryStream buf;
			{
				auto o =
					mrpt::ptr_cast<CSerializable>::from(cl->createObject());
				mrpt::serialization::archiveFrom(buf) << *o;
				o.reset();
			}

			CSerializable::Ptr recons;
			buf.Seek(0);
			mrpt::serialization::archiveFrom(buf) >> recons;
		}
		catch (const std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << cl->className << "':\n"
						 << e.what() << endl;
		}
	}
}

TEST(SerializeTestOpenGL, PredefinedSceneFile)
{
	using namespace std::string_literals;

	//! JS_PRELOAD_FILE <tests/default-scene.3Dscene>
	const std::string fil =
		mrpt::UNITTEST_BASEDIR() + "/tests/default-scene.3Dscene"s;

	mrpt::opengl::COpenGLScene scene;

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
