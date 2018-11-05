/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/opengl.h>

#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::serialization;
using namespace std;

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(SerializeTestOpenGL, WriteReadToMem)
{
	const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
		CLASS_ID(CAxis),
		CLASS_ID(CBox),
		CLASS_ID(CFrustum),
		CLASS_ID(CDisk),
		CLASS_ID(CGridPlaneXY),
#if MRPT_HAS_OPENCV  // These classes need CImage serialization
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
		CLASS_ID(CGeneralizedCylinder),
		CLASS_ID(CPolyhedron),
		CLASS_ID(CArrow),
		CLASS_ID(CCamera),
		CLASS_ID(CEllipsoid),
		CLASS_ID(CGridPlaneXZ),
		CLASS_ID(COpenGLScene),
		CLASS_ID(CSetOfObjects),
		CLASS_ID(CSimpleLine),
		CLASS_ID(CText),
		CLASS_ID(CText3D),
		CLASS_ID(CEllipsoidInverseDepth2D),
		CLASS_ID(CEllipsoidInverseDepth3D),
		CLASS_ID(CEllipsoidRangeBearing2D),
		CLASS_ID(COctoMapVoxels)
	};

	for (auto& lstClasse : lstClasses)
	{
		try
		{
			mrpt::io::CMemoryStream buf;
			{
				auto* o =
					static_cast<CSerializable*>(lstClasse->createObject());
				mrpt::serialization::archiveFrom(buf) << *o;
				delete o;
			}

			CSerializable::Ptr recons;
			buf.Seek(0);
			mrpt::serialization::archiveFrom(buf) >> recons;
		}
		catch (const std::exception& e)
		{
			GTEST_FAIL() << "Exception during serialization test for class '"
						 << lstClasse->className << "':\n"
						 << e.what() << endl;
		}
	}
}
