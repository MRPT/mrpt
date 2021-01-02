/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::io;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace std;

/* ------------------------------------------------------------------------
					Test_SoG_Merge
   ------------------------------------------------------------------------ */
void Test_SoG_Merge()
{
	CPosePDFSOG pdf;

	CPosePDFSOG::TGaussianMode m;
	m.mean = CPose2D(1.1, -0.1, -2.0_deg);

	m.cov.setIdentity();
	m.cov(0, 0) = m.cov(1, 1) = square(0.1);
	m.cov(2, 2) = square(2.0_deg);
	m.log_w = 0;

	pdf.clear();
	pdf.push_back(m);

	m.mean = CPose2D(1.1, 0.1, 2.0_deg);
	pdf.push_back(m);

	m.mean = CPose2D(2, 0, 20.0_deg);
	pdf.push_back(m);

	cout << "Initial PDF: mean: " << pdf.getMeanVal() << endl;
	cout << pdf.getCovariance() << endl << endl;

#if MRPT_HAS_WXWIDGETS
	CDisplayWindow3D win_before("Before merge");
	CDisplayWindow3D win_after("After merge");
#endif

	{
		COpenGLScene scene;
		CSetOfObjects::Ptr o = CSetOfObjects::Create();
		pdf.getAs3DObject(o);
		scene.insert(o);
		scene.insert(CGridPlaneXY::Create(-5, 5, -5, 5, 0, 1));

		CFileGZOutputStream f("sog_before.3Dscene");
		archiveFrom(f) << scene;

#if MRPT_HAS_WXWIDGETS
		COpenGLScene::Ptr sc = win_before.get3DSceneAndLock();
		*sc = scene;
		win_before.unlockAccess3DScene();
		win_before.setCameraZoom(5);
		win_before.setCameraPointingToPoint(1, 0, 0);
		win_before.forceRepaint();
#endif
	}

	cout << "Merging...";
	pdf.mergeModes(0.9, true);
	cout << " # modes after: " << pdf.size() << endl;

	cout << "Final PDF: mean: " << pdf.getMeanVal() << endl;
	cout << pdf.getCovariance() << endl << endl;

	{
		COpenGLScene scene;
		CSetOfObjects::Ptr o = CSetOfObjects::Create();
		pdf.getAs3DObject(o);
		scene.insert(o);
		scene.insert(CGridPlaneXY::Create(-5, 5, -5, 5, 0, 1));

		CFileGZOutputStream f("sog_after.3Dscene");
		archiveFrom(f) << scene;

#if MRPT_HAS_WXWIDGETS
		COpenGLScene::Ptr sc = win_after.get3DSceneAndLock();
		*sc = scene;
		win_after.unlockAccess3DScene();
		win_after.setCameraZoom(5);
		win_after.setCameraPointingToPoint(1, 0, 0);
		win_after.forceRepaint();
#endif
	}

#if MRPT_HAS_WXWIDGETS
	cout << "Push any key to exit..." << endl;
	mrpt::system::os::getch();
#endif
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		Test_SoG_Merge();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
