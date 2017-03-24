/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace std;

/* ------------------------------------------------------------------------
					Test_SoG_Merge
   ------------------------------------------------------------------------ */
void Test_SoG_Merge()
{
	CPosePDFSOG		pdf;

	CPosePDFSOG::TGaussianMode	m;
	m.mean = CPose2D( 1.1, -0.1, DEG2RAD(-2) );

	m.cov.unit();
	m.cov(0,0) = m.cov(1,1) = square(0.1);
	m.cov(2,2) = square(DEG2RAD(2));
	m.log_w = 0;

	pdf.clear();
	pdf.push_back(m);

	m.mean = CPose2D( 1.1, 0.1, DEG2RAD(2) );
	pdf.push_back(m);

	m.mean = CPose2D(2, 0, DEG2RAD(20) );
	pdf.push_back(m);

	cout << "Initial PDF: mean: " <<  pdf.getMeanVal() << endl;
	cout << pdf.getCovariance() << endl << endl;


#if MRPT_HAS_WXWIDGETS
	CDisplayWindow3D	win_before("Before merge");
	CDisplayWindow3D	win_after("After merge");
#endif

	{
		COpenGLScene	scene;
		CSetOfObjectsPtr  o = CSetOfObjects::Create();
		pdf.getAs3DObject(o);
		scene.insert(o);
		scene.insert( CGridPlaneXY::Create(-5,5,-5,5,0,1) );
		CFileGZOutputStream("sog_before.3Dscene") << scene;

	#if MRPT_HAS_WXWIDGETS
		COpenGLScenePtr sc = win_before.get3DSceneAndLock();
		*sc = scene;
		win_before.unlockAccess3DScene();
		win_before.setCameraZoom(5);
		win_before.setCameraPointingToPoint(1,0,0);
		win_before.forceRepaint();
	#endif
	}

	cout << "Merging...";
	pdf.mergeModes(0.9, true);
	cout << " # modes after: " << pdf.size() << endl;

	cout << "Final PDF: mean: " <<  pdf.getMeanVal() << endl;
	cout << pdf.getCovariance() << endl << endl;



	{
		COpenGLScene	scene;
		CSetOfObjectsPtr  o = CSetOfObjects::Create();
		pdf.getAs3DObject(o);
		scene.insert(o);
		scene.insert( CGridPlaneXY::Create(-5,5,-5,5,0,1) );
		CFileGZOutputStream("sog_after.3Dscene") << scene;

	#if MRPT_HAS_WXWIDGETS
		COpenGLScenePtr sc = win_after.get3DSceneAndLock();
		*sc = scene;
		win_after.unlockAccess3DScene();
		win_after.setCameraZoom(5);
		win_after.setCameraPointingToPoint(1,0,0);
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
int main(int argc, char **argv)
{
	try
	{
		Test_SoG_Merge();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
