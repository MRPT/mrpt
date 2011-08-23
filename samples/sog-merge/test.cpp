/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/base.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
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
