/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/ransac_applications.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CTexturedPlane.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;


mrpt::gui::CDisplayWindow3DPtr  win;

// ------------------------------------------------------
//				TestRANSACPlanes
// ------------------------------------------------------
void TestRANSACPlanes()
{
	randomGenerator.randomize();

	// Generate random points:
	// ------------------------------------
	const size_t N_PLANES = 3;

	const size_t N_plane = 300;
	const size_t N_noise = 300;

	const double PLANE_EQ[N_PLANES][4]={ 
		{ 1,-1,1, -2 },
		{ 1,+1.5, 1, -1 },
		{ 0,-1,1, +2 } };

	CVectorDouble xs,ys,zs;
	for (size_t p=0;p<N_PLANES;p++)
	{
		for (size_t i=0;i<N_plane;i++)
		{
			const double xx = randomGenerator.drawUniform(-3,3) + 5*cos(0.4*p);
			const double yy = randomGenerator.drawUniform(-3,3) + 5*sin(0.4*p);
			const double zz = -(PLANE_EQ[p][3]+PLANE_EQ[p][0]*xx+PLANE_EQ[p][1]*yy)/PLANE_EQ[p][2];
			xs.push_back(xx);
			ys.push_back(yy);
			zs.push_back(zz);
		}
	}

	for (size_t i=0;i<N_noise;i++)
	{
		xs.push_back( randomGenerator.drawUniform(-7,7));
		ys.push_back( randomGenerator.drawUniform(-7,7));
		zs.push_back( randomGenerator.drawUniform(-7,7));
	}


	// Run RANSAC
	// ------------------------------------
	vector<pair<size_t,TPlane> >   detectedPlanes;
	const double DIST_THRESHOLD = 0.05;

	CTicTac	tictac;

	ransac_detect_3D_planes(xs,ys,zs,detectedPlanes,DIST_THRESHOLD,40 );

	// Display output:
	cout << "RANSAC method: ransac_detect_3D_planes" << endl;
	cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	cout << " " << detectedPlanes.size() << " planes detected." << endl;


	// Show GUI
	// --------------------------
	win = mrpt::gui::CDisplayWindow3DPtr( new mrpt::gui::CDisplayWindow3D("RANSAC: 3D planes", 500,500));

	opengl::COpenGLScenePtr scene = opengl::COpenGLScene::Create();

	scene->insert( opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1) );
	scene->insert( opengl::stock_objects::CornerXYZ() );

	for (vector<pair<size_t,TPlane> >::iterator p=detectedPlanes.begin();p!=detectedPlanes.end();++p)
	{
		opengl::CTexturedPlanePtr glPlane = opengl::CTexturedPlane::Create(-10,10,-10,10);

		CPose3D   glPlanePose;
		p->second.getAsPose3D( glPlanePose );
		glPlane->setPose(glPlanePose);

		glPlane->setColor( randomGenerator.drawUniform(0,1), randomGenerator.drawUniform(0,1),randomGenerator.drawUniform(0,1), 0.6);

		scene->insert( glPlane );
	}

	{
		opengl::CPointCloudPtr  points = opengl::CPointCloud::Create();
		points->setColor(0,0,1);
		points->setPointSize(3);
		points->enableColorFromZ();

		// Convert double -> float:
		vector<float> xsf,ysf,zsf;
		metaprogramming::copy_container_typecasting(xs,xsf);
		metaprogramming::copy_container_typecasting(ys,ysf);
		metaprogramming::copy_container_typecasting(zs,zsf);

		points->setAllPoints(xsf,ysf,zsf);

		scene->insert( points );
	}

	win->get3DSceneAndLock() = scene;
	win->unlockAccess3DScene();
	win->forceRepaint();


	win->waitForKey();
}

// ------------------------------------------------------
//				TestRANSACLines
// ------------------------------------------------------
void TestRANSACLines()
{
	randomGenerator.randomize();

	// Generate random points in 2D
	// ------------------------------------
	const size_t N_LINES = 4;

	const size_t N_line = 30;
	const size_t N_noise = 50;

	const double LINE_EQ[N_LINES][3]={ 
		{ 1,-1, -2 },
		{ 1,+1.5, -1 },
		{ 0,-1, +2 },
		{ 0.5,-0.3, +1 } };

	CVectorDouble xs,ys;
	for (size_t p=0;p<N_LINES;p++)
	{
		for (size_t i=0;i<N_line;i++)
		{
			const double xx = randomGenerator.drawUniform(-10,10);
			const double yy = randomGenerator.drawGaussian1D(0,0.05)  -(LINE_EQ[p][2]+LINE_EQ[p][0]*xx)/LINE_EQ[p][1];
			xs.push_back(xx);
			ys.push_back(yy);
		}
	}

	for (size_t i=0;i<N_noise;i++)
	{
		xs.push_back( randomGenerator.drawUniform(-15,15));
		ys.push_back( randomGenerator.drawUniform(-15,15));
	}


	// Run RANSAC
	// ------------------------------------
	vector<pair<size_t,TLine2D > >   detectedLines;
	const double DIST_THRESHOLD = 0.2;

	CTicTac	tictac;

	ransac_detect_2D_lines(xs,ys,detectedLines,DIST_THRESHOLD,20 );

	// Display output:
	cout << "RANSAC method: ransac_detect_2D_lines" << endl;
	cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	cout << " " << detectedLines.size() << " lines detected." << endl;


	// Show GUI
	// --------------------------
	mrpt::gui::CDisplayWindowPlots  win2("Set of points", 500,500);

	win2.plot(xs,ys,".b4","points");

	unsigned int n=0;
	for (vector<pair<size_t,TLine2D> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
	{
		CVectorDouble lx(2),ly(2);
		lx[0] = -15;
		lx[1] = 15;
		for (CVectorDouble::Index q=0;q<lx.size();q++)
			ly[q] = -(p->second.coefs[2]+p->second.coefs[0]*lx[q])/p->second.coefs[1];
		win2.plot(lx,ly,"r-1",format("line_%u",n++));
	}

	win2.axis_fit();
	win2.axis_equal();

	win2.waitForKey();
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestRANSACPlanes();
		cout << endl << "Now running detection of lines..." << endl << endl;
		TestRANSACLines();

		win.clear();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
