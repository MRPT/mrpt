/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/ransac.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
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



void  ransac3Dplane_fit(
	const CMatrixDouble  &allData,
	const vector_size_t  &useIndices,
	vector< CMatrixDouble > &fitModels )
{
	ASSERT_(useIndices.size()==3);

	TPoint3D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]),allData(2,useIndices[0]) );
	TPoint3D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]),allData(2,useIndices[1]) );
	TPoint3D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]),allData(2,useIndices[2]) );

	try
	{
		TPlane  plane( p1,p2,p3 );
		fitModels.resize(1);
		CMatrixDouble &M = fitModels[0];

		M.setSize(1,4);
		for (size_t i=0;i<4;i++)
			M(0,i)=plane.coefs[i];
	}
	catch(exception &)
	{
		fitModels.clear();
		return;
	}



}

void ransac3Dplane_distance(
	const CMatrixDouble &allData,
	const vector< CMatrixDouble > & testModels,
	const double distanceThreshold,
	unsigned int & out_bestModelIndex,
	vector_size_t & out_inlierIndices )
{
	ASSERT_( testModels.size()==1 )
	out_bestModelIndex = 0;
	const CMatrixDouble &M = testModels[0];

	ASSERT_( size(M,1)==1 && size(M,2)==4 )

	TPlane  plane;
	plane.coefs[0] = M(0,0);
	plane.coefs[1] = M(0,1);
	plane.coefs[2] = M(0,2);
	plane.coefs[3] = M(0,3);

	const size_t N = size(allData,2);
	out_inlierIndices.clear();
	out_inlierIndices.reserve(100);
	for (size_t i=0;i<N;i++)
	{
		const double d = plane.distance( TPoint3D( allData.get_unsafe(0,i),allData.get_unsafe(1,i),allData.get_unsafe(2,i) ) );
		if (d<distanceThreshold)
			out_inlierIndices.push_back(i);
	}
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac3Dplane_degenerate(
	const CMatrixDouble &allData,
	const mrpt::vector_size_t &useIndices )
{
	return false;
}


// ------------------------------------------------------
//				TestRANSAC
// ------------------------------------------------------
void TestRANSAC()
{
	randomGenerator.randomize();

	// Generate random points:
	// ------------------------------------
	const size_t N_plane = 300;
	const size_t N_noise = 100;

	const double PLANE_EQ[4]={ 1,-1,1, -2 };

	CMatrixDouble data(3,N_plane+N_noise);
	for (size_t i=0;i<N_plane;i++)
	{
		const double xx = randomGenerator.drawUniform(-3,3);
		const double yy = randomGenerator.drawUniform(-3,3);
		const double zz = -(PLANE_EQ[3]+PLANE_EQ[0]*xx+PLANE_EQ[1]*yy)/PLANE_EQ[2];
		data(0,i) = xx;
		data(1,i) = yy;
		data(2,i) = zz;
	}

	for (size_t i=0;i<N_noise;i++)
	{
		data(0,i+N_plane) = randomGenerator.drawUniform(-4,4);
		data(1,i+N_plane) = randomGenerator.drawUniform(-4,4);
		data(2,i+N_plane) = randomGenerator.drawUniform(-4,4);
	}


	// Run RANSAC
	// ------------------------------------
	CMatrixDouble best_model;
	vector_size_t best_inliers;
	const double DIST_THRESHOLD = 0.2;


	CTicTac	tictac;
	const size_t TIMES=100;

	math::RANSAC myransac;
	for (size_t iters=0;iters<TIMES;iters++)
		myransac.execute(
			data,
			ransac3Dplane_fit,
			ransac3Dplane_distance,
			ransac3Dplane_degenerate,
			DIST_THRESHOLD,
			3,  // Minimum set of points
			best_inliers,
			best_model,
			iters==0 ? mrpt::utils::LVL_DEBUG : mrpt::utils::LVL_INFO  // Verbose
			);

	cout << "Computation time: " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

	ASSERT_(size(best_model,1)==1 && size(best_model,2)==4)

	cout << "RANSAC finished: Best model: " << best_model << endl;
//	cout << "Best inliers: " << best_inliers << endl;

	TPlane  plane( best_model(0,0), best_model(0,1),best_model(0,2),best_model(0,3) );



	// Show GUI
	// --------------------------
	mrpt::gui::CDisplayWindow3D  win("Set of points", 500,500);
	opengl::COpenGLScenePtr scene = opengl::COpenGLScene::Create();

	scene->insert( opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1) );
	scene->insert( opengl::stock_objects::CornerXYZ() );

	opengl::CPointCloudPtr  points = opengl::CPointCloud::Create();
	points->setColor(0,0,1);
	points->setPointSize(3);
	points->enableColorFromZ();

	{
		std::vector<float> xs,ys,zs;

		data.extractRow(0, xs);
		data.extractRow(1, ys);
		data.extractRow(2, zs);
		points->setAllPointsFast(xs,ys,zs);
	}


	scene->insert( points );

	opengl::CTexturedPlanePtr glPlane = opengl::CTexturedPlane::Create(-4,4,-4,4);

	CPose3D   glPlanePose;
	plane.getAsPose3D( glPlanePose );
	glPlane->setPose(glPlanePose);

	scene->insert( glPlane );

	win.get3DSceneAndLock() = scene;
	win.unlockAccess3DScene();
	win.forceRepaint();




	win.waitForKey();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestRANSAC();
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
