/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/ransac.h>
#include <mrpt/math/model_search.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

struct Fit3DPlane {
	typedef TPlane3D		Model;
	typedef double			Real;

	const std::vector<TPoint3D>&  allData;

	Fit3DPlane( const std::vector<TPoint3D>& _allData ) : allData(_allData) {}

	size_t getSampleCount( void ) const
	{
		return allData.size();
	}

	bool  fitModel( const vector_size_t& useIndices, TPlane3D& model ) const
	{
		ASSERT_(useIndices.size()==3);
		TPoint3D  p1( allData[useIndices[0]] );
		TPoint3D  p2( allData[useIndices[1]] );
		TPoint3D  p3( allData[useIndices[2]] );

		try
		{
			model = TPlane( p1,p2,p3 );
		}
		catch(exception &)
		{
			return false;
		}

		return true;
	}

	double testSample( size_t index, const TPlane3D& model ) const
	{
		return model.distance( allData[index] );
	}
};


// ------------------------------------------------------
//				TestRANSAC
// ------------------------------------------------------
void TestRANSAC()
{
	randomGenerator.randomize(789);

	// Generate random points:
	// ------------------------------------
	const size_t N_plane = 300;
	const size_t N_noise = 100;

	const double PLANE_EQ[4]={ 1,-1,1, -2 };

	std::vector<TPoint3D> data;
	for (size_t i=0;i<N_plane;i++)
	{
		const double xx = randomGenerator.drawUniform(-3,3);
		const double yy = randomGenerator.drawUniform(-3,3);
		const double zz = -(PLANE_EQ[3]+PLANE_EQ[0]*xx+PLANE_EQ[1]*yy)/PLANE_EQ[2];
		data.push_back( TPoint3D(xx,yy,zz) );
	}

	for (size_t i=0;i<N_noise;i++)
	{
		TPoint3D& p = data[i];
		p += TPoint3D( randomGenerator.drawUniform(-4,4),
					   randomGenerator.drawUniform(-4,4),
					   randomGenerator.drawUniform(-4,4) );
	}

	// Run RANSAC
	// ------------------------------------
	TPlane3D best_model;
	vector_size_t best_inliers;
	const double DIST_THRESHOLD = 0.2;


	CTicTac	tictac;
	const size_t TIMES=100;

	bool found = false;
	for (size_t iters=0;iters<TIMES;iters++)
	{
		ModelSearch search;
		Fit3DPlane fit( data );
		found = search.geneticSingleModel( fit, 3, DIST_THRESHOLD, 10, 100, best_model, best_inliers );
	}
	cout << "Computation time (genetic): " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

	for (size_t iters=0;iters<TIMES;iters++)
	{
		ModelSearch search;
		Fit3DPlane fit( data );
		found = search.ransacSingleModel( fit, 3, DIST_THRESHOLD, best_model, best_inliers );
	}
	cout << "Computation time (ransac): " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

	if( !found )
		return;

//	cout << "RANSAC finished: Best model: " << best_model << endl;
//	cout << "Best inliers: " << best_inliers << endl;


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
		std::vector<float> x,y,z;
		x.reserve( data.size() );
		y.reserve( data.size() );
		z.reserve( data.size() );
		for( size_t i = 0; i < data.size(); i++ )
		{
			x.push_back(data[i].x);
			y.push_back(data[i].y);
			z.push_back(data[i].z);
		}
		points->setAllPointsFast(x,y,z);
	}


	scene->insert( points );

	opengl::CTexturedPlanePtr glPlane = opengl::CTexturedPlane::Create(-4,4,-4,4);

	CPose3D   glPlanePose;
	best_model.getAsPose3D( glPlanePose );
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
