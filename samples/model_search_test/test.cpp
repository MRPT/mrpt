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

#include <mrpt/math/ransac.h>
#include <mrpt/math/model_search.h>
#include <mrpt/base.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
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
