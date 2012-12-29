/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
