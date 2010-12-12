/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/gui.h>
#include <mrpt/opengl.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;

void insertRandomPoints_uniform(
	const size_t N,
	opengl::CPointCloudPtr &gl, 
	const mrpt::math::TPoint3D &p_min,
	const mrpt::math::TPoint3D &p_max)
{
	for (size_t i=0;i<N;i++)
		gl->insertPoint(
			random::randomGenerator.drawUniform(p_min.x,p_max.x),
			random::randomGenerator.drawUniform(p_min.y,p_max.y),
			random::randomGenerator.drawUniform(p_min.z,p_max.z) );
}

void insertRandomPoints_gauss(
	const size_t N,
	opengl::CPointCloudPtr &gl, 
	const mrpt::math::TPoint3D &p_mean,
	const mrpt::math::TPoint3D &p_stddevs)
{
	for (size_t i=0;i<N;i++)
		gl->insertPoint(
			random::randomGenerator.drawGaussian1D(p_mean.x,p_stddevs.x),
			random::randomGenerator.drawGaussian1D(p_mean.y,p_stddevs.y),
			random::randomGenerator.drawGaussian1D(p_mean.z,p_stddevs.z) );
}


// ------------------------------------------------------
//				TestOctreeRenderHugePointCloud
// ------------------------------------------------------
void TestOctreeRenderHugePointCloud()
{
	CDisplayWindow3D	win("Demo of MRPT's octree pointclouds",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

//	theScene->insert( mrpt::opengl::CGridPlaneXY::Create(-10000,10000,-10000,10000,0, 100 ) );

	// CPointCloud
	opengl::CPointCloudPtr gl_pointcloud = opengl::CPointCloud::Create();
	theScene->insert( gl_pointcloud );

	gl_pointcloud->setPointSize(2.0);
	gl_pointcloud->enablePointSmooth();
	gl_pointcloud->enableColorFromZ();

	// Set the list of all points:
	const double R1 = 500;
	const double R2 = 30;
	const double H  = 100;
	const size_t nSteps = 200;

	for (float a=0;a<2*M_PI;a+=2*M_PI/nSteps)
		insertRandomPoints_gauss(
			1e3,
			gl_pointcloud, 
			TPoint3D(R1*0.5+ R1*cos(a) - R2*cos(4*a), R1*0.5+ R1*sin(a) - R2*sin(4*a),H*cos(8*a)), 
			TPoint3D(2,2,0.6)
			);

	insertRandomPoints_uniform(
		1e6, 
		gl_pointcloud, 
		TPoint3D(-20,-20,-5), 
		TPoint3D(1000,20,5) 
		);

	insertRandomPoints_uniform(
		1e6, 
		gl_pointcloud, 
		TPoint3D(1000-20,-20,-5), 
		TPoint3D(1000+20,1000+20,5) 
		);

	insertRandomPoints_uniform(
		1e6, 
		gl_pointcloud, 
		TPoint3D(-20,1000-20,-5), 
		TPoint3D(1000,1000+20,5)
		);

	insertRandomPoints_uniform(
		1e6, 
		gl_pointcloud, 
		TPoint3D(-20,-20,-5), 
		TPoint3D(+20,1000+20,5) 
		);


	// Draw the octree bounding boxes:
	mrpt::opengl::CSetOfObjectsPtr gl_bb = mrpt::opengl::CSetOfObjects::Create();
	gl_pointcloud->octree_get_graphics_boundingboxes(*gl_bb);
	theScene->insert( gl_bb );

	//gl_pointcloud->octree_debug_dump_tree(std::cout);

	win.setCameraZoom(600);
	//win.setMaxRange(100000);

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window or press any key to end.\n";
	bool end=false;
	while (win.isOpen() && !end)
	{
		mrpt::system::sleep(5);

		if (win.keyHit())
		{
			switch (win.getPushedKey())
			{
			case 'q': 
				end=true;
				break;
			case 'b': 
				gl_bb->setVisibility( !gl_bb->isVisible() );
				break;
			};
		}
		
		// Update the texts on the gl display:
		string s = mrpt::format("FPS=%5.02f | Points=%.02e | Actually rendered=%.02e | Visib.oct.nodes: %u/%u",
			win.getRenderingFPS(), 
			(double)gl_pointcloud->size(),
			(double)gl_pointcloud->getActuallyRendered(),
			(unsigned int)gl_pointcloud->octree_get_visible_nodes(),
			(unsigned int)gl_pointcloud->octree_get_nonempty_node_count()
			);

		win.get3DSceneAndLock();
			win.addTextMessage(5,5, s, TColorf(1,0,0),0, MRPT_GLUT_BITMAP_HELVETICA_12 );
			win.addTextMessage(5,35, "'b': switch bounding-boxes visible, 'q': quit", TColorf(1,0,0),1, MRPT_GLUT_BITMAP_HELVETICA_12 );
		win.unlockAccess3DScene();
		win.repaint();

	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestOctreeRenderHugePointCloud();

		mrpt::system::sleep(50);

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
