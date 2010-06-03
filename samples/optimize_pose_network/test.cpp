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

#include <mrpt/slam.h>

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;


#define STD_XY    0.2
#define STD_PHI   DEG2RAD(10)

void addNoisyEdge(size_t from, size_t to, const std::vector<CPose2D> &real_poses,CNetworkOfPoses2D &graph_links,const CMatrixDouble33 &cov)
{
	CPose2D p = real_poses[to] - real_poses[from];
	p.x_incr( randomGenerator.drawGaussian1D(0,STD_XY) );
	p.y_incr( randomGenerator.drawGaussian1D(0,STD_XY) );
	p.phi_incr( randomGenerator.drawGaussian1D(0,STD_PHI) );
	p.normalizePhi();

	graph_links.insertEdge(from,to,CPosePDFGaussian(p,cov));
}

// ------------------------------------------------------
//				TestGlobalOptimizationPosesGraph
// ------------------------------------------------------
void TestGlobalOptimizationPosesGraph()
{
	CTicTac tictac;
	CNetworkOfPoses2D	graph_links;
	CNetworkOfPoses2D::type_global_poses	optimal_poses, optimal_poses_dijkstra;

	randomGenerator.randomize(123);

	// Fill list of links:
	// ----------------------------
	CMatrixDouble33 cov;
	cov(0,0) = cov(1,1) = square(STD_XY);
	cov(2,2) = square(STD_PHI);

	std::vector<CPose2D>  real_poses;

	const size_t N_VERTEX = 8;

	for (unsigned j=0;j<N_VERTEX;j++)
	{
		CPose2D p(
			(cos( j*M_2PI/N_VERTEX)-1) * N_VERTEX,
			sin( j*M_2PI/N_VERTEX) * N_VERTEX,
			0 // 0.5*M_PI + (j+0.5)*M_2PI/N_VERTEX;
			);
		real_poses.push_back(p);
	}

	// Add the nodes to optimize
	for (unsigned j=0;j<N_VERTEX;j++)
		addNoisyEdge(j, (j+1)%N_VERTEX,  real_poses,graph_links,cov);

	addNoisyEdge(0,5,  real_poses,graph_links,cov);
	cout << graph_links.getEdge(0,5).mean << endl;
	//graph_links.edges.erase(make_pair(7,0));
		
	// ----------------------------
	// Do optimization:
	// ----------------------------
	// First pass: 
	//  With max_iters=0 -> Return un-optimized, Dijkstra coordinates
	mrpt::slam::optimizePoseGraph_levmarq(graph_links,optimal_poses_dijkstra,0);  

	// Second pass: The real optimization
	tictac.Tic();
	mrpt::slam::optimizePoseGraph_levmarq(graph_links,optimal_poses,300);
	cout << "Time: " << tictac.Tac() * 1000 << " ms" << endl;
	
	// Dump resulting poses:
	for (CNetworkOfPoses2D::type_global_poses::const_iterator i=optimal_poses_dijkstra.begin();i!=optimal_poses_dijkstra.end();++i)
		cout << "Node #" << i->first << " -> " << i->second.mean << endl;
	
	cout << endl;

	for (CNetworkOfPoses2D::type_global_poses::const_iterator i=optimal_poses.begin();i!=optimal_poses.end();++i)
		cout << "Node #" << i->first << " -> " << i->second.mean << endl;


	// Measure overall error:
	double avr_err = 0;
	for (CNetworkOfPoses2D::type_global_poses::const_iterator p=optimal_poses.begin();p!=optimal_poses.end();++p)
		avr_err += real_poses[p->first].sqrDistanceTo( p->second.mean );

	avr_err/=optimal_poses.size();
	avr_err = sqrt(avr_err);

	double avr_err_dijks  = 0;
	for (CNetworkOfPoses2D::type_global_poses::const_iterator p= optimal_poses_dijkstra.begin();p!=optimal_poses_dijkstra.end();++p)
		avr_err_dijks  += real_poses[p->first].sqrDistanceTo( p->second.mean );
	avr_err_dijks/=optimal_poses.size();
	avr_err_dijks = sqrt(avr_err_dijks);

	cout << " RMS error in XYZ for dijkstra poses: " << avr_err_dijks << endl;
	cout << " RMS error in XYZ for LM-optimized poses: " << avr_err << endl;


	// ----------------------------
	// Display results graphically:
	// ----------------------------
	CDisplayWindow3D	win("Consistent pose optimization: Result");
	CDisplayWindow3D	win2("Consistent pose optimization: Dijkstra-only estimation");

	COpenGLScenePtr	scene  = COpenGLScene::Create();
	COpenGLScenePtr	scene2 = COpenGLScene::Create();


	win.setCameraAzimuthDeg(90);
	win.setCameraElevationDeg(90);
	win.setCameraZoom(30);

	win2.setCameraAzimuthDeg(90);
	win2.setCameraElevationDeg(90);
	win2.setCameraZoom(30);

	scene->insert( mrpt::opengl::CGridPlaneXY::Create(-40,40,-40,40,0,1) );
	scene2->insert( mrpt::opengl::CGridPlaneXY::Create(-40,40,-40,40,0,1) );

	// Add one axis for each pose:
	for (CNetworkOfPoses2D::type_global_poses::const_iterator p=optimal_poses.begin();p!=optimal_poses.end();++p)
	{
		CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZ();
		obj->setName(format("%u",(unsigned)p->first));
		obj->enableShowName();
		obj->setScale(2.0);

		obj->setPose( p->second.mean );
		
		scene->insert(obj);

		// Add a small sphere for the real positions:
		CSpherePtr sph = CSphere::Create(0.4);
		sph->setPose( real_poses[p->first] );
		scene->insert(sph);
		scene2->insert(sph);

		// Non-optimal:
		CSetOfObjectsPtr obj2 = CSetOfObjects::Create();
		*obj2 = *obj;
		obj2->setName(format("%u",(unsigned)p->first));
		obj2->setPose( optimal_poses_dijkstra[p->first].mean );
		scene2->insert(obj2);
	}

	// Add links:
	for (CNetworkOfPoses2D::const_iterator ed=graph_links.begin();ed!=graph_links.end();++ed)
	{
		CPose2D p1 = optimal_poses[ ed->first.first ].mean;
		CPose2D p2 = optimal_poses[ ed->first.second ].mean;

		CSimpleLinePtr lin = CSimpleLine::Create();
		
		lin->setLineCoords(p1.x(),p1.y(),p1.z(),p2.x(),p2.y(),p2.z());
		lin->setLineWidth(3);
		lin->setColor(1,0,0);

		scene->insert(lin);
	}

	// Add links to "non-optimal" scene:
	for (CNetworkOfPoses2D::const_iterator ed=graph_links.begin();ed!=graph_links.end();++ed)
	{
		CPose2D p1 = optimal_poses_dijkstra[ ed->first.first ].mean;
		CPose2D p2 = optimal_poses_dijkstra[ ed->first.second ].mean;

		CSimpleLinePtr lin = CSimpleLine::Create();
		
		lin->setLineCoords(p1.x(),p1.y(),p1.z(),p2.x(),p2.y(),p2.z());
		lin->setLineWidth(3);
		lin->setColor(1,0,0);

		scene2->insert(lin);
	}
		

	win.get3DSceneAndLock() = scene;
	win.unlockAccess3DScene();
	win.forceRepaint();

	win2.get3DSceneAndLock() = scene2;
	win2.unlockAccess3DScene();
	win2.forceRepaint();

	win.setPos(10,10); win.resize(400,400);
	win2.setPos(420,10); win2.resize(400,400);

	win.waitForKey();
}

int main()
{
	try
	{
		TestGlobalOptimizationPosesGraph();

		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}

