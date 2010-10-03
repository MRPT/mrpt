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

#include <mrpt/slam/graph_slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;

// This determines the kind of graph and poses to use:
typedef CNetworkOfPoses2D   my_graph_t;
//typedef CNetworkOfPoses3D   my_graph_t;

// adds a new edge to the graph. The edge is annotated with the relative position of the two nodes
void addEdge(TNodeID from, TNodeID to, const my_graph_t::global_poses_t &real_poses,my_graph_t &graph)
{
	my_graph_t::edge_t RelativePose = real_poses.find(to)->second - real_poses.find(from)->second;

	// Add noise to edge data:
	// ...

	graph.insertEdge(from,to, RelativePose );
}

// ------------------------------------------------------
//				GraphSLAMDemo
// ------------------------------------------------------
void GraphSLAMDemo()
{
	// The graph: nodes + edges:
	my_graph_t  graph;

	// The global poses of each node (without covariance):
	my_graph_t::global_poses_t  real_node_poses;

	randomGenerator.randomize(10);

	// ----------------------------
	// Create a random graph:
	// ----------------------------
	const size_t N_VERTEX = 20;
	const double DIST_THRES = 10;
	const double NODES_XY_MAX = 15;

	vector_float  xs,ys; // for the GUI

	for (TNodeID j=0;j<N_VERTEX;j++)
	{
		CPose2D p(
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-M_PI,M_PI) );

		// Save real pose:
		real_node_poses[j] = p;

		// Copy the nodes to the graph, and add some noise:
		graph.nodes[j] = p;
		// ...

		// for the figure:
		xs.push_back(p.x());
		ys.push_back(p.y());
	}


	// Add some edges
	for (TNodeID i=0;i<N_VERTEX;i++)
	{
		for (TNodeID j=0;j<N_VERTEX;j++)
		{
			if (i==j) continue;
			if ( real_node_poses[i].distanceTo(real_node_poses[j]) < DIST_THRES )
				addEdge(i,j,real_node_poses,graph);
		}
	}

	// ----------------------------
	//  Run graph slam:
	// ----------------------------
	TParametersDouble  params;
	params["verbose"]  = 1;
	params["profiler"] = 1;

	graphslam::optimize_graph_spa_levmarq(graph, NULL, params);




}

int main()
{
	try
	{
		GraphSLAMDemo();
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
