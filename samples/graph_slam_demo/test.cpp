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

// This determines the kind of graph and poses to use -------------------
typedef CNetworkOfPoses2D   my_graph_t;
//typedef CNetworkOfPoses3D   my_graph_t;
// ----------------------------------------------------------------------

// adds a new edge to the graph. The edge is annotated with the relative position of the two nodes
void addEdge(TNodeID from, TNodeID to, const my_graph_t::global_poses_t &real_poses,my_graph_t &graph)
{
	my_graph_t::edge_t RelativePose = real_poses.find(to)->second - real_poses.find(from)->second;
	graph.insertEdge(from,to, RelativePose );
}

vector_double  log_sq_err_evolution;

template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
void my_levmarq_feedback(
	const typename mrpt::graphslam::graphslam_traits<EDGE_TYPE,MAPS_IMPLEMENTATION>::graph_t &graph,
	const size_t iter,
	const size_t max_iter,
	const double cur_sq_error )
{
	log_sq_err_evolution.push_back(std::log(cur_sq_error));
	if ((iter % 100)==0)
		cout << "Progress: " << iter << " / " << max_iter << ", total sq err = " << cur_sq_error << endl;
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

	randomGenerator.randomize(123);

	// ----------------------------
	// Create a random graph:
	// ----------------------------
	const size_t N_VERTEX = 50;
	const double DIST_THRES = 7;
	const double NODES_XY_MAX = 20;

	// Level of noise in nodes initial positions:
	const double STD_NOISE_NODE_XYZ = 0.5;
	const double STD_NOISE_NODE_ANG = DEG2RAD(5);

	// Level of noise in edges:
	const double STD_NOISE_EDGE_XYZ = 0; //0.01;
	const double STD_NOISE_EDGE_ANG = 0; //DEG2RAD(0.1);


	for (TNodeID j=0;j<N_VERTEX;j++)
	{
#if 0  // Use random nodes
		CPose2D p(
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-M_PI,M_PI) );
#else
		// Use evenly distributed nodes:
		static double ang = 2*M_PI/N_VERTEX;
		const double R = NODES_XY_MAX + 2 * (j % 2 ? 1:-1);
		CPose2D p(
			R*cos(ang*j),
			R*sin(ang*j),
			ang);
#endif

		// Save real pose:
		real_node_poses[j] = p;

		// Copy the nodes to the graph, and add some noise:
		graph.nodes[j] = p;
	}


	// Add some edges
	for (TNodeID i=0;i<N_VERTEX;i++)
	{
		for (TNodeID j=i+1;j<N_VERTEX;j++)
		{
			if ( real_node_poses[i].distanceTo(real_node_poses[j]) < DIST_THRES )
				addEdge(i,j,real_node_poses,graph);
		}
	}

	// Cross-links:
	addEdge(0,N_VERTEX/2,real_node_poses,graph);

	// The root node (the origin of coordinates):
	graph.root = TNodeID(0);

	// This is the ground truth graph (make a copy for later use):
	const my_graph_t  graph_GT = graph;

	cout << "graph nodes: " << graph_GT.nodeCount() << endl;
	cout << "graph edges: " << graph_GT.edgeCount() << endl;

	// Add noise to edges & nodes:
	for (my_graph_t::edges_map_t::iterator itEdge=graph.edges.begin();itEdge!=graph.edges.end();++itEdge)
		itEdge->second += my_graph_t::edge_t( CPose3D(
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
			randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG) ) );

	for (my_graph_t::global_poses_t::iterator itNode=graph.nodes.begin();itNode!=graph.nodes.end();++itNode)
		if (itNode->first!=graph.root)
			itNode->second += my_graph_t::edge_t::type_value( CPose3D(
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_NODE_ANG) ) );


	// This is the initial input graph (make a copy for later use):
	const my_graph_t  graph_initial = graph;
//	graph_GT.saveToTextFile("test_GT.graph");
//	graph_initial.saveToTextFile("test.graph");

	// ----------------------------
	//  Run graph slam:
	// ----------------------------
	TParametersDouble  params;
	//params["verbose"]  = 1;
	params["profiler"] = 1;
	//params["max_iterations"] = 100000;
	params["max_iterations"] = 10000;

	graphslam::TResultInfoSpaLevMarq  levmarq_info;

	graphslam::optimize_graph_spa_levmarq(graph, levmarq_info, NULL, params, &my_levmarq_feedback<my_graph_t::constraint_t,my_graph_t::maps_implementation_t>);

	// ----------------------------
	//  Display results:
	// ----------------------------
	CDisplayWindow3D  win("graph-slam demo results");
	CDisplayWindow3D  win2("graph-slam demo initial state");

	// The final optimized graph:
	TParametersDouble  graph_render_params1;
	graph_render_params1["show_edges"] = 1;
	graph_render_params1["edge_width"] = 1;
	graph_render_params1["nodes_corner_scale"] = 1;
	CSetOfObjectsPtr gl_graph1 = mrpt::opengl::graph_tools::graph_visualize(graph, graph_render_params1 );

	// The initial noisy graph:
	TParametersDouble  graph_render_params2;
	graph_render_params2["show_ground_grid"] = 0;
	graph_render_params2["show_edges"] = 0;
	graph_render_params2["show_node_corners"] = 0;
	graph_render_params2["nodes_point_size"]  = 7;

	CSetOfObjectsPtr gl_graph2 = mrpt::opengl::graph_tools::graph_visualize(graph_initial, graph_render_params2 );

	// The ground truth graph:
	TParametersDouble  graph_render_params3;
	graph_render_params3["show_ground_grid"] = 0;
	graph_render_params3["show_ID_labels"] = 1;
	graph_render_params3["show_edges"] = 1;
	graph_render_params3["edge_width"] = 3;
	graph_render_params3["nodes_corner_scale"] = 2;
	CSetOfObjectsPtr gl_graph3 = mrpt::opengl::graph_tools::graph_visualize(graph_GT, graph_render_params3 );
	CSetOfObjectsPtr gl_graph4 = mrpt::opengl::graph_tools::graph_visualize(graph_initial, graph_render_params3 );


	win.addTextMessage(5,5   , "Ground truth: Big corners & thick edges", TColorf(0,0,0), 1000 /* arbitrary, unique text label ID */, MRPT_GLUT_BITMAP_HELVETICA_12 );
	win.addTextMessage(5,5+15, "Initial graph: Gray thick points.", TColorf(0,0,0), 1001 /* arbitrary, unique text label ID */, MRPT_GLUT_BITMAP_HELVETICA_12 );
	win.addTextMessage(5,5+30, "Final graph: Small corners & thin edges", TColorf(0,0,0), 1002 /* arbitrary, unique text label ID */, MRPT_GLUT_BITMAP_HELVETICA_12 );

	win2.addTextMessage(5,5   , "Ground truth: Big corners & thick edges", TColorf(0,0,0), 1000 /* arbitrary, unique text label ID */, MRPT_GLUT_BITMAP_HELVETICA_12 );
	win2.addTextMessage(5,5+15, "Initial graph: Small corners & thin edges", TColorf(0,0,0), 1001 /* arbitrary, unique text label ID */, MRPT_GLUT_BITMAP_HELVETICA_12 );

	{
		COpenGLScenePtr &scene = win.get3DSceneAndLock();
		scene->insert(gl_graph1);
		scene->insert(gl_graph3);
		scene->insert(gl_graph2);
		win.unlockAccess3DScene();
		win.repaint();
	}

	{
		COpenGLScenePtr &scene = win2.get3DSceneAndLock();
		scene->insert(gl_graph3);
		scene->insert(gl_graph4);
		win2.unlockAccess3DScene();
		win2.repaint();
	}

	// Show progress of error:
	CDisplayWindowPlots  win_err("Evolution of log(sq. error)");
	win_err.plot(log_sq_err_evolution,"-b");
	win_err.axis_fit();


	// wait end:
	cout << "Close any window to end...\n";
	while (win.isOpen() && win2.isOpen() && win_err.isOpen())
	{
		mrpt::system::sleep(10);
	}
}

int main()
{
	try
	{
		GraphSLAMDemo();
		mrpt::system::sleep(20);
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
