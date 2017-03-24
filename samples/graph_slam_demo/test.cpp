/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/random.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/system/threads.h> // sleep()
#include <mrpt/gui.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/graph_tools.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::random;
using namespace std;


// Level of noise in nodes initial positions:
const double STD_NOISE_NODE_XYZ = 0.5;
const double STD_NOISE_NODE_ANG = DEG2RAD(5);

// Level of noise in edges:
const double STD_NOISE_EDGE_XYZ = 0.001;
const double STD_NOISE_EDGE_ANG = DEG2RAD(0.01);

const double STD4EDGES_COV_MATRIX       = 10;
const double ERROR_IN_INCOMPATIBLE_EDGE = 0.3; // ratio [0,1]


/** 
 * Generic struct template
 * Auxiliary class to add a new edge to the graph. The edge is annotated with the relative position of the two nodes
 */
template <class GRAPH,bool EDGES_ARE_PDF = GRAPH::edge_t::is_PDF_val> struct EdgeAdders;

/**
 * Specific templates based on the above EdgeAdders template
 * Non-PDF version:
 */
template <class GRAPH> struct EdgeAdders<GRAPH,false>
{
	static const int DIM = GRAPH::edge_t::type_value::static_size;
	typedef CMatrixFixedNumeric<double,DIM,DIM> cov_t;

	static void addEdge(TNodeID from, TNodeID to, const typename GRAPH::global_poses_t &real_poses,GRAPH &graph, const cov_t &COV_MAT)
	{
    /** 
     * No covariance argument here (although it is passed in the function
     * declaration above)
     * See also : https://groups.google.com/d/msg/mrpt-users/Sr9LSydArgY/vRNM5V_uA-oJ 
     * for a more detailed explanation on how it is treated
     */
		typename GRAPH::edge_t RelativePose(
        real_poses.find(to)->second - real_poses.find(from)->second);
		graph.insertEdge(from,to, RelativePose );
	}
};
// PDF version:
template <class GRAPH> struct EdgeAdders<GRAPH,true>
{
	static const int DIM = GRAPH::edge_t::type_value::static_size;
	typedef CMatrixFixedNumeric<double,DIM,DIM> cov_t;

	static void addEdge(TNodeID from, TNodeID to, const typename GRAPH::global_poses_t &real_poses,GRAPH &graph, const cov_t &COV_MAT)
	{
		typename GRAPH::edge_t RelativePose(
        real_poses.find(to)->second - real_poses.find(from)->second, 
        COV_MAT);
		graph.insertEdge(from,to, RelativePose );
	}
};

// Container to handle the propagation of the square root error of the problem
vector<double>  log_sq_err_evolution;

// This example lives inside this template class, which can be instanced for different kind of graphs (see main()):
template <class my_graph_t>
struct ExampleDemoGraphSLAM
{
	template <class GRAPH_T>
	static void my_levmarq_feedback(
		const GRAPH_T &graph,
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
	void run(bool add_extra_tightening_edge)
	{
		// The graph: nodes + edges:
		my_graph_t  graph;

		// The global poses of the graph nodes (without covariance):
		typename my_graph_t::global_poses_t  real_node_poses;

    /**
     * Initialize the PRNG from the given random seed.
     * Method used to initially randomise the generator
     */
		randomGenerator.randomize(123);

		// ----------------------------
		// Create a random graph:
		// ----------------------------
		const size_t N_VERTEX = 50;
		const double DIST_THRES = 7;
		const double NODES_XY_MAX = 20;

    /**
     * First add all the nodes (so that, when I add edges, I can refer to them
     */
		for (TNodeID j=0;j<N_VERTEX;j++)
		{
			// Use evenly distributed nodes:
			static double ang = 2*M_PI/N_VERTEX;
			const double R = NODES_XY_MAX + 2 * (j % 2 ? 1:-1);
			CPose2D p(
				R*cos(ang*j),
				R*sin(ang*j),
				ang);

			// Save real pose:
			real_node_poses[j] = p;

			// Copy the nodes to the graph, and add some noise:
			graph.nodes[j] = p;
		}


		/** 
     * Add some edges
     * Also initialize the information matrix used for EACH constraint. For
     * simplicity the same information matrix is used for each one of the edges. 
     * This information matrix is RELATIVE to each constraint/edge (not in
     * global ref. frame)
     * see also: https://groups.google.com/d/msg/mrpt-users/Sr9LSydArgY/wYFeU2BXr4kJ 
     */
		typedef EdgeAdders<my_graph_t> edge_adder_t;
		typename edge_adder_t::cov_t   inf_matrix;
		inf_matrix.unit(edge_adder_t::cov_t::RowsAtCompileTime, square(1.0/STD4EDGES_COV_MATRIX));

    /**
     * add the edges using the node ids added to the graph before
     */
		for (TNodeID i=0;i<N_VERTEX;i++)
		{
			for (TNodeID j=i+1;j<N_VERTEX;j++)
			{
				if ( real_node_poses[i].distanceTo(real_node_poses[j]) < DIST_THRES )
					edge_adder_t::addEdge(i,j,real_node_poses,graph,inf_matrix);
			}
		}

		// Add an additional edge to deform the graph?
		if (add_extra_tightening_edge)
		{
			//inf_matrix.unit(square(1.0/(STD4EDGES_COV_MATRIX)));
			edge_adder_t::addEdge(0,N_VERTEX/2,real_node_poses,graph,inf_matrix);

			// Tweak this last node to make it incompatible with the rest:
			// It must exist, don't check errors...
			typename my_graph_t::edge_t &ed = graph.edges.find(make_pair(TNodeID(0), TNodeID(N_VERTEX/2) ))->second;
			ed.getPoseMean().x( (1-ERROR_IN_INCOMPATIBLE_EDGE) * ed.getPoseMean().x() );
		}

		// The root node (the origin of coordinates):
		graph.root = TNodeID(0);

		// This is the ground truth graph (make a copy for later use):
		const my_graph_t  graph_GT = graph;

		cout << "graph nodes: " << graph_GT.nodeCount() << endl;
		cout << "graph edges: " << graph_GT.edgeCount() << endl;

		// Add noise to edges & nodes:
		for (typename my_graph_t::edges_map_t::iterator itEdge=graph.edges.begin();itEdge!=graph.edges.end();++itEdge)
		{
			const typename my_graph_t::edge_t::type_value delta_noise(CPose3D(
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_XYZ),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG),
				randomGenerator.drawGaussian1D(0,STD_NOISE_EDGE_ANG) ));
			itEdge->second.getPoseMean() += typename my_graph_t::edge_t::type_value(delta_noise);
		}


		for (typename my_graph_t::global_poses_t::iterator itNode=graph.nodes.begin();itNode!=graph.nodes.end();++itNode)
			if (itNode->first!=graph.root)
				itNode->second.getPoseMean() += typename my_graph_t::edge_t::type_value( CPose3D(
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
		params["max_iterations"] = 500;
		params["scale_hessian"] = 0.1;  // If <1, will "exagerate" the scale of the gradient and, normally, will converge much faster.
		params["tau"] = 1e-3;

		// e2: Lev-marq algorithm iteration stopping criterion #2: |delta_incr| < e2*(x_norm+e2)
//		params["e1"] = 1e-6;
//		params["e2"] = 1e-6;

		graphslam::TResultInfoSpaLevMarq  levmarq_info;
		log_sq_err_evolution.clear();

		cout << "Global graph RMS error / edge = " << std::sqrt(graph.getGlobalSquareError(false)/graph.edgeCount()) << endl;
		cout << "Global graph RMS error / edge = " << std::sqrt(graph.getGlobalSquareError(true)/graph.edgeCount()) << " (ignoring information matrices)." << endl;

		// Do the optimization
		graphslam::optimize_graph_spa_levmarq(
			graph,
			levmarq_info,
			NULL,  // List of nodes to optimize. NULL -> all but the root node.
			params,
			&my_levmarq_feedback<my_graph_t>);

		cout << "Global graph RMS error / edge = " << std::sqrt(graph.getGlobalSquareError(false)/graph.edgeCount()) << endl;
		cout << "Global graph RMS error / edge = " << std::sqrt(graph.getGlobalSquareError(true)/graph.edgeCount()) << " (ignoring information matrices)." << endl;

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

		graph_render_params2["nodes_point_size"]  = 5;
		CSetOfObjectsPtr gl_graph5 = mrpt::opengl::graph_tools::graph_visualize(graph, graph_render_params2 );

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
			scene->insert(gl_graph5);
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
		while (win.isOpen() && win2.isOpen() && win_err.isOpen() && !mrpt::system::os::kbhit())
		{
			mrpt::system::sleep(10);
		}
	}
};

int main()
{
	try
	{
//typedef CNetworkOfPoses<CPose2D,map_traits_map_as_vector>   my_graph_t;

		cout << "Select the type of graph to optimize:\n"
			"1.  CNetworkOfPoses2D \n"
			"2.  CNetworkOfPoses2DInf \n"
			"3.  CNetworkOfPoses3D \n"
			"4.  CNetworkOfPoses3DInf \n";

		cout << ">> ";

		int i=0;
		{
			string l;
			std::getline(cin,l);
			l=mrpt::system::trim(l);
			i = atoi(&l[0]);
		}

		bool add_extra_tightening_edge;
		cout << "Add an extra, incompatible tightening edge? [y/N] ";
		{
			string l;
			std::getline(cin,l);
			l=mrpt::system::trim(l);
			add_extra_tightening_edge = l[0]=='Y' || l[0]=='y';
		}


		switch(i)
		{
		case 1:	{
				ExampleDemoGraphSLAM<CNetworkOfPoses2D>  demo;
				demo.run(add_extra_tightening_edge);
			} break;
		case 2:	{
				ExampleDemoGraphSLAM<CNetworkOfPoses2DInf>  demo;
				demo.run(add_extra_tightening_edge);
			} break;
		case 3:	{
				ExampleDemoGraphSLAM<CNetworkOfPoses3D>  demo;
				demo.run(add_extra_tightening_edge);
			} break;
		case 4:	{
				ExampleDemoGraphSLAM<CNetworkOfPoses3DInf>  demo;
				demo.run(add_extra_tightening_edge);
			} break;
		};

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
