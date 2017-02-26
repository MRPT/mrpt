/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphs/dijkstra.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;

// The type of my Dijkstra problem:
typedef CDijkstra<CNetworkOfPoses2D>   CMyDijkstra;   // See other options in mrpt::poses::CNetworkOfPoses<>

// Before MRPT 0.9.5 it was:
// typedef CDijkstra<CNetworkOfPoses2D::edge_t> CMyDijkstra;

// adds a new edge to the graph. The edge is annotated with the relative position of the two nodes
void addEdge(TNodeID from, TNodeID to, const aligned_containers<TNodeID,CPose2D>::map_t &real_poses,CNetworkOfPoses2D &graph_links)
{
	CPose2D p = real_poses.find(to)->second - real_poses.find(from)->second;
	graph_links.insertEdge(from,to, p );
}

// weight is the distance between two nodes.
double myDijkstraWeight(const CMyDijkstra::graph_t &g, const TNodeID from,const TNodeID to, const CMyDijkstra::edge_t& edge)
{
//	return 1;					// Topological distance
	return edge.norm();	// Metric distance
}

// ------------------------------------------------------
//				TestDijkstra
// ------------------------------------------------------
void TestDijkstra()
{
	CTicTac tictac;
	CNetworkOfPoses2D	graph_links;
	CNetworkOfPoses2D::global_poses_t optimal_poses, optimal_poses_dijkstra;
	aligned_containers<TNodeID,CPose2D>::map_t  real_poses;

	randomGenerator.randomize(10);

	// ----------------------------
	// Create a random graph:
	// ----------------------------
	const size_t N_VERTEX = 20;
	const double DIST_THRES = 10;
	const double NODES_XY_MAX = 15;

	vector<float>  xs,ys;

	for (size_t j=0;j<N_VERTEX;j++)
	{
		CPose2D p(
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-NODES_XY_MAX,NODES_XY_MAX),
			randomGenerator.drawUniform(-M_PI,M_PI) );
		real_poses[j] = p;

		// for the figure:
		xs.push_back(p.x());
		ys.push_back(p.y());
	}

	// Add some edges
	for (size_t i=0;i<N_VERTEX;i++)
	{
		for (size_t j=0;j<N_VERTEX;j++)
		{
			if (i==j) continue;
			if ( real_poses[i].distanceTo(real_poses[j]) < DIST_THRES )
				addEdge(i,j,real_poses,graph_links);
		}
	}

	// ----------------------------
	//  Dijkstra
	// ----------------------------
	tictac.Tic();
	const size_t SOURCE_NODE = 0;

	CMyDijkstra  myDijkstra(
		graph_links,
		SOURCE_NODE,
		&myDijkstraWeight);

	cout << "Dijkstra took " << tictac.Tac()*1e3 << " ms for " << graph_links.edges.size() << " edges." << endl;

	// Demo of getting the tree representation of
	//  the graph & visit its nodes:
	// ---------------------------------------------------------
	CMyDijkstra::tree_graph_t   graphAsTree;
	myDijkstra.getTreeGraph(graphAsTree);

	// Text representation of the tree:
	cout << "TREE:\n" << graphAsTree.getAsTextDescription() << endl;

	struct CMyVisitor : public CMyDijkstra::tree_graph_t::Visitor
	{
		virtual void OnVisitNode( const TNodeID parent, const CMyDijkstra::tree_graph_t::TEdgeInfo &edge_to_child, const size_t depth_level ) MRPT_OVERRIDE
		{
			cout << string(depth_level*3, ' ');
			cout << edge_to_child.id << endl;
		}
	};

	CMyVisitor myVisitor;

	cout << "Depth-first traverse of graph:\n";
	cout << SOURCE_NODE << endl;
	graphAsTree.visitDepthFirst( SOURCE_NODE, myVisitor );

	cout << endl << "Breadth-first traverse of graph:\n";
	cout << SOURCE_NODE << endl;
	graphAsTree.visitBreadthFirst( SOURCE_NODE, myVisitor );



	// ----------------------------
	// Display results graphically:
	// ----------------------------
	CDisplayWindowPlots	win("Dijkstra example");

	win.hold_on();
	win.axis_equal();

	for (TNodeID i=0;i<N_VERTEX && win.isOpen() ;i++)
	{
		if (i==SOURCE_NODE) continue;

		CMyDijkstra::edge_list_t	  path;

		myDijkstra.getShortestPathTo(i,path);

		cout << "to " << i << " -> #steps= " << path.size() << endl;

		win.setWindowTitle(format("Dijkstra path %u->%u",static_cast<unsigned int>(SOURCE_NODE),static_cast<unsigned int>(i) ));

		win.clf();

		// plot all edges:
		for (CNetworkOfPoses2D::iterator e= graph_links.begin();e!=graph_links.end();++e)
		{
			const CPose2D &p1 = real_poses[ e->first.first ];
			const CPose2D &p2 = real_poses[ e->first.second ];

			vector<float> X(2);
			vector<float> Y(2);
			X[0] = p1.x();  Y[0] = p1.y();
			X[1] = p2.x();  Y[1] = p2.y();
			win.plot(X,Y,"k1");
		}

		// Draw the shortest path:
		for (CMyDijkstra::edge_list_t::const_iterator a=path.begin();a!=path.end();++a)
		{
			const CPose2D &p1 = real_poses[ a->first];
			const CPose2D &p2 = real_poses[ a->second ];

			vector<float> X(2);
			vector<float> Y(2);
			X[0] = p1.x();  Y[0] = p1.y();
			X[1] = p2.x();  Y[1] = p2.y();
			win.plot(X,Y,"g3");
		}

		// Draw All nodes:
		win.plot(xs,ys,".b7");
		win.axis_fit(true);

		cout << "Press any key to show next shortest path, close window to end...\n";
		win.waitForKey();
	}

	win.clear();
}

int main()
{
	try
	{
		TestDijkstra();
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
