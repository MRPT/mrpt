#include <mrpt/poses.h>
#include <mrpt/opengl.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::opengl;

int main()
{
	CTimeLogger  tims;

	tims.enter("load graph");

#if 1
	CNetworkOfPoses2DInf graph;
	graph.loadFromTextFile("/code/TORO/data/2D/w10000-odom.graph");// "/home/jlblanco/code/toro-svn/data/2D/w10000-odom.graph");
	//graph.saveToTextFile("./saved_2d.graph"); return 0;
#endif

#if 0
	CNetworkOfPoses3DInf graph;
	graph.loadFromTextFile("/home/jlblanco/code/toro-svn/data/3D/sphere_smallnoise.graph");
	//graph.loadFromTextFile("/home/jlblanco/code/toro-svn/data/3D/sphere_mednoise.graph");
	//graph.saveToTextFile("./saved_3d.graph"); return 0;
#endif

	tims.leave("load graph");


	tims.enter("dijkstra");
	graph.dijkstra_nodes_estimate();
	tims.leave("dijkstra");

	cout << "edges        : " << graph.edgeCount() << endl;
	cout << "nodes        : " << graph.nodeCount() << endl;
	cout << "# diff nodes : " << graph.countDifferentNodesInEdges() << endl;

	cout << graph.getEdge(11,10) << endl;
	cout << graph.nodes[10] << endl;


	TParametersDouble params;
	//params["nodes_point_size"] = 4;
	//params["show_node_corners"] = 0;
	params["show_ID_labels"] = 1;
	params["nodes_corner_scale"] = 1.0;
	params["edge_width"] = 1.0;

	CSetOfObjectsPtr gl_graph = graph_tools::graph_visualize(graph,params);

	mrpt::gui::CDisplayWindow3D  win("Graph", 500,400);

	COpenGLScenePtr &scene = win.get3DSceneAndLock();
	scene->insert(gl_graph);
	win.unlockAccess3DScene();
	win.repaint();

	win.waitForKey();

	return 0;
}

