/** \defgroup mrpt_graphs_grp [mrpt-graphs]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-graphs</code></h2>
<hr>

Graph-related stuff: generic directed graphs (mrpt::graphs::CDirectedGraph) and trees (mrpt::graphs::CDirectedTree).

Graphs of pose constraints are also defined in this library, via a generic template mrpt::graphs::CNetworkOfPoses, capable
of reading and writing to both binary and <a href="http://www.mrpt.org/Robotics_file_formats" >text pose-graph file</a> formats.
Predefined typedefs exist for:
 - mrpt::graphs::CNetworkOfPoses2D     -> 2D graphs (x,y,phi), covariance matrix.
 - mrpt::graphs::CNetworkOfPoses3D     -> 3D graphs (x,y,z,yaw,pitch,roll), covariance matrix.
 - mrpt::graphs::CNetworkOfPoses2DInf  -> 2D graphs (x,y,phi), inverse covariance (information) matrix.
 - mrpt::graphs::CNetworkOfPoses3DInf  -> 3D graphs (x,y,z,yaw,pitch,roll), inverse covariance (information) matrix.



*/

