#ifndef TMRSLAMEDGEANNOTATIONS_H
#define TMRSLAMEDGEANNOTATIONS_H

namespace mrpt { namespace graphs { namespace detail {

/**\brief Struct to be used as the EDGE_ANNOTATIONS template argument in
 * CNetworkOfPoses class instances for use in multiple-robot SLAM applications
 *
 * \ingroup mrpt_graphs_grp
 */
struct TMRSlamEdgeAnnotations {

	/**\brief Indicate whether this edge is a connection between nodes that have
	 * been registered by two different SLAM agents
	 */
	bool is_interconnecting_edge;

};

} } } // end of namespaces

#endif /* end of include guard: TMRSLAMEDGEANNOTATIONS_H */
