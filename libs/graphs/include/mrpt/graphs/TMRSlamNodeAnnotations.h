#ifndef TMRSLAMNODEANNOTATIONS_H
#define TMRSLAMNODEANNOTATIONS_H

#include <mrpt/utils/types_simple.h>
#include <string>

namespace mrpt { namespace graphs { namespace detail {

/**\brief Struct to be used as the NODE_ANNOTATIONS template argument in
 * CNetworkOfPoses class instances for use in multiple-robot SLAM applications
 *
 * \ingroup mrpt_graphs_grp
 */
struct TMRSlamNodeAnnotations {

	/**\brief string identifier of the SLAM Agent that initially registered this node. */
	std::string agent_ID_str;
	/**\brief ID of node in the graph of the SLAM Agent that initially registered
	 * this node.
	 *
	 * \note Field is handy especially in cases where one SLAM agent communicates
	 * its local graph to another agent and we still want to keep track of the
	 * node ID in the former's graph.
	 */
	mrpt::utils::TNodeID nodeID_loc;

};

} } } // end of namespaces


#endif /* end of include guard: TMRSLAMNODEANNOTATIONS_H */
