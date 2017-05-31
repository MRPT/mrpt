#ifndef TMRSLAMNODEANNOTATIONS_H
#define TMRSLAMNODEANNOTATIONS_H

#include <mrpt/graphs/TNodeAnnotations.h>
#include <mrpt/utils/types_simple.h>
#include <sstream>
#include <string>

namespace mrpt { namespace graphs { namespace detail {

/**\brief Struct to be used as the NODE_ANNOTATIONS template argument in
 * CNetworkOfPoses class instances for use in multiple-robot SLAM applications
 *
 * \ingroup mrpt_graphs_grp
 */
struct TMRSlamNodeAnnotations : public TNodeAnnotations {
	typedef TNodeAnnotations parent_t;
	typedef TMRSlamNodeAnnotations self_t;


	TMRSlamNodeAnnotations():
		agent_ID_str(""),
		nodeID_loc(INVALID_NODEID) { }

	TMRSlamNodeAnnotations(const TMRSlamNodeAnnotations& other):
		parent_t(other)
	{
		this->setAnnots(other);
	}

	TNodeAnnotations* getCopyOfAnnots() const {
		self_t* annots = new self_t(*this);
		return annots;
	}
	bool setAnnots(const parent_t& other) {
		parent_t::setAnnots(other);

		bool res;
		const self_t* mr_slam_annots = dynamic_cast<const self_t*>(&other);
		if (mr_slam_annots) {
			this->agent_ID_str = mr_slam_annots->agent_ID_str;
			this->nodeID_loc = mr_slam_annots->nodeID_loc;
			res = true;
		}
		else {
			res = false;
		}

		return res;
	}

	bool operator==(const TNodeAnnotations& other) const {
		const TMRSlamNodeAnnotations* mr_slam_annots =
			dynamic_cast<const TMRSlamNodeAnnotations*>(&other);

		bool res = false;
		if (mr_slam_annots) {
			res = (
					this->agent_ID_str == mr_slam_annots->agent_ID_str &&
					this->nodeID_loc == mr_slam_annots->nodeID_loc);
		}

		return res;
	}

	void getAnnotsAsString(std::string* s) const {
		parent_t::getAnnotsAsString(s);

		std::stringstream ss;
		ss << "agent_ID_str: " << agent_ID_str << "| "
			<< "nodeID_loc: " << nodeID_loc;

		s->clear();
		*s = ss.str();
	}

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

// declare as ttypename - in mrpt::utils namespace
namespace mrpt { namespace utils {

	MRPT_DECLARE_TTYPENAME(mrpt::graphs::detail::TMRSlamNodeAnnotations);

} } // end of namespaces

#endif /* end of include guard: TMRSLAMNODEANNOTATIONS_H */
