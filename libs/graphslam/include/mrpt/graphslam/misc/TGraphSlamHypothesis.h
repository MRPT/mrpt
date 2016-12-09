/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef TGRAPHSLAMHYPOTHESIS_H
#define TGRAPHSLAMHYPOTHESIS_H

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/graphslam/misc/SensorSourceOfMeasurements.h>

#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

namespace mrpt { namespace graphslam { namespace detail {

/**\brief A graphSLAM Hypothesis.
 *
 * Struct practically provides a wrapper around the GRAPH_t::constraint_t instance.
 * Represents a hypothesis for a potential, perhaps loop closing, edge.
 *
 * \sa mrpt::deciders::CLoopCloserERD
 * \ingroup mrpt_graphslam_grp
 * (i.e. a graph constraint/edge)
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
struct TGraphSlamHypothesis {
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef typename GRAPH_t::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename constraint_t::type_value pose_t;
	/**\}*/
	/**\brief Constructor */
	TGraphSlamHypothesis();
	/**\brief Destructor */
	~TGraphSlamHypothesis();
	/**\brief Return a string representation of the object at hand
	 *
	 */
	/**\{*/
	std::string getAsString(bool oneline=true) const;
	void getAsString(std::string* str, bool oneline=true) const;
	/**\}*/

	/**\brief Getter methods for the underlying edge
	 *
	 * \sa setEdge
	 */
	/**\{*/
	void getEdge(constraint_t* edge) const;
	constraint_t getEdge() const;
	/**\}*/

	/**\brief Setter method for the underlying edge
	 *
	 * \sa getEdge
	 */
	void setEdge(const constraint_t& edge);

	/**\brief Getter methods for the inverse of the underlying edge */
	/**\{*/
	void getInverseEdge(constraint_t* edge) const;
	constraint_t getInverseEdge() const;
	/**\}*/

	/**\brief Reverse the hypothesis.
	 *
	 * Reversing implies, at least, changing order of from/to nodes and reversing
	 * the underlying edge
	 */
	void inverseHypothesis();

	/**\brief ID of the current hypothesis */
	size_t id;
	/**\brief Starting node of the hypothesis */
	mrpt::utils::TNodeID from;
	/**\brief Ending node of the hypothesis */
	mrpt::utils::TNodeID to;
	/**\brief Field that specifies if the hypothesis is to be considered */
	bool is_valid;
	/**\brief Goodness value corresponding to the hypothesis edge
	 *
	 * \note For ICP edges this resolves to the CICP goodness measure for the
	 * alignment operation. 
	 *
	 * \sa edge, mrpt::slam::CICP::goodness
	 */
	double goodness;
	// TODO
	/**\brief Indicates the sensor used to register the current edge */
	mrpt::graphslam::detail::SensorSourceOfMeasurements meas_source;

	/**\brief Compare the start and end nodes of two hypothesis
	 *
	 * \return True if ends match.
	 */
	bool sameEndsWith(const TGraphSlamHypothesis<GRAPH_t>& other) const;
	/**\brief Check if the start, end nodes are the ones given
	 *
	 * \return True if ends are the given ones
	 */
	bool hasEnds(
			const mrpt::utils::TNodeID from,
			const mrpt::utils::TNodeID to) const;

	private:
	/**\brief Edge connecting the two nodes */
	constraint_t edge;

};

	// TODO - not working
class HypothesisNotFoundException: public std::runtime_error {
 	public:
  	HypothesisNotFoundException(
  			mrpt::utils::TNodeID from,
  			mrpt::utils::TNodeID to):
  		runtime_error("Hypothesis between set of nodes was not found") {

				this->clear();
				m_to = to;
				m_from = from;

				std::stringstream ss;
    		ss << std::runtime_error::what() << ":\t" <<
    			"From = " << m_from << " | " <<
    			"To = " << m_to << std::endl;
				m_msg = ss.str();
				std::cout << "m_msg: " << m_msg << std::endl;
  		}
  	HypothesisNotFoundException(size_t id):
  		runtime_error("Hypothesis with the given ID was not found") {

  			this->clear();
  			m_id = id;

				std::stringstream ss;
    		ss << std::runtime_error::what() << ":\t" <<
    			"ID = " << m_id << std::endl;

				// TODO - When a HypohtesisNotFoundException is thrown, and what()
				// method is called from logFmt in grahpslam-engine catch statement, an
				// memory error is reported
				// `double free or corruption (!prev).`
				// what() output is not printed if I use the logger.logFmt method  but
				// IS printed if I used a printf call.
				MRPT_TODO("Double free or corruption error if HypothesisNotFoundException is raised.");
				std::cout << ss.str() << std::endl;
				m_msg = ss.str();
  		}

  	void clear() {
  		m_to = INVALID_NODEID;
  		m_from = INVALID_NODEID;
  		m_id = SIZE_MAX;
  		m_msg.clear();
  	}
  	~HypothesisNotFoundException() throw() {}
  	
		std::string getErrorMsg() const throw() {
  		return m_msg;
  	}

	const char* what() const throw() {
   		m_cnvt.str("");
    	m_cnvt << getErrorMsg();
    	return m_cnvt.str().c_str();
	}

 	private:
	mrpt::utils::TNodeID m_from;
	mrpt::utils::TNodeID m_to;

	/**\brief Hypothesis ID */
	size_t m_id;

	/**\brief Error message */
	std::string m_msg;
	static std::ostringstream m_cnvt;
};

std::ostringstream HypothesisNotFoundException::m_cnvt;

} } } // end of namespaces

#include "TGraphSlamHypothesis_impl.h"

#endif /* end of include guard: TGRAPHSLAMHYPOTHESIS_H */
