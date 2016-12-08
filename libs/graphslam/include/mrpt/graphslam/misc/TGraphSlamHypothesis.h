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

#include <string>
#include <sstream>

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

	/**\brief ID of the current hypothesis */
	int id;
	/**\brief Starting node of the hypothesis */
	mrpt::utils::TNodeID from;
	/**\brief Ending node of the hypothesis */
	mrpt::utils::TNodeID to;

	/**\brief Edge connecting the two nodes */
	constraint_t edge;

	/**\brief Goodness value corresponding to the hypothesis edge
	 *
	 * \sa edge
	 */
	double goodness;
	/**\brief Field that specifies if the hypothesis is to be considered */
	bool is_valid;
	// TODO
	/**\brief Indicates the sensor used to register the current edge */
	mrpt::graphslam::detail::SensorSourceOfMeasurements meas_source;

	/**\brief Compare the start and end nodes of two hypothesis
	 *
	 * \return True if ends match.
	 */
	bool sameEndsWith(const TGraphSlamHypothesis<GRAPH_t>& other);
	/**\brief Check if the start, end nodes are the ones given
	 *
	 * \return True if ends are the given ones
	 */
	bool hasEnds(
			const mrpt::utils::TNodeID from,
			const mrpt::utils::TNodeID to);

};

} } } // end of namespaces

#include "TGraphSlamHypothesis_impl.h"

#endif /* end of include guard: TGRAPHSLAMHYPOTHESIS_H */
