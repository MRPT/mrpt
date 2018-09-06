/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <cstdint>

#include <iostream>
#include <string>
#include <sstream>

namespace mrpt::graphs::detail
{
/**\brief An edge hypothesis between two nodeIDs.
 *
 * Struct practically provides a wrapper around the GRAPH_T::constraint_t
 * instance. Represents a hypothesis for a potential, perhaps loop closing,
 * edge (i.e. a graph constraint/edge), between two nodeIDs of the graph.
 *
 * \sa mrpt::deciders::CLoopCloserERD
 * \ingroup mrpt_graphs_grp
 */
template <class GRAPH_T>
struct THypothesis
{
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	using constraint_t = typename GRAPH_T::constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	using pose_t = typename constraint_t::type_value;
	using self_t = THypothesis<GRAPH_T>;
	/**\}*/
	/**\brief Constructor */
	THypothesis();
	/**\brief Destructor */
	~THypothesis();
	/**\brief Return a string representation of the object at hand
	 *
	 */
	/**\{*/
	std::string getAsString(bool oneline = true) const;
	void getAsString(std::string* str, bool oneline = true) const;
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
	 * Reversing implies, at least, changing order of from/to nodes and
	 * reversing
	 * the underlying edge
	 */
	void inverseHypothesis();

	/**\brief ID of the current hypothesis */
	size_t id;
	/**\brief Starting node of the hypothesis */
	mrpt::graphs::TNodeID from;
	/**\brief Ending node of the hypothesis */
	mrpt::graphs::TNodeID to;
	/**\brief Field that specifies if the hypothesis is to be considered */
	bool is_valid{true};
	/**\brief Goodness value corresponding to the hypothesis edge
	 *
	 * \note For ICP edges this resolves to the CICP goodness measure for the
	 * alignment operation.
	 *
	 * \sa edge, mrpt::slam::CICP::goodness
	 */
	double goodness{0};
	/**\brief Compare the start and end nodes of two hypothesis
	 *
	 * \return True if ends match.
	 */
	bool sameEndsWith(const self_t& other) const;
	/**\brief Check if the start, end nodes are the ones given
	 *
	 * \return True if ends are the given ones
	 */
	bool hasEnds(
		const mrpt::graphs::TNodeID from, const mrpt::graphs::TNodeID to) const;

	/**\brief Handy operator for using THypothesis in std::set
	 */
	bool operator<(const self_t& other) const;

	inline friend std::ostream& operator<<(
		std::ostream& o, const THypothesis<GRAPH_T>& h)
	{
		o << h.getAsString(/*oneline=*/true) << std::endl;
		return o;
	}

   private:
	/**\brief Edge connecting the two nodes */
	constraint_t edge;
};
}  // namespace mrpt::graphs::detail
#include "THypothesis_impl.h"
