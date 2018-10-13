/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/gui/CDisplayWindow3D.h>

#include "CWindowManager.h"

#include <string>
#include <map>

namespace mrpt::graphslam::detail
{
/**\brief Generic class for tracking the total number of edges for different
 * tpes of edges and for storing visualization-related information for each
 * type
 *
 * \ingroup mrpt_graphslam_grp
 */
class CEdgeCounter
{
   public:
	using iterator = std::map<std::string, int>::iterator;
	using const_iterator = std::map<std::string, int>::const_iterator;

	CEdgeCounter();
	~CEdgeCounter() = default;
	/**\brief Provide the instance with a CWindowManager.
	 */
	void setWindowManagerPtr(mrpt::graphslam::CWindowManager* win_manager);
	/**\brief State how many of the existing edges have been removed.
	 *
	 * Method is to be called after CNetworkOfPoses::collapseDuplicatedEdges
	 * method has been executed.
	 */
	void setRemovedEdges(int removed_edges);
	/**\brief Method for manually setting the number of loop closures
	 * registered so far.
	 */
	void setLoopClosureEdgesManually(int num_loop_closures);
	/**\brief Returns the edges that form loop closures in the current graph.
	 */
	int getLoopClosureEdges() const;
	/**\brief Return the total amount of registered edges.
	 * \sa getNumForEdgeType, getLoopClosureEdges
	 */
	int getTotalNumOfEdges() const;
	/**\brief Return the total amount of registered edges.
	 *
	 * \sa getNumForEdgeType, getLoopClosureEdges
	 */
	void getTotalNumOfEdges(int* total_num_edges) const;
	/**\brief Return the number of edges for the specified type.
	 *
	 * \exception std::exception If edge is not found
	 * \sa getTotalNumOfEdges
	 */
	int getNumForEdgeType(const std::string& name) const;
	/** Return the number of edges for the specified type.
	 *
	 * \exception std::exception If edge is not found
	 * \sa getTotalNumOfEdges
	 */
	void getNumForEdgeType(const std::string& name, int* total_num);
	/**\brief Set number of a specific edge type manually.
	 *
	 * Handy for not having to call addEdge multiple times in a row.
	 *
	 * \sa addEdge
	 */
	void setEdgesManually(const std::string& name, int num_of_edges);
	/**\brief Increment the number of edges for the specified type.
	 *
	 * \exception std::exception If edge exists and \b is_new is True
	 *
	 * \sa setEdgesManually
	 */
	void addEdge(
		const std::string& name, bool is_loop_closure = false,
		bool is_new = false);
	/**\brief Explicitly register a new edge type.
	 */
	void addEdgeType(const std::string& name);
	/**\brief Reset the state of the CEdgeCounter instance.
	 */
	void clearAllEdges();
	/**\brief Dump a report of the registered, so far, edges to the console.
	 *
	 * \sa getAsString
	 */
	void dumpToConsole() const;
	/**\brief Fill the provided string with a detailed report of the
	 * registered, so far, edges.
	 */
	void getAsString(std::string* str_out) const;
	/**\brief Return a detailed report of the registered, so far, edges in a
	 * string representation.
	 */
	std::string getAsString() const;

	// VISUALIZATION RELATED METHODS
	// ////////////////////////////

	/**\brief Add the textMessage parameters to the object
	 * All the names in the given std::maps have to be already
	 * specified and added in the object via addEdge with is_new=true or
	 * addEdgeType
	 *
	 * \exception std::exception If a name in the provided std::map doesn't
	 * already exist
	 */
	void setTextMessageParams(
		const std::map<std::string, double>& name_to_offset_y,
		const std::map<std::string, int>& name_to_text_index);

	/**\brief Handle the extra visualization parameters for the total number of
	 * edges and for loop closures and then passes execution to the other
	 * setTextMessageParams function.
	 */
	void setTextMessageParams(
		const std::map<std::string, double>& name_to_offset_y,
		const std::map<std::string, int>& name_to_text_index,
		const double& offset_y_total_edges, const int& text_index_total_edges,
		const double& offset_y_loop_closures,
		const int& text_index_loop_closures);

	/**\brief Instance Iterators */
	inline iterator begin() { return m_name_to_edges_num.begin(); }
	inline const_iterator cbegin() const
	{
		return m_name_to_edges_num.cbegin();
	}
	inline iterator end() { return m_name_to_edges_num.end(); }
	inline const_iterator cend() const { return m_name_to_edges_num.cend(); }

   private:
	/**\brief Update the given CDisplayWindow3D with the edges registered so
	 * far.
	 */
	void updateTextMessages() const;

	mrpt::gui::CDisplayWindow3D* m_win = nullptr;
	mrpt::graphslam::CWindowManager* m_win_manager = nullptr;

	/**\brief Map edge name <=> num of edges
	 *
	 * Tracking number of edges
	 */
	std::map<std::string, int> m_name_to_edges_num;
	int m_num_loop_closures;
	int m_unique_edges = 0;

	// visualization std::maps
	std::map<std::string, double> m_name_to_offset_y;
	std::map<std::string, int> m_name_to_text_index;

	bool m_has_read_textmessage_params;

	// specifics to loop closures, total edges
	bool m_display_total_edges = false;
	bool m_display_loop_closures = false;
	int m_offset_y_total_edges = 0;
	int m_offset_y_loop_closures = 0;
	int m_text_index_total_edges = 0;
	int m_text_index_loop_closures = 0;
};
}  // namespace mrpt::graphslam::detail
