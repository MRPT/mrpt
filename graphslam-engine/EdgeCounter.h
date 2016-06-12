/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef EDGECOUNTER_H
#define EDGECOUNTER_H



#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

/**
 * Generic class for tracking the total number of edges for different tpes of
 * edges and for storing visualization-related information for each type
 *
 */
class EdgeCounter_t {
	public:

		EdgeCounter_t(mrpt::gui::CDisplayWindow3D* win = NULL) {
			m_win = win;
			initEdgeCounter_t();
		}
		~EdgeCounter_t() {}

		void initEdgeCounter_t() {

			this->clearAllEdges();
			m_has_read_textmessage_params = false;

			// default font-related variables
			m_font_name = "Sans";
			m_font_size = 20;

			// visualization parameters for total edges / loop closures
			m_display_total_edges = false; m_display_loop_closures = false;
			m_offset_y_total_edges = 0.0; m_offset_y_loop_closures = 0.0;
			m_text_index_total_edges = 0; m_text_index_loop_closures = 0;

			m_unique_edges = 0;

		}

		/**
		 * setRemovedEdges
		 *
		 * Method to be called after call to CNetworkOfPoses::collapseDuplicatedEdges 
		 * method. States how many of the existing edges have been removed.
		 */
		void setRemovedEdges(int removed_edges) { 
			m_unique_edges = this->getTotalNumOfEdges() - removed_edges; 
			//std::cout << "setRemovedEdges: Unique edges: " << m_unique_edges << std::endl;
		}
		/**
		 * setLoopClosureEdgesManually()
		 *
		 * Method for manually setting the number of loop closures registered so
		 * far
		 */
		void setLoopClosureEdgesManually(int num_loop_closures) {
			m_num_loop_closures = num_loop_closures;
		}
		/**
		 * getLoopClosureEdges()
		 *
		 * Returns the edges that form loop closures in the current graph
		 */
		int getLoopClosureEdges() const {
			return m_num_loop_closures;
		}
		/**
		 * Return the total amount of registered edges
		 * \sa getTotalNumOfEdges(int* total_num_edges)
		 */
		int getTotalNumOfEdges() const {
			int sum = 0;

			for (std::map<std::string, int>::const_iterator it = m_name_to_edges_num.begin();
					it != m_name_to_edges_num.end(); ++it) {
				sum += it->second;
			}
			return sum;
		}
		/**
		 * Return the total amount of registered edges
		 * \sa getTotalNumOfEdges()
		 */
		int getTotalNumOfEdges(int* total_num_edges) const {
			int sum = 0;

			for (std::map<std::string, int>::const_iterator it = m_name_to_edges_num.begin();
					it != m_name_to_edges_num.end(); ++it) {
				sum += it->second;
			}
			*total_num_edges = sum;
		}
		/**
		 * getNumForEdgeType
		 *
		 * Return the number of edges for the specified type
		 * \sa getNumForEdgeType(const std::string& name, int* total_num)
		 */
		int getNumForEdgeType(const std::string& name) const {
			std::map<std::string, int>::const_iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				return search->second;
			}
			else {
				THROW_EXCEPTION("No edge with such name exists")
			}
		}
		/**
		 * getNumForEdgeType
		 *
		 * Return the number of edges for the specified type. If edge is not found,
		 * throw an exception
		 *
		 * \sa getNumForEdgeType(const std::string& name)
		 */
		void getNumForEdgeType(const std::string& name, int* total_num) {
			std::map<std::string, int>::const_iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				*total_num = search->second;
			}
			else {
				THROW_EXCEPTION("No edge with such name exists")
			}
		}
		/**
		 * setEdgesManually
		 *
		 * Set number of a specific edge type manually. Handy for not having to
		 * call addEdge multiple times in a row.
		 */
		void setEdgesManually(const std::string& name, int num_of_edges) {
			std::map<std::string, int>::iterator search = m_name_to_edges_num.find(name);
			std::cout << "Setting edges manually" << std::endl;
			if ( search != m_name_to_edges_num.end() ) {
				search->second = num_of_edges;
			}
			else {
				std::string str_err = "No edge with such name exists.";
				THROW_EXCEPTION(str_err)
			}
			// Update the visualization if the user has already set the vizualization
			// parameters
			if (m_has_read_textmessage_params && m_win) {
				updateTextMessages();
			}
		}
		/**
		 * addEdge
		 *
		 * Increment the number of edges for the specified type.
		 * If edge exists and is_new is_also true, throw exception
		 */
		void addEdge(const std::string& name, bool is_loop_closure=false, bool is_new=false) {
			std::map<std::string, int>::iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				(search->second)++; // increment to the found element

				// specify warning if is_new = true
				if (is_new) {
					std::string str_err = "Specified edge type already exists but is_new is also specified!";
					THROW_EXCEPTION(str_err)
						//std::stringstream ss_warn;
						//ss_warn << "Commencing with the increment normally" << std::endl;
						//MRPT_WARNING(ss_warn.str())
				}
				if (is_loop_closure) {
					// throw if user has also specified the is new boolean flag
					if (is_new) {
						std::string str_err = "Both is_new and is_loop_closure flags are true. Exiting...";
						THROW_EXCEPTION(str_err)
					}
					m_num_loop_closures++;
				}
			}
			else {
				if (is_new) {
					m_name_to_edges_num[name] = 1;
				}
				else {
					std::string str_err = "No edge with such name exists. Specify is_new parameter if you want to add it";
					THROW_EXCEPTION(str_err)
				}
			}

			// Update the visualization if the user has already set the vizualization
			// parameters
			if (m_has_read_textmessage_params && m_win) {
				updateTextMessages();
			}
		}
		/**
		 * addEdgeType
		 *
		 * Explicitly register a new edge type.
		 */
		void addEdgeType(const std::string& name) {
			std::map<std::string, int>::const_iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				THROW_EXCEPTION(mrpt::format("Specified edge type %s already exists", name.c_str()))
			}
			else {
				m_name_to_edges_num[name] = 0;
			}
		}
		/**
		 * clearAllEdges
		 *
		 * Bring the class instance to an empty state
		 */
		void clearAllEdges() {
			m_num_loop_closures = 0;

			m_name_to_edges_num.clear();
			m_name_to_offset_y.clear();
			m_name_to_text_index.clear();

			m_has_read_textmessage_params = false;
			m_display_total_edges					= false;
			m_display_loop_closures				= false;
		}

		/**
		 * printEdgesSummary
		 *
		 * Dump a detailed report for all the edges registered thus far
		 */
		void printEdgesSummary() const {
			std::stringstream ss_out;
			ss_out << "Summary of Edges: " << std::endl;
			ss_out << "---------------------------" << std::endl;

			ss_out << "\t Total edges: " << this->getTotalNumOfEdges() << std::endl;

			for (std::map<std::string, int>::const_iterator it = m_name_to_edges_num.begin();
					it != m_name_to_edges_num.end(); ++it) {
				ss_out << "\t " << it->first << " edges: " << it->second << std::endl;
			}
			ss_out << "\t Loop closure edges: " << this->getLoopClosureEdges() << std::endl;

			std::cout << ss_out.str() << std::endl;
		}

		// VISUALIZATION RELATED METHODS
		// ////////////////////////////

		/**
		 * setVisualizationWindow
		 *
		 * Add the visualization window. Handy function for not having to
		 * specify it in the class constructor
		 */
		void setVisualizationWindow(mrpt::gui::CDisplayWindow3D* win) { m_win = win; }

		/**
		 * setTextMessageParams
		 *
		 * Add the textMessage parameters to the object - used during visualization
		 * All the names in the given std::maps have to be already specified and added in
		 * the object via addEdge with is_new=true or addEdgeType
		 */
		void setTextMessageParams(const std::map<std::string, double>& name_to_offset_y,
				const std::map<std::string, int>& name_to_text_index,
				const std::string& font_name, const int& font_size) {

			//std::cout << "in setTextMessageParams " << std::endl
			//<< "m_offset_y_total_edges: " << m_offset_y_total_edges << std::endl
			//<< "m_text_index_total_edges: " << m_text_index_total_edges << std::endl;
			//std::cout << "in setTextMessageParams:  " << std::endl
			//<< "m_offset_y_loop_closures: " << m_offset_y_loop_closures << std::endl
			//<< "m_text_index_loop_closures" << m_text_index_loop_closures << std::endl;

			assert(m_win &&
					"Visualization of data was requested but no CDisplayWindow pointer was given");
			assert(name_to_offset_y.size() == name_to_text_index.size());

			for (std::map<std::string, double>::const_iterator it = name_to_offset_y.begin();
					it != name_to_offset_y.end(); ++it) {
				std::string name = it->first;

				// check if name already exist, otherwise throw exception
				std::map<std::string, int>::const_iterator search = m_name_to_edges_num.find(name);
				if ( search == m_name_to_edges_num.end() ) {
					std::stringstream ss_err;
					ss_err << "Name " << name << " is not recognized as an Edge type." << std::endl;
					THROW_EXCEPTION(ss_err.str())
				}
				// name exists ...

				double offset_y = it->second;
				int text_index = name_to_text_index.find(name)->second;

				//std::cout << "in setTextMessageParams: " << std::endl;
				//std::cout << "name: " << name << " | offset_y: " << offset_y << " | text_index: " << text_index << std::endl;

				m_name_to_offset_y[name] = offset_y;
				m_name_to_text_index[name] = text_index;

			}

			// font parameters
			m_font_name = font_name;
			m_font_size = font_size;

			m_has_read_textmessage_params = true;
		}

		/**
		 * setTextMessageParams
		 *
		 * Handles the extra visualization parameters for the total number of edges
		 * and for loop closures and then passes execution to the other
		 * setTextMessageParams function.
		 */
		void setTextMessageParams(const std::map<std::string, double>& name_to_offset_y,
				const std::map<std::string, int>& name_to_text_index,
				const double& offset_y_total_edges, const int& text_index_total_edges,
				const double& offset_y_loop_closures, const int& text_index_loop_closures,
				const std::string& font_name, const int& font_size) {

			// set the parameters for total edges / loop closures
			m_display_total_edges = true;
			m_display_loop_closures = true;

			m_offset_y_total_edges = offset_y_total_edges;
			m_offset_y_loop_closures = offset_y_loop_closures;

			m_text_index_total_edges = text_index_total_edges;
			m_text_index_loop_closures = text_index_loop_closures;

			// pass execution to the other setTextMessageParams
			this->setTextMessageParams(name_to_offset_y, name_to_text_index,
					font_name, font_size);

		}

		/**
		 * updateTextMessages
		 *
		 * Updates the given CDisplayWindow3D with the edges registered so far.
		 */
		void updateTextMessages() const {
			assert(m_win);
			assert(m_has_read_textmessage_params);
			assert(m_name_to_offset_y.size() == m_name_to_text_index.size());

			//std::cout << "Updating total amount of edges" << std::endl;

			//Add text message for the total amount of edges
			std::stringstream title;
			title << "Total edges: " <<  this->getTotalNumOfEdges();
			//if (m_unique_edges) {
				//title << " |Unique: " << m_unique_edges << std::endl;
			//}
			if (m_display_total_edges) {
				m_win->addTextMessage(5,-m_offset_y_total_edges,
						title.str(),
						mrpt::utils::TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_total_edges);
			}

			// add a textMessage for every stored edge type
			for (std::map<std::string, double>::const_iterator it = m_name_to_offset_y.begin();
					it != m_name_to_offset_y.end(); ++it) {

				std::string name = it->first;
				double offset_y = it->second;
				int text_index = m_name_to_text_index.find(name)->second;
				int edges_num = m_name_to_edges_num.find(name)->second;

				//std::cout << "name: " << name << " | offset_y: " << offset_y << " | text_index: " << text_index << std::endl;

				std::stringstream title;
				title << "  " << name << ": " <<	edges_num << std::endl;
				m_win->addTextMessage(5,-offset_y,
						title.str(),
						mrpt::utils::TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ text_index);
			}


			// add text message for the loop closures
			if (m_display_loop_closures) {
				std::stringstream title;
				title << "  " << "Loop closures: " <<  m_num_loop_closures << std::endl;
				m_win->addTextMessage(5,-m_offset_y_loop_closures,
						title.str(),
						mrpt::utils::TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_loop_closures);
			}

			m_win->forceRepaint();
		}

	private:
		mrpt::gui::CDisplayWindow3D* m_win;

		// Tracking number of edges
		std::map<std::string, int> m_name_to_edges_num;;
		int m_num_loop_closures;
		int m_unique_edges;

		// visualization std::maps
		std::map<std::string, double> m_name_to_offset_y;
		std::map<std::string, int> m_name_to_text_index;

		std::string m_font_name;
		int m_font_size;
		bool m_has_read_textmessage_params;

		// specifics to loop closures, total edges
		bool m_display_total_edges, m_display_loop_closures; // whether to show them at all
		int m_offset_y_total_edges, m_offset_y_loop_closures;
		int m_text_index_total_edges, m_text_index_loop_closures;
};


#endif /* end of include guard: EDGECOUNTER_H */
