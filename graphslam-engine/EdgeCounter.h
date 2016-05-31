#ifndef EDGECOUNTER_H
#define EDGECOUNTER_H



#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;

using namespace std;

// TODO - make it case-insensitive

/**
 * Generic class for tracking the total number of edges for different tpes of
 * edges and for storing visualization-related information for each type
 *
 */
class EdgeCounter_t {
	public:

		EdgeCounter_t(CDisplayWindow3D* win = NULL) {
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

			for (map<string, int>::const_iterator it = m_name_to_edges_num.begin();
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

			for (map<string, int>::const_iterator it = m_name_to_edges_num.begin();
					it != m_name_to_edges_num.end(); ++it) {
				sum += it->second;
			}
			*total_num_edges = sum;
		}
		/**
		 * getNumForEdgeType
		 *
		 * Return the number of edges for the specified type
		 * \sa getNumForEdgeType(const string& name, int* total_num)
		 */
		int getNumForEdgeType(const string& name) const {
			map<string, int>::const_iterator search = m_name_to_edges_num.find(name);
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
		 * \sa getNumForEdgeType(const string& name)
		 */
		void getNumForEdgeType(const string& name, int* total_num) {
			map<string, int>::const_iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				*total_num = search->second;
			}
			else {
				THROW_EXCEPTION("No edge with such name exists")
			}
		}

		/**
		 * addEdge
		 *
		 * Increment the number of edges for the specified type.
		 * If edge exists and is_new is_also true, throw exception
		 */
		void addEdge(const string& name, bool is_loop_closure=false, bool is_new=false) {
			map<string, int>::iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				(search->second)++; // increment to the found element

				// specify warning if is_new = true
				if (is_new) {
					string str_err = "Specified edge type already exists but is_new is also specified!";
					THROW_EXCEPTION(str_err)
						//stringstream ss_warn;
						//ss_warn << "Commencing with the increment normally" << endl;
						//MRPT_WARNING(ss_warn.str())
				}
				if (is_loop_closure) {
					// throw if user has also specified the is new boolean flag
					if (is_new) {
						string str_err = "Both is_new and is_loop_closure flags are true. Exiting...";
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
					string str_err = "No edge with such name exists. Specify is_new parameter if you want to add it";
					THROW_EXCEPTION(str_err)
				}
			}

			// Update the visualization if the user has already set the vizualization
			// parameters
			if (m_has_read_textmessage_params) {
				updateTextMessages();
			}
		}
		/**
		 * addEdgeType
		 *
		 * Explicitly register a new edge type.
		 */
		void addEdgeType(const string& name) {
			map<string, int>::const_iterator search = m_name_to_edges_num.find(name);
			if ( search != m_name_to_edges_num.end() ) {
				THROW_EXCEPTION(format("Specified edge type %s already exists", name.c_str()))
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
			stringstream ss_out;
			ss_out << "Summary of Edges: " << endl;
			ss_out << "---------------------------" << endl;

			ss_out << "\t Total edges: " << this->getTotalNumOfEdges() << endl;

			for (map<string, int>::const_iterator it = m_name_to_edges_num.begin();
					it != m_name_to_edges_num.end(); ++it) {
				ss_out << "\t " << it->first << " edges: " << it->second << endl;
			}
			ss_out << "\t Loop closure edges: " << this->getLoopClosureEdges() << endl;

			cout << ss_out.str() << endl;
		}

		// VISUALIZATION RELATED METHODS
		// ////////////////////////////

		/**
		 * setVisualizationWindow
		 *
		 * Add the visualization window. Handy function for not having to
		 * specify it in the class constructor
		 */
		void setVisualizationWindow(CDisplayWindow3D* win) { m_win = win; }

		/**
		 * setTextMessageParams
		 *
		 * Add the textMessage parameters to the object - used during visualization
		 * All the names in the given maps have to be already specified and added in
		 * the object via addEdge with is_new=true or addEdgeType
		 */
		void setTextMessageParams(const map<string, double>& name_to_offset_y,
				const map<string, int>& name_to_text_index,
				const string& font_name, const int& font_size) {

			//cout << "in setTextMessageParams " << endl
			//<< "m_offset_y_total_edges: " << m_offset_y_total_edges << endl
			//<< "m_text_index_total_edges: " << m_text_index_total_edges << endl;
			//cout << "in setTextMessageParams:  " << endl
			//<< "m_offset_y_loop_closures: " << m_offset_y_loop_closures << endl
			//<< "m_text_index_loop_closures" << m_text_index_loop_closures << endl;


			assert(m_win);
			assert(name_to_offset_y.size() == name_to_text_index.size());

			for (map<string, double>::const_iterator it = name_to_offset_y.begin();
					it != name_to_offset_y.end(); ++it) {
				string name = it->first;

				// check if name already exist, otherwise throw exception
				map<string, int>::const_iterator search = m_name_to_edges_num.find(name);
				if ( search == m_name_to_edges_num.end() ) {
					stringstream ss_err;
					ss_err << "Name " << name << " is not recognized as an Edge type." << endl;
					THROW_EXCEPTION(ss_err.str())
				}
				// name exists ...

				double offset_y = it->second;
				int text_index = name_to_text_index.find(name)->second;

				//cout << "in setTextMessageParams: " << endl;
				//cout << "name: " << name << " | offset_y: " << offset_y << " | text_index: " << text_index << endl;

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
		void setTextMessageParams(const map<string, double>& name_to_offset_y,
				const map<string, int>& name_to_text_index,
				const double& offset_y_total_edges, const int& text_index_total_edges,
				const double& offset_y_loop_closures, const int& text_index_loop_closures,
				const string& font_name, const int& font_size) {

			//cout << "in setTextMessageParams (extended) " << endl
			//<< "offset_y_total_edges: " << offset_y_total_edges << endl
			//<< "text_index_total_edges" << text_index_total_edges <<endl;
			//cout << "in setTextMessageParams (extended):	" << endl
			//<< "offset_y_loop_closures: " << offset_y_loop_closures << endl
			//<< "text_index_loop_closures" << text_index_loop_closures << endl;

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
			//cout << "In updateTextMessages fun" << endl;
			//cout << "m_display_total_edges: " << m_display_total_edges << endl;
			//cout << "m_offset_y_total_edges: " << m_offset_y_total_edges << endl;
			//cout << "m_text_index_total_edges: " << m_text_index_total_edges << endl;
			//cout << "m_display_loop_closures: " << m_display_loop_closures << endl;
			//cout << "m_offset_y_loop_closures: " << m_offset_y_loop_closures << endl;
			//cout << "m_text_index_loop_closures: " << m_text_index_loop_closures << endl;
			//cout << endl;

			assert(m_win);
			assert(m_has_read_textmessage_params);
			assert(m_name_to_offset_y.size() == m_name_to_text_index.size());

			//Add text message for the total amount of edges
			if (m_display_total_edges) {
				//cout << "Updating total amount of edges" << endl;
				stringstream title;
				title << "Total edges: " <<  this->getTotalNumOfEdges() << endl;
				m_win->addTextMessage(5,-m_offset_y_total_edges,
						title.str(),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_total_edges);
			}

			// add a textMessage for every stored edge type
			for (map<string, double>::const_iterator it = m_name_to_offset_y.begin();
					it != m_name_to_offset_y.end(); ++it) {

				string name = it->first;
				double offset_y = it->second;
				int text_index = m_name_to_text_index.find(name)->second;
				int edges_num = m_name_to_edges_num.find(name)->second;

				//cout << "name: " << name << " | offset_y: " << offset_y << " | text_index: " << text_index << endl;

				stringstream title;
				title << "  " << name << ": " <<	edges_num << endl;
				m_win->addTextMessage(5,-offset_y,
						title.str(),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ text_index);
			}


			// add text message for the loop closures
			if (m_display_loop_closures) {
				stringstream title;
				title << "  " << "Loop closures: " <<  m_num_loop_closures << endl;
				m_win->addTextMessage(5,-m_offset_y_loop_closures,
						title.str(),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_loop_closures);
			}

			m_win->forceRepaint();
		}

	private:
		CDisplayWindow3D* m_win;

		// Tracking number of edges
		map<string, int> m_name_to_edges_num;;
		int m_num_loop_closures;

		// visualization parameters
		map<string, double> m_name_to_offset_y;
		map<string, int> m_name_to_text_index;

		string m_font_name;
		int m_font_size;
		bool m_has_read_textmessage_params;

		// specifics to loop closures, total edges
		bool m_display_total_edges, m_display_loop_closures; // whether to show them at all
		int m_offset_y_total_edges, m_offset_y_loop_closures;
		int m_text_index_total_edges, m_text_index_loop_closures;

};


#endif /* end of include guard: EDGECOUNTER_H */
