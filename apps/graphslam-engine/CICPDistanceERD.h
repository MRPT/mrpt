	/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPDistanceERD_H
#define CICPDistanceERD_H

#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/slam/CICP.h>

#include <map>
#include <string>
#include <stdlib.h> // abs

#include "CEdgeRegistrationDecider.h"

#include <iostream>

// TODO - remove these
using namespace mrpt;
using namespace mrpt::synch;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::maps;



namespace mrpt { namespace graphslam { namespace deciders {

	/**
	 * Register new edges in the graph with the last added node. Criterion for
	 * adding new nodes should  be the goodness of the potential ICP edge. The
	 * nodes for ICP should be picked based on the distance from the last
	 * inserted node.
	 */
	template<
		  class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
		class CICPDistanceERD_t :
			public mrpt::graphslam::deciders::CEdgeRegistrationDecider_t<GRAPH_t> {
	  		public:
					typedef typename GRAPH_t::constraint_t constraint_t;
					// type of underlying poses (2D/3D)
					typedef typename GRAPH_t::constraint_t::type_value pose_t; 

					// Public methods
					//////////////////////////////////////////////////////////////
	    		CICPDistanceERD_t();
	    		~CICPDistanceERD_t();

					void updateDeciderState(
							mrpt::obs::CActionCollectionPtr action,
							mrpt::obs::CSensoryFramePtr observations,
							mrpt::obs::CObservationPtr observation );


					void setGraphPtr(GRAPH_t* graph);
					void setCDisplayWindowPtr(mrpt::gui::CDisplayWindow3D* win);
    			void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager);
    			void getEdgesStats(
    					std::map<const std::string, int>* edge_types_to_nums);

    			void initializeVisuals();
    			void updateVisuals();

    			struct TParams: public mrpt::utils::CLoadableOptions {
    				public:
    					TParams();
    					~TParams();

    					void loadFromConfigFile(
    							const mrpt::utils::CConfigFileBase &source,
    							const std::string &section);
							void 	dumpToTextStream(mrpt::utils::CStream &out) const;

							mrpt::slam::CICP icp;
 							// maximum distance for checking other nodes for ICP constraints
							double ICP_max_distance;
							// threshold for accepting an ICP constraint in the graph
							double ICP_goodness_thresh;
							int LC_min_nodeid_diff;

    			};


					// Public variables
					// ////////////////////////////
    			TParams params;

	  		private:
					// Private functions
					//////////////////////////////////////////////////////////////
					/**
		 	 	 	 * Initialization function to be called from the various constructors
		 	 	 	 */
					void initCICPDistanceERD_t();
					void checkRegistrationCondition(
							const std::set<mrpt::utils::TNodeID>& nodes_set);
    			void registerNewEdge(
    					const mrpt::utils::TNodeID& from, 
    					const mrpt::utils::TNodeID& to,
    					const constraint_t& rel_edge );
    			/**
    			 * Run ICP for the laser scans of the given nodeIDs and fill the
    			 * potential constraint between them. Returns the goodness of the
    			 * potential constraint so that the caller can decide if they want to
    			 * keep the constraint or not. Returns 0 if laser scans have not been
    			 * registered at one of the give node positions
    			 */
					double getICPEdge(
							const mrpt::utils::TNodeID& from,
							const mrpt::utils::TNodeID& to,
							constraint_t* rel_edge );
					/**
					 * fill list of nodes that are within certain distance from a given
					 * node
					 */
		 			void getNearbyNodesOf(
		 					std::set<mrpt::utils::TNodeID> *nodes_set, 
							const mrpt::utils::TNodeID& cur_nodeID,
							double distance );

					// Private variables
					//////////////////////////////////////////////////////////////

					GRAPH_t* m_graph;
					mrpt::gui::CDisplayWindow3D* m_win;
					mrpt::gui::CWindowManager_t* m_win_manager;

					bool m_initialized_visuals;

					mrpt::utils::TColorf m_search_disk_color; // see Ctor for initialization
					double m_offset_y_search_disk;
					int m_text_index_search_disk;
					

					std::map<const mrpt::utils::TNodeID, 
						mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans;
					std::map<const std::string, int> m_edge_types_to_nums;

    			int m_last_total_num_of_nodes;
    			CObservation2DRangeScanPtr m_last_laser_scan;

			};

} } } // end of namespaces

#include "CICPDistanceERD_impl.h"
#endif /* end of include guard: CICPDistanceERD_H */

