/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_ENGINE_IMPL_H
#define GRAPH_SLAM_ENGINE_IMPL_H
// Only to be included from within <mrpt/graphslam/GraphSlamEngine.h>

namespace mrpt { namespace graphslam {

	// Default ctor:
	template <class GRAPH_T,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	GraphSlamEngine<GRAPH_T,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::GraphSlamEngine()
	{
		m_solver.graph = &m_graph; // Attach the map to the solver.
	}



} } // End of namespaces

#endif
