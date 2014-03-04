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

	// Main entry point (alternative 1): Process a pair "action"+"sensory observations" to update the map.
	template <class GRAPH_T,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<GRAPH_T,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::processActionObservation (
		const mrpt::slam::CActionCollection &action, 
		const mrpt::slam::CSensoryFrame &in_SF)
	{
		using namespace mrpt::slam; 
		using namespace mrpt::poses; 

		// 1) process action:
		CActionRobotMovement2DPtr movEstimation = action.getBestMovementEstimation();
		if (movEstimation)
		{
			m_auxAccumOdometry.composeFrom( m_auxAccumOdometry, CPose3D(movEstimation->poseChange->getMeanVal()) );

			CObservationOdometryPtr obs = CObservationOdometry::Create();
			obs->timestamp = movEstimation->timestamp;
			obs->odometry = CPose2D(m_auxAccumOdometry);
			this->processObservation(obs);
		}
		// 2) Process observations one by one:
		for (CSensoryFrame::const_iterator i=in_SF.begin();i!=in_SF.end();++i)
			this->processObservation(*i);
	}

	/** Main entry point (alternative 2): Process one odometry OR sensor observation. */
	template <class GRAPH_T,class F2F_MATCH,class UPDATE_DECIDER,class MAPS_IMPLEMENTATION,class GRAPHSLAM_SOLVER>
	void GraphSlamEngine<GRAPH_T,F2F_MATCH,UPDATE_DECIDER,MAPS_IMPLEMENTATION,GRAPHSLAM_SOLVER>::processObservation (const mrpt::slam::CObservationPtr &obs)
	{

	}



} } // End of namespaces

#endif
