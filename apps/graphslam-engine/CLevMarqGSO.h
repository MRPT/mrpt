/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_H
#define CLEVMARQGSO_H

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/system/threads.h>

#include "CGraphslamOptimizer.h"
#include "CWindowManager.h"

#include <iostream>

namespace mrpt { namespace graphslam { namespace optimizers {

/**
 * Levenberg-Marquardt non-linear graph slam optimization scheme
 */

template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CLevMarqGSO_t:
	public mrpt::graphslam::optimizers::CGraphSlamOptimizer_t<GRAPH_t>
{
  public:
		// Public methods
		//////////////////////////////////////////////////////////////

		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;

    CLevMarqGSO_t();
    ~CLevMarqGSO_t();
    void initCLevMarqGSO_t();

		bool updateOptimizerState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

		void setGraphPtr(GRAPH_t* graph);
		void setRawlogFname(const std::string& rawlog_fname);
		void setCDisplayWindowPtr(mrpt::gui::CDisplayWindow3D* win); 
    void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager);

    void initializeVisuals();
    void updateVisuals();

		// struct for holding the optimizer-related variables in a compact form
    struct TParams: public mrpt::utils::CLoadableOptions {
    	public:
    		TParams();
    		~TParams();

    		void loadFromConfigFile(
    				const mrpt::utils::CConfigFileBase &source,
    				const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;
    };


		// Public members
		// ////////////////////////////
    TParams params;

  private:
  	// Private methods
		void optimizeGraph();

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph;
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::gui::CWindowManager_t* m_win_manager;

		std::string m_rawlog_fname;

		bool m_initialized_visuals;



};

} } } // end of namespaces

#include "CLevMarqGSO_impl.h"

#endif /* end of include guard: CLEVMARQGSO_H */
