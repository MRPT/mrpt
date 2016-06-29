/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_IMPL_H
#define CLEVMARQGSO_IMPL_H

using namespace mrpt::graphslam::optimizers;

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::CLevMarqGSO_t() {
	MRPT_START;

	MRPT_END;
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::~CLevMarqGSO_t() {
	MRPT_START;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::initCLevMarqGSO_t() {
	MRPT_START;

	MRPT_END;
}

// Member function implementations
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
bool CLevMarqGSO_t<GRAPH_t>::updateOptimizerState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;


	return false;
	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setRawlogFname(const std::string& rawlog_fname) {
	MRPT_START;

	MRPT_END;
}


template<class GRAPH_t> void
CLevMarqGSO_t<GRAPH_t>::setCDisplayWindowPtr(
		mrpt::gui::CDisplayWindow3D* win) {
	MRPT_START;

	m_win = win;

	std::cout << "[CLevMarqGSO:] Fetched the CDisplayWindow successfully"
		<< std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setWindowManagerPtr(
		mrpt::gui::CWindowManager_t* win_manager) {
	m_win_manager = win_manager;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::initializeVisuals() {
	MRPT_START;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::updateVisuals() {
	MRPT_START;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::optimizeGraph() {
	MRPT_START;

	MRPT_END;
}

// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::TParams::TParams() {
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::TParams::~TParams() {
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Levenberg-Marquardt Optimization ]------------------\n");

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;

	std::cout << "Successfully loaded CLevMarqGSO parameters. " << std::endl;

	MRPT_END;
}



#endif /* end of include guard: CLEVMARQGSO_IMPL_H */
