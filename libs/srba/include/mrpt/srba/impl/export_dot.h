/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace mrpt { namespace srba {

/** Exports all the keyframes and landmarks as a directed graph in DOT (graphviz) format */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
bool RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::save_graph_as_dot(
	const std::string &targetFileName,
	const bool all_landmarks
	) const
{
	using namespace std;

	std::ofstream f(targetFileName.c_str());
	if (!f.is_open()) return false;

	f << "digraph G {\n";

	if (!rba_state.keyframes.empty())
	{
		// Keyframes:
		f << "/* KEYFRAMES */\n"
		     "node [shape=box,style=filled];\n";
		//for (typename rba_problem_state_t::keyframe_map_t::const_iterator it=rba_state.keyframes.begin();it!=rba_state.keyframes.end();++it)
		for (size_t id=0;id<rba_state.keyframes.size();++id)
			f << id << "; ";
		f << "\n";

		// k2k edges:
		f << "/* KEYFRAME->KEYFRAME edges */\n"
		     "edge [style=bold];\n";
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
			for (typename rba_problem_state_t::k2k_edges_deque_t::const_iterator itEdge2 = rba_state.k2k_edges.begin();itEdge2!=rba_state.k2k_edges.end();++itEdge2)
			{
				const k2k_edge_t * itEdge = itEdge2->pointer();
#else
			for (typename rba_problem_state_t::k2k_edges_deque_t::const_iterator itEdge = rba_state.k2k_edges.begin();itEdge!=rba_state.k2k_edges.end();++itEdge)
			{
#endif
				f << itEdge->from << "->" << itEdge->to << ";\n";
			}

		if (all_landmarks)
		{
			// Landmarks with fixed position:
			f << "/* LANDMARKS with known relative position, and its base keyframe */\n"
				 "node [shape=triangle,style=filled,fillcolor=gray80];\n"
				 "edge [style=bold,color=black];\n";
			for (typename TRelativeLandmarkPosMap::const_iterator itLM = rba_state.known_lms.begin();itLM != rba_state.known_lms.end();++itLM)
				f << itLM->second.id_frame_base << " -> " << "L"<<itLM->first << "; ";
			f << "\n";

			// Landmarks with fixed position:
			f << "/* LANDMARKS with unknown relative position */\n"
				 "node [shape=triangle,style=filled,fillcolor=white];\n"
				 "edge [style=solid,color=gray20];\n";
			for (typename TRelativeLandmarkPosMap::const_iterator itLM = rba_state.unknown_lms.begin();itLM != rba_state.unknown_lms.end();++itLM)
				f << itLM->second.id_frame_base << " -> " << "L"<<itLM->first << "; ";
			f << "\n";

			// Observations:
			f << "/* OBSERVATIONS */\n"
				 "edge [style=dotted,color=black];\n";


			for (typename rba_problem_state_t::all_observations_deque_t::const_iterator itO=rba_state.all_observations.begin();itO!=rba_state.all_observations.end();++itO)
			{
#ifdef SRBA_WORKAROUND_MSVC9_DEQUE_BUG
				f << (*itO)->obs.kf_id << " -> L" << (*itO)->obs.obs.feat_id << ";\n";
#else
				f << itO->obs.kf_id << " -> L" << itO->obs.obs.feat_id << ";\n";
#endif
			}
			f << "\n";
		}

	} // end if graph is not empty

	f << "\n}\n";

	return true;
}


} }  // end namespaces
