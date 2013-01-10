/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CSparseMatrix.h>
#include <memory>  // for auto_ptr<>
#include <queue>

namespace mrpt { namespace srba {


#define SYM_ST_SUPER_VERBOSE 0

using namespace std;

// Aux. function for going thru the entire path between two KFs in a spanning tree, saving the path step by step.
namespace internal
{
	template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
	void recursive_update_all_edges(
		typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::k2k_edge_vector_t & path,
		const TKeyFrameID dst_id,
		const TKeyFrameID cur_id,
		const TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE> * rba
		);
}

/** Incremental update of spanning trees after the insertion of ONE new node and ONE OR MORE edges */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
void TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree::update_symbolic_new_node(
	const TKeyFrameID                    new_node_id,
	const TPairKeyFrameID & new_edge,
	const topo_dist_t                    max_depth,
	const bool                           check_all_obs_are_connected,
	const new_kf_observations_t        * obs
	)
{
	ASSERT_(max_depth>=1)

	// Maintain a list of those nodes whose list of shortest spanning trees ("next_edge") has been modified, so we
	// can rebuild their "all_edges" lists.
	MRPT_TODO("Optimize and specify [i]->[j], not only [i]->all")
	std::set<TKeyFrameID>  kfs_with_modified_next_edge;

	// Generic algorithm for 1 or more new edges at once from the new_kf_id to the rest of the graph:
	// -----------------------------------------------------------------------------------------------
	// The first edge was already introduced in the STs above. Go on with the rest:
	// (Notation is according to the published paper, see explanatory graphs or understanding this is a hell!! ;-)
	{
		//const TKeyFrameID ik = getTheOtherFromPair(new_node_id, new_edges[nE] );
		const TKeyFrameID ik = getTheOtherFromPair(new_node_id, new_edge );

		// Build set tk = all nodes within distance <=(max_depth-1) from "ik"
		const map<TKeyFrameID,TSpanTreeEntry> & st_ik = sym.next_edge[ik];  // O(1)
		vector<pair<TKeyFrameID,topo_dist_t> > tk;
		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=st_ik.begin();it!=st_ik.end();++it)
			if (it->second.distance<max_depth)
				tk.push_back( make_pair(it->first,it->second.distance) );
		tk.push_back( make_pair(ik,0) ); // The set includes the root itself, which is not in the STs structures.

		// Build set STn = all nodes within distance <=max_depth from "new_node_id"
		// This will also CREATE the empty ST for "new_node_id" upon first call to [], in amortized O(1)
		const map<TKeyFrameID,TSpanTreeEntry> & st_n = sym.next_edge[new_node_id];  // access O(1)
		vector<pair<TKeyFrameID,const TSpanTreeEntry*> > STn;
		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=st_n.begin();it!=st_n.end();++it)
			STn.push_back( make_pair(it->first,&it->second) );
		STn.push_back( pair<TKeyFrameID,const TSpanTreeEntry*>(new_node_id,NULL) ); // The set includes the root itself, which is not in the STs structures.

		// for each r \in ST_W(n)
		for (size_t r_idx=0;r_idx<STn.size();r_idx++)
		{
			const TKeyFrameID      r        = STn[r_idx].first;
			const TSpanTreeEntry * ste_n2r  = STn[r_idx].second;
			const topo_dist_t      dist_r2n = ste_n2r ? ste_n2r->distance : 0;

			map<TKeyFrameID,TSpanTreeEntry> & st_r = sym.next_edge[r];  // O(1)

			TSpanTreeEntry * ste_r2n = NULL; // Will stay NULL for r=new_node_id
			if (r!=new_node_id)
			{
				map<TKeyFrameID,TSpanTreeEntry>::iterator it = st_r.find(new_node_id);
				ASSERT_(it!=st_r.end())
				ste_r2n = &it->second;
			}

			// for each s \in ST_{W-1}(i_k)
			for (size_t s_idx=0;s_idx<tk.size();s_idx++)
			{
				const TKeyFrameID s         = tk[s_idx].first;
				if (r==s) continue;
				const topo_dist_t dist_s2ik = tk[s_idx].second;

				map<TKeyFrameID,TSpanTreeEntry> & st_s = sym.next_edge[s];  // O(1)

				TSpanTreeEntry *ste_s2ik=NULL;  // =NULL if s==ik
				if (s!=ik)
				{
					map<TKeyFrameID,TSpanTreeEntry>::iterator it_ik_inSTs = st_s.find(ik);
					ASSERTDEB_(it_ik_inSTs != st_s.end())
					ste_s2ik = &it_ik_inSTs->second;
				}

				// new tentative distance s <--> ik
				const topo_dist_t new_dist = dist_r2n + dist_s2ik + 1;

				// Is s \in ST(r)?
				map<TKeyFrameID,TSpanTreeEntry>::iterator it_s_inSTr = st_r.find(s);    // O(log N)
				if (it_s_inSTr != st_r.end())
				{	// Found:
					// Is it shorter to go (r)->(n)--[dist=1]-->(ik)->(s) than (r)->(s) ? Then modify spanning tree
					if (new_dist < it_s_inSTr->second.distance)
					{
#if SYM_ST_SUPER_VERBOSE
cout << "ST: Shorter path ST["<<r<<"]["<<s<<"].N was "<<it_s_inSTr->second.next << " => "<<(ste_r2n ? ste_r2n->next : ik) << "[D:"<<it_s_inSTr->second.distance<<"=>"<<new_dist<<"]"<<endl;
cout << "ST: Shorter path ST["<<s<<"]["<<r<<"].N was "<<st_s[r].next << " => "<<(ste_s2ik ? ste_s2ik->next : new_node_id) << "[D:"<<st_s[r].distance<<"=>"<<new_dist<<"]"<<endl;
#endif
						// It's shorter: change spanning tree
						//  ST[r][s]
						it_s_inSTr->second.distance = new_dist;
						it_s_inSTr->second.next     = ste_r2n ? ste_r2n->next : ik;  // Next node in the direction towards "new_node_id"
						ASSERT_NOT_EQUAL_(r,it_s_inSTr->second.next) // no self-loops!

						//  ST[s][r]
						TSpanTreeEntry &ste_r_inSTs = st_s[r];
						ste_r_inSTs.distance = new_dist;
						ste_r_inSTs.next     = ste_s2ik ? ste_s2ik->next : new_node_id; // Next node in the direction towards "ik" or to "n" if this is "ik"
						ASSERT_NOT_EQUAL_(s,ste_r_inSTs.next) // no self-loops!

						// Mark nodes with their "next_node" modified:
						kfs_with_modified_next_edge.insert(s);
						kfs_with_modified_next_edge.insert(r);
					}
					// Otherwise, leave things stay.
				}
				else
				{	// Not found:
					if (new_dist<=max_depth)
					{
#if SYM_ST_SUPER_VERBOSE
cout << "ST: New path ST["<<r<<"]["<<s<<"].N ="<<(ste_r2n ? ste_r2n->next : ik) << "[D:"<<dist_r2n + dist_s2ik + 1<<"]"<<endl;
cout << "ST: New path ST["<<s<<"]["<<r<<"].N ="<<(ste_s2ik ? ste_s2ik->next : new_node_id) << "[D:"<<dist_r2n + dist_s2ik + 1<<"]"<<endl;
#endif

						// Then the node "s" wasn't reachable from "r" but now it is:
						//  ST[r][s]
						TSpanTreeEntry &ste_s_inSTr = st_r[s]; // O(log N)

						ste_s_inSTr.distance = new_dist;
						ste_s_inSTr.next     = ste_r2n ? ste_r2n->next : ik;  // Next node in the direction towards "new_node_id"
						ASSERT_NOT_EQUAL_(r,ste_s_inSTr.next) // no self-loops!

						//  ST[s][r]
						TSpanTreeEntry &ste_r_inSTs = st_s[r]; // O(log N)
						ste_r_inSTs.distance = new_dist;
						ste_r_inSTs.next     = ste_s2ik ? ste_s2ik->next : new_node_id; // Next node in the direction towards "ik"
						ASSERT_NOT_EQUAL_(s,ste_r_inSTs.next) // no self-loops!

						// Mark nodes with their "next_node" modified:
						kfs_with_modified_next_edge.insert(r);
						kfs_with_modified_next_edge.insert(s);
					}
				}

			} // end for each s
		} // end for each r

	} // end for each new edge


	// Optional check for correct connection of observed base KFs to current KF  -------------
	if (check_all_obs_are_connected)
	{
		ASSERT_(obs)

		// 1) Build a first list of pending spanning trees to build from observations:
		std::set<TPairKeyFrameID>  pending_checks;

		for (size_t i=0;i<obs->size();i++)
		{
			const new_kf_observation_t &o = (*obs)[i];

			// If it's a new Landmark (observed for the first time, already not added to the data structures), just skip
			// since we won't need spanning trees for it.
			if (o.obs.feat_id>=m_parent->all_lms.size() || m_parent->all_lms[ o.obs.feat_id ].rfp==NULL)
				continue;

			const TKeyFrameID base_id = m_parent->all_lms[ o.obs.feat_id ].rfp->id_frame_base;

			// make sure a spanning tree exists between "new_node_id" -> "base_id":
			pending_checks.insert( TPairKeyFrameID( new_node_id, base_id ) );
		}

		std::set<TPairKeyFrameID>  done_checks;
		while (!pending_checks.empty())
		{
			// Get first one:
			const TPairKeyFrameID ft = *pending_checks.begin();  // copy, don't make a ref since we're deleting the element in the container!
			pending_checks.erase(pending_checks.begin());

			if (done_checks.find(ft)!=done_checks.end())
				continue; // We already done this one.

			// Create path:
			std::map<TKeyFrameID,TSpanTreeEntry> & span_trees_from = sym.next_edge[ft.first];

			if ( span_trees_from.find(ft.second)==span_trees_from.end() )
			{	// It doesn't exist: create it.

				// We need the starting edge from "ft.first" in the direction towards "ft.second",
				//  and in the way mark as pending all the edges "intermediary node" -> "ft.second".
				vector<TKeyFrameID> found_path;

				bool found = m_parent->find_path_bfs(
					ft.first, // from
					ft.second, // target node
					found_path);

				ASSERT_(found && !found_path.empty())

				// save direction:
				TSpanTreeEntry & ste = span_trees_from[ft.second];
				ste.distance = found_path.size();
				ste.next = *found_path.rbegin();  // the last element

				// and append the rest of KFs as pending:
				for (size_t i=0;i<found_path.size();i++)
					if (found_path[i]!=ft.second)
						pending_checks.insert( TPairKeyFrameID( found_path[i] , ft.second ) );

				// Remember to update the node's "all_edges" field:
				kfs_with_modified_next_edge.insert( ft.first );
			}

			// Mark as done:
			done_checks.insert(ft);
		}

	} // end if "check_all_obs_are_connected"


#if 0
	{
		static int i=0;
		const std::string sFil    = mrpt::format("debug_spantree_%05i.txt",i);
		const std::string sFilDot = mrpt::format("debug_spantree_%05i.dot",i);
		const std::string sFilPng = mrpt::format("debug_spantree_%05i.png",i);
		this->dump_as_text_to_file(sFil);
		this->save_as_dot_file(sFilDot);
		::system(mrpt::format("dot %s -o %s -Tpng",sFilDot.c_str(), sFilPng.c_str() ).c_str());
		i++;
	}
#endif

	// Update "all_edges" --------------------------------------------
	// Only for those who were really modified.
	for (std::set<TKeyFrameID>::const_iterator it=kfs_with_modified_next_edge.begin();it!=kfs_with_modified_next_edge.end();++it)
	{
		const TKeyFrameID kf_id = *it;
		const std::map<TKeyFrameID,TSpanTreeEntry> & Ds = sym.next_edge[ kf_id ];  // O(1) in map_as_vector

		for (std::map<TKeyFrameID,TSpanTreeEntry>::const_iterator it2=Ds.begin();it2!=Ds.end();++it2)
		{
			const TKeyFrameID dst_kf_id = it2->first;

			const TKeyFrameID from = std::max(dst_kf_id, kf_id);
			const TKeyFrameID to   = std::min(dst_kf_id, kf_id);

			typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::k2k_edge_vector_t & path = sym.all_edges[from][to];  // O(1) in map_as_vector
			path.clear();
			internal::recursive_update_all_edges(path, to, from, this->m_parent );
		}
	} // end for each "kfs_with_modified_next_edge"


#if 0
	{
		// Security check: All spanning trees must have a max. depth of "max_depth"
		// 1st step: define nodes & their depths:
		for (next_edge_maps_t::const_iterator it1=sym.next_edge.begin();it1!=sym.next_edge.end();++it1)
			for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=it1->second.begin();it!=it1->second.end();++it)
				if (it->second.distance>max_depth)
				{
					std::stringstream s;
					s << "CONSISTENCY ERROR: Spanning tree of kf #" << it1->first << " has kf #" << it->first
					<< " at depth=" << it->second.distance << " > Max=" << max_depth << endl;
					s << "After updating symbolic spanning trees for new kf #" << new_node_id << " with new edges: ";
					for (size_t i=0;i<new_edges.size();i++) s << new_edges[i].first << "->" << new_edges[i].second << ", ";
					s << endl;
					THROW_EXCEPTION(s.str())
				}
	}
#endif
}


struct TBFSEntry
{
	TBFSEntry() : dist( std::numeric_limits<topo_dist_t>::max() )
	{}

	TKeyFrameID prev;
	topo_dist_t dist;
};

// Aux. function for "check_all_obs_are_connected":
//  Breadth-first search (BFS) for "trg_node"
//  Return: true: found
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
bool TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::find_path_bfs(
	const TKeyFrameID           cur_node,
	const TKeyFrameID           trg_node,
	std::vector<TKeyFrameID>  & found_path) const
{
	found_path.clear();
	if (cur_node==trg_node) return true; // No need to do any search...

	std::set<TKeyFrameID>   visited;
	std::queue<TKeyFrameID> pending;
	std::map<TKeyFrameID,TBFSEntry>    preceding;

	// Insert:
	pending.push(cur_node);
	visited.insert(cur_node);
	preceding[cur_node].dist = 0;

	while (!pending.empty())
	{
		const TKeyFrameID next_kf = pending.front();
		pending.pop();

		TBFSEntry & bfs_data_next = preceding[next_kf];
		const topo_dist_t cur_dist = bfs_data_next.dist;

		if (next_kf==trg_node)
		{
			// Path found: go thru the path in inverse order:
			topo_dist_t  dist = bfs_data_next.dist;
			found_path.resize(dist);
			TKeyFrameID path_node = trg_node;
			while (path_node != cur_node)
			{
				ASSERT_(dist>=0)
				found_path[--dist] = path_node;

				std::map<TKeyFrameID,TBFSEntry>::const_iterator it_prec = preceding.find(path_node);
				ASSERT_(it_prec != preceding.end())
				path_node = it_prec->second.prev;
			}
			return true; // End of search
		}

		// Get all connections of this node:
		ASSERTDEB_(next_kf < keyframes.size())
		const keyframe_info & kfi = keyframes[next_kf];

		for (size_t i=0;i<kfi.adjacent_k2k_edges.size();i++)
		{
			const k2k_edge_t* ed = kfi.adjacent_k2k_edges[i];
			const TKeyFrameID new_kf = getTheOtherFromPair2(next_kf, *ed);
			if (!visited.count(new_kf))
			{
				pending.push(new_kf);
				visited.insert(new_kf);
				
				TBFSEntry & p = preceding[new_kf];

				if (p.dist>cur_dist+1)
				{
					p.dist = cur_dist+1;
					p.prev = next_kf;
				}
			}
		}
	}
	return false; // No path found.
}


// Aux. function for going thru the entire path between two KFs in a spanning tree, saving the path step by step.
// Called from: TSpanningTree::update_symbolic()

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
void internal::recursive_update_all_edges(
	typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::k2k_edge_vector_t & path,
	const TKeyFrameID dst_id,
	const TKeyFrameID cur_id,
	const TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE> * rba
	)
{
	MRPT_TODO("Do non-recursive")

	// Move from "cur_id" in the direction of "dst_id":
	typename TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::TSpanningTree::next_edge_maps_t::const_iterator it_next_ed = rba->spanning_tree.sym.next_edge.find(cur_id);
	ASSERT_(it_next_ed != rba->spanning_tree.sym.next_edge.end())

	map<TKeyFrameID,TSpanTreeEntry>::const_iterator it_dst = it_next_ed->second.find(dst_id);
	ASSERT_(it_dst != it_next_ed->second.end())

	const TKeyFrameID next_id = it_dst->second.next;

	// Search for the edge: "cur_id" -> "next_id":
	ASSERT_(cur_id< rba->keyframes.size())

	const typename kf2kf_pose_traits<KF2KF_POSE_TYPE>::k2k_edge_vector_t &edges = rba->keyframes[cur_id].adjacent_k2k_edges;

	for (size_t i=0;i<edges.size();i++)
	{
		if ( (edges[i]->to==cur_id && edges[i]->from==next_id) ||
			 (edges[i]->from==cur_id && edges[i]->to==next_id) )
		{
			path.push_back(edges[i]);
			if (next_id==dst_id)
				 return; // Done! End of recursion
			else return internal::recursive_update_all_edges(path, dst_id,next_id, rba);
		}
	}
	// If we reach here it means we didn't find a path...
	THROW_EXCEPTION(mrpt::format("Couldn't find a topological path %u -> %u", static_cast<unsigned int>(cur_id),static_cast<unsigned int>(dst_id) ).c_str() )
}

} } // end NS
