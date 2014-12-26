/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>  // stringstream

namespace mrpt { namespace srba {

using namespace std;

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::clear()
{
	num.clear();
	sym.next_edge.clear();
	sym.all_edges.clear();
}


template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::dump_as_text(string &s)  const
{
	using mrpt::format;

	s.clear();

	s +=
	"  From  | Shortest path to:=>Next node to move to [Distance]              \n"
	"--------+-----------------------------------------------------------------\n";
	for (typename next_edge_maps_t::const_iterator it1=sym.next_edge.begin();it1!=sym.next_edge.end();++it1)
	{
		s += format(" %6u |",static_cast<unsigned int>(it1->first) );

		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it2=it1->second.begin();it2!=it1->second.end();++it2)
			s += format(" %5u:=>%5u [%u] |",static_cast<unsigned int>(it2->first), static_cast<unsigned int>(it2->second.next), static_cast<unsigned int>(it2->second.distance));

		s +=
		"\n"
		"--------+-----------------------------------------------------------------\n";
	}

	s +=
	"\n\n"
	"  From  |   To   | Shortest path sequence                                 \n"
	"--------+--------+--------------------------------------------------------\n";
	for (typename all_edges_maps_t::const_iterator it1=sym.all_edges.begin();it1!=sym.all_edges.end();++it1)
	{
		for (typename map<TKeyFrameID, k2k_edge_vector_t >::const_iterator it2=it1->second.begin();it2!=it1->second.end();++it2)
		{
			s += format(" %6u | %6u |",static_cast<unsigned int>(it1->first),static_cast<unsigned int>(it2->first) );

			const k2k_edge_vector_t & edges = it2->second;
			for (typename k2k_edge_vector_t::const_iterator it3=edges.begin();it3!=edges.end();++it3)
				s += format(" [%4u => %4u] ",static_cast<unsigned int>((*it3)->from),static_cast<unsigned int>((*it3)->to) );

			s+=
			"\n"
			"--------+--------+--------------------------------------------------------\n";
		}
	}
}

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
bool TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::dump_as_text_to_file(const string &sFileName) const
{
	ofstream f;
	f.open(sFileName.c_str());
	if (!f.is_open()) return false;

	string s;
	this->dump_as_text(s);

	return !(f << s).fail();
}

namespace internal
{
	template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
	void recursive_print_st_dot(
		set< pair<string,string> > & all_edges,
		const string &prefix,
		const TKeyFrameID came_from,
		const TKeyFrameID root,
		const map<TKeyFrameID,TSpanTreeEntry> &root_entries,
		const typename TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::next_edge_maps_t &all,
		set<TKeyFrameID> &visited,
		const map<TKeyFrameID,TSpanTreeEntry> &top_root_entries)
	{
		visited.insert(root);

		// All nodes at depth=1
		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=root_entries.begin();it!=root_entries.end();++it)
		{
			if (it->second.distance==1 && top_root_entries.find(it->first)!=top_root_entries.end())
			{
				const TKeyFrameID child = it->first;
				//s << prefix << root << " -> " << prefix << child << ";\n";
				const string s1 = prefix + mrpt::format("%06u",static_cast<unsigned int>( std::max(root,child) ));
				const string s2 = prefix + mrpt::format("%06u",static_cast<unsigned int>( std::min(root,child) ));
				all_edges.insert( make_pair(s1,s2) );

				if (!visited.count(it->first))
				{
					typename TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::next_edge_maps_t::const_iterator it_ce = all.find(child);
					ASSERT_(it_ce != all.end())
					internal::recursive_print_st_dot(all_edges,prefix,root,child,it_ce->second,all,visited,top_root_entries);
				}
			}
		}
	}

} // end NS "internal"

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
bool TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::save_as_dot_file(const string &sFileName, const std::vector<TKeyFrameID> &kf_roots_to_save )  const
{
	using mrpt::format;

	ofstream f;
	f.open(sFileName.c_str());
	if (!f.is_open()) return false;

	vector<next_edge_maps_t::const_iterator> its_to_process;

	if (kf_roots_to_save.empty())
	{
		// All:
		its_to_process.reserve(sym.next_edge.size());
		for (next_edge_maps_t::const_iterator it1=sym.next_edge.begin();it1!=sym.next_edge.end();++it1)
			its_to_process.push_back(it1);
	}
	else
	{
		for (size_t i=0;i<kf_roots_to_save.size();i++)
		{
			next_edge_maps_t::const_iterator it=sym.next_edge.find(kf_roots_to_save[i]);
			if (it!=sym.next_edge.end())  // silently ignore queries for KFs without a tree
				its_to_process.push_back(it);
		}
	}

	stringstream s;

	s << "graph G {\n";

	map<size_t, set<string> >  kfs_by_depth;
	map<string, size_t>   depth_kf;

	// 1st step: define nodes & their depths:
	for (size_t k=0;k<its_to_process.size();k++)
	{
		const next_edge_maps_t::const_iterator it1=its_to_process[k];

		const TKeyFrameID root = it1->first;
		const string sR = format("%06u",static_cast<unsigned int>(root) );

		const string sNodeDefR = sR + mrpt::format("%06u [label=%u]",static_cast<unsigned int>(root),static_cast<unsigned int>(root));
		kfs_by_depth[0].insert( sNodeDefR );
		depth_kf[mrpt::format("%06u%06u",static_cast<unsigned int>(root),static_cast<unsigned int>(root))] = 0;

		// All nodes at depth=1
		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=it1->second.begin();it!=it1->second.end();++it)
		{
			const string sNodeDef = sR + mrpt::format("%06u [label=%u]",static_cast<unsigned int>(it->first),static_cast<unsigned int>(it->first));
			kfs_by_depth[it->second.distance].insert( sNodeDef );
			depth_kf[mrpt::format("%06u%06u",static_cast<unsigned int>(root),static_cast<unsigned int>(it->first))] = it->second.distance;
		}
	}

	for (map<size_t, set<string> >::const_iterator it=kfs_by_depth.begin();it!=kfs_by_depth.end();++it)
	{
		//const size_t depth = it->first;
		const set<string> & sNodes = it->second;

		s <<
		"subgraph {\n"
		"  rank = same;\n";
		for (set<string>::const_iterator itS=sNodes.begin();itS!=sNodes.end();++itS)
			s << *itS << "\n";
		s <<"};\n";
	}

	// 2nd step: generate list of all edges in all trees:
	set< pair<string,string> > all_edges;

	for (size_t k=0;k<its_to_process.size();k++)
	{
		const next_edge_maps_t::const_iterator it1=its_to_process[k];

		const TKeyFrameID root = it1->first;

		// All nodes at all depths:
		for (map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=it1->second.begin();it!=it1->second.end();++it)
		{
			const TKeyFrameID other = it->first;

			const TKeyFrameID id1 = std::max(root,other);
			const TKeyFrameID id2 = std::min(root,other);

			// Get all edges in the shortest path between them:
			// (we only store <max,min> since this table is symmetric)
			typename all_edges_maps_t::const_iterator it_eds_id1 = sym.all_edges.find(id1);
			ASSERT_(it_eds_id1 != sym.all_edges.end())

			const std::map<TKeyFrameID, k2k_edge_vector_t> &eds_id1 = it_eds_id1->second;
			typename std::map<TKeyFrameID, k2k_edge_vector_t>::const_iterator eds_it = eds_id1.find(id2);
			ASSERT_(eds_it!=eds_id1.end())

			const k2k_edge_vector_t &eds = eds_it->second;

			for (size_t i=0;i<eds.size();i++)
			{
				const TKeyFrameID to   = eds[i]->to;
				const TKeyFrameID from = eds[i]->from;

				const string s1 = mrpt::format("%06u%06u",static_cast<unsigned int>(root),static_cast<unsigned int>( std::max(to,from) ));
				const string s2 = mrpt::format("%06u%06u",static_cast<unsigned int>(root),static_cast<unsigned int>( std::min(to,from) ));

				all_edges.insert( make_pair(s1,s2) );
			}
		}
	}


	// And now only draw those at a different depth level:
	for (set< pair<string,string> >::const_iterator it=all_edges.begin();it!=all_edges.end();++it)
			s << it->first << " -- " << it->second << ";\n";

	// And now generate invisible edges between nodes across all depths ("ranks") so graphviz really put them in different physical heights:
	for (map<size_t, set<string> >::const_iterator it=kfs_by_depth.begin();it!=kfs_by_depth.end();++it)
	{
		const set<string> & sNodes = it->second;
		if (sNodes.empty()) continue; // But shouldn't occur!

		if (it!=kfs_by_depth.begin())
		  s << " -- ";

		s << it->second.begin()->substr(0,12) << " ";
	}
	if (!kfs_by_depth.empty()) s << " [style=invis];\n";

	s << "}\n";

	return !(f << s.str()).fail();
}


/** Returns min/max and mean/std stats on the number of nodes found on all the spanning trees. Runs in O(N), N=number of keyframes. */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void TRBA_Problem_state<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::TSpanningTree::get_stats(
	size_t &num_nodes_min,
	size_t &num_nodes_max,
	double &num_nodes_mean,
	double &num_nodes_std) const
{
	num_nodes_min = 0;
	num_nodes_max  = 0;
	num_nodes_mean  = 0;
	num_nodes_std  = 0;

	std::vector<size_t> num_nodes;
	num_nodes.reserve(sym.next_edge.size());

	for (next_edge_maps_t::const_iterator it1=sym.next_edge.begin();it1!=sym.next_edge.end();++it1)
		num_nodes.push_back( it1->second.size() );

	mrpt::math::meanAndStd(num_nodes,num_nodes_mean,num_nodes_std);
	mrpt::math::minimum_maximum(num_nodes, num_nodes_min, num_nodes_max);
}


} } // end NS
