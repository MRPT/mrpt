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

#include <mrpt/srba.h>
#include <mrpt/base.h>
#include <mrpt/random.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::srba;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

typedef  RBA_Problem_P6D_L3D_ObsMono my_rba_t;

/*
 topo=0 -> linear graph
 topo=1 -> linear graph w/ loops

 topo=10 -> like 0, inverted edges
 topo=11 -> like 1, inverted edges

 topo=2 -> "roundabout"
*/
void test_spantree_topology(
	const int topo,
	const size_t nKFs,
	const size_t max_depth,
	const uint32_t rnd_seed)
{
	randomGenerator.randomize(rnd_seed);
	my_rba_t::traits_t::new_kf_observations_t  dummy_obs; // Not used

	// The test object:
	my_rba_t rba;
	rba.enable_time_profiler(false);
	rba.parameters.srba.max_tree_depth = max_depth;

	const double PROB_LOOP_CLOSURE = 0.05;

	const double SPACE_MAX_XYZ = 10;
	mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t  GT_KF_poses(nKFs); // Ground truth poses

	// Create random global coordinates GT pose:
	for (size_t kf=0;kf<nKFs;kf++)
	{
		GT_KF_poses[kf] = mrpt::poses::CPose3D(
			randomGenerator.drawUniform(-SPACE_MAX_XYZ,SPACE_MAX_XYZ),
			randomGenerator.drawUniform(-SPACE_MAX_XYZ,SPACE_MAX_XYZ),
			randomGenerator.drawUniform(-SPACE_MAX_XYZ,SPACE_MAX_XYZ),
			randomGenerator.drawUniform(-M_PI,M_PI),
			randomGenerator.drawUniform(-0.5*M_PI,0.5*M_PI),
			randomGenerator.drawUniform(-M_PI,M_PI) );
	}

	// Create the graph and the incremental STs:
	for (size_t kf=0;kf<nKFs;kf++)
	{
		// Create KF:
		const TKeyFrameID new_kf = rba.alloc_keyframe();
		if (!new_kf) continue; // First KF has no edge!

		// Decide edges for the desired topology:
		std::vector<TPairKeyFrameID> new_edges;

		switch(topo)
		{
			// Linear graph --------------
			case 0:
				new_edges.push_back( TPairKeyFrameID(new_kf-1, new_kf) );
				break;
			case 100:
				new_edges.push_back( TPairKeyFrameID(new_kf, new_kf-1) );
				break;

			// Linear graph w/ loop closures --------------
			case 1:
			case 101:
			{
				const bool is_inv = topo>100;
				new_edges.push_back( TPairKeyFrameID(new_kf-1, new_kf) );
				if (new_kf>2)
				{
					while ( randomGenerator.drawUniform(0,1)<PROB_LOOP_CLOSURE )
					{
						TKeyFrameID id;
						randomGenerator.drawUniformUnsignedIntRange(id,0,new_kf-2);

						if ( !rba.get_rba_state().are_keyframes_connected(id,new_kf) )
							new_edges.push_back( is_inv ? TPairKeyFrameID(new_kf,id) : TPairKeyFrameID(id,new_kf)  );
					}
				}
			}
			break;

			default: ASSERT_(false)
		};

		// Create:
		for (size_t i=0;i<new_edges.size();i++)
		{
			const mrpt::poses::CPose3D GT_rel_inv_pose = GT_KF_poses[new_edges[i].first] - GT_KF_poses[new_edges[i].second];
			rba.create_kf2kf_edge(new_kf, new_edges[i], dummy_obs , GT_rel_inv_pose );
		}
	}

	// Update ALL numeric relative poses.
	// --------------------------------------------
	rba.get_rba_state().spanning_tree.update_numeric(false /*skip those marked as up-to-date => So: false=just update them all*/);

	// Compare incremental STs with BFS trees:
	// --------------------------------------------
	const my_rba_t::rba_problem_state_t::TSpanningTree::next_edge_maps_t & kf_nexts = rba.get_rba_state().spanning_tree.sym.next_edge;
	std::vector<bool> aux_visited;
	for (size_t kf=0;kf<nKFs;kf++)
	{
		// Run BFS to find reachable KFs:
		my_rba_t::frameid2pose_map_t st;
		rba.create_complete_spanning_tree(kf,st,max_depth);

		// Check if number of reachable KFs matches:
		my_rba_t::rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_st_it = kf_nexts.find(kf);
		ASSERT_(it_st_it != kf_nexts.end())

		const std::map<TKeyFrameID,TSpanTreeEntry> & st_i = it_st_it->second;

		EXPECT_GE(st.size(),1); // "create_complete_spanning_tree()" returns the root node, in the STs we don't, so that's the why of the "-1" next:
		EXPECT_EQ(st_i.size(), st.size()-1 )
			<< "Expected ST of KF " << kf << " of depth "<<max_depth<<" to be of size " << st.size()-1 << " but it's " << st_i.size() << endl;

		if (st_i.size() != (st.size()-1) )
		{
#ifdef _DEBUG
			rba.save_graph_as_dot("dbg.dot");
			::system("dot dbg.dot -o dbg.png -Tpng");
			mrpt::system::pause();
#endif
			return;
		}

		// Get the numeric ST of this KF:
		kf2kf_pose_traits<my_rba_t::kf2kf_pose_type>::TRelativePosesForEachTarget::const_iterator it_num_st_i = rba.get_rba_state().spanning_tree.num.find(kf);
		ASSERT_(it_num_st_i != rba.get_rba_state().spanning_tree.num.end())

		const my_rba_t::frameid2pose_map_t & num_st_i = it_num_st_i->second;

		// For each reachable KF:
		for (my_rba_t::frameid2pose_map_t::const_iterator it=st.begin();it!=st.end();++it)
		{
			const TKeyFrameID dst_kf = it->first;
			if (dst_kf==kf) continue;

			// Check that they are the same are at the same depth:
			std::vector<TKeyFrameID>  found_path;
			bool found = rba.find_path_bfs(kf, dst_kf, found_path);

			EXPECT_TRUE(found && !found_path.empty())
				<< "Expected to find path between " << kf << " <-> " << dst_kf << " path.size()=" << found_path.size() << endl;
			if (!found || found_path.empty())
			{
#ifdef _DEBUG
				rba.save_graph_as_dot("dbg.dot");
				::system("dot dbg.dot -o dbg.png -Tpng");
				mrpt::system::pause();
#endif
				return;
			}

			// Check that they are the same KFs, by the way...
			std::map<TKeyFrameID,TSpanTreeEntry>::const_iterator it_st_i = st_i.find(dst_kf);
			EXPECT_TRUE(it_st_i != st_i.end())
				<< "Expected to find KF " << dst_kf << " in the ST of kf " << kf <<", but it wasn't." << endl;
			if (it_st_i == st_i.end())
			{
#ifdef _DEBUG
				rba.save_graph_as_dot("dbg.dot");
				::system("dot dbg.dot -o dbg.png -Tpng");
				mrpt::system::pause();
#endif
				return;
			}

			const TSpanTreeEntry &st_ij = it_st_i->second;

			EXPECT_EQ(found_path.size(), st_ij.distance);

			// and the final check: both reconstructions must lead to the same relative poses!
			// -------------------------------------------------------------------------------
			const mrpt::poses::CPose3D &rel_pose_complete_st = it->second.pose;

			my_rba_t::frameid2pose_map_t::const_iterator it_num_st_i_j = num_st_i.find(dst_kf);
			ASSERT_(it_num_st_i_j != num_st_i.end())

			const mrpt::poses::CPose3D &rel_pose_incr_st = it_num_st_i_j->second.pose;

			EXPECT_NEAR(0, (rel_pose_complete_st.getAsVectorVal() - rel_pose_incr_st.getAsVectorVal()).array().abs().sum(), 1e-6)
				<< "Error in numeric relative pose of KF " << dst_kf << " from KF " << kf <<":" << endl
				<< "rel_pose_complete_st: "<< rel_pose_complete_st << endl
				<< "rel_pose_incr_st: "<< rel_pose_incr_st << endl;

			// Compare to GT:
			const mrpt::poses::CPose3D &GT_rel_pose =  GT_KF_poses[dst_kf] - GT_KF_poses[kf];

			EXPECT_NEAR(0, (GT_rel_pose.getAsVectorVal() - rel_pose_incr_st.getAsVectorVal()).array().abs().sum(), 1e-6)
				<< "Error in numeric relative pose of KF " << dst_kf << " from KF " << kf <<":" << endl
				<< "GT_rel_pose: "<< GT_rel_pose << endl
				<< "rel_pose_incr_st: "<< rel_pose_incr_st << endl;
		}
	}

}

size_t Ns[5]={10, 50, 300};

void run_spantree_topology(int topo)
{
	for (uint32_t i=0;i<sizeof(Ns)/sizeof(Ns[0]);i++)
	{
		const size_t N = Ns[i];
		for (uint32_t depth=1;depth<=4;depth++)
		{
			for (uint32_t random_seed=1;random_seed<10;random_seed++)
				test_spantree_topology(topo, N, depth, random_seed);
		}
	}
}


TEST(SpanTreeTests,LinearGraphs)           { run_spantree_topology(0);  }
TEST(SpanTreeTests,LinearGraphsInv)        { run_spantree_topology(100);  }

TEST(SpanTreeTests,LinearGraphsWithLoops)     { run_spantree_topology(1);  }
TEST(SpanTreeTests,LinearGraphsWithLoopsInv)  { run_spantree_topology(101);  }

