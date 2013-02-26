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

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __SemanticClustering_H
#define __SemanticClustering_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils.h>
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/pbmap/link_pragmas.h>

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/Plane.h>

static std::vector<unsigned> DEFAULT_VECTOR_U;

namespace mrpt {
namespace pbmap {

  /*! This class arranges the planes of a PbMap into groups according to a co-visibility measure
   * The groups are divided using graph minimum normaized-cut (minNcut). The resulting groups can
   * be used to represent semantic groups corresponding human identifiable structures such as rooms,
   * or environments as an office. These groups of closely related planes can be used also for
   * relocalization.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PBMAP_IMPEXP SemanticClustering
  {
   private:

    PbMap &mPbMap;

    unsigned currentSemanticGroup;

    std::vector<unsigned> neighborGroups;

    std::map<unsigned, std::vector<unsigned> > groups; //[group][planeID]

    std::map<unsigned, std::vector<unsigned> > vicinity; //[group][neighborGroup]

    mrpt::math::CMatrix co_visibility;

    std::vector<unsigned> planesVicinity_order;


    /*!
    * Build the co-visibility matrix
    */
    void buildCoVisibilityMatrix()
    {
      size_t neigSize = groups[currentSemanticGroup].size();

      // Set the Vicinity (planes of the current group and its neighbors)
//      std::vector<unsigned> planesVicinity_order;
      neighborGroups = vicinity[currentSemanticGroup];
      neighborGroups.push_back(currentSemanticGroup);
      for(unsigned i=0; i < neighborGroups.size(); i++)
      {
        neigSize += groups[neighborGroups[i] ].size();
        planesVicinity_order.insert(planesVicinity_order.end(), groups[neighborGroups[i] ].begin(), groups[neighborGroups[i] ].end());
      }

      // Fill the matrix
    assert(neigSize <= mPbMap.vPlanes.size());
      co_visibility.resize(neigSize,neigSize);
      co_visibility.zeros();
      for(unsigned i=0; i < planesVicinity_order.size(); i++)
      {
        unsigned plane_i = planesVicinity_order[i];
        for(unsigned j=i+1; j < planesVicinity_order.size(); j++)
        {
          unsigned plane_j = planesVicinity_order[j];
          if(mPbMap.vPlanes[plane_i].neighborPlanes.count(plane_j))
          {
            // Calculate the co-visibility index
            float sso = (float)mPbMap.vPlanes[plane_i].neighborPlanes[plane_j] / std::min(mPbMap.vPlanes[plane_i].numObservations, mPbMap.vPlanes[plane_j].numObservations);
            co_visibility(i,j) = co_visibility(j,i) = sso;
          }
        }

      }

    };

    /*!
    * Arrange the semantic groups
    */
    void arrangeNewGroups(std::vector<mrpt::vector_uint> &parts)
    {
      int group_diff = parts.size() - neighborGroups.size();
      std::map<unsigned, std::vector<unsigned> > newGroups;

      if(group_diff > 0) // Create new groups
      {
        for(unsigned i=0; i < neighborGroups.size(); i++)
          newGroups[neighborGroups[i]] = DEFAULT_VECTOR_U;
        for(int i=0; i < group_diff; i++)
          newGroups[groups.size()+i] = DEFAULT_VECTOR_U;
//          groups[groups.size()] = DEFAULT_VECTOR_U;
      }
      else if(group_diff < 0) // Discard old groups
      {
        for(unsigned i=0; i < parts.size(); i++)
          newGroups[neighborGroups[i]] = DEFAULT_VECTOR_U;
      }
      else // Re-arrange previous groups
      {
        for(unsigned i=0; i < neighborGroups.size(); i++)
          newGroups[neighborGroups[i]] = DEFAULT_VECTOR_U;
      }

      std::vector<unsigned> group_limits(1,0);
      for(unsigned i=0; i < neighborGroups.size(); i++)
        group_limits.push_back(group_limits.back() + groups[neighborGroups[i]].size());

      // Re-assign plane's semanticGroup and groups
      for(unsigned i=0; i < parts.size(); i++)
      {
        for(unsigned j=0; j < parts[i].size(); j++)
        {
          if(i < neighborGroups.size())
          {
            if(parts[i][j] < group_limits[i] || parts[i][j] >= group_limits[i+1]) // The plane has changed the group
            {
              for(unsigned k=0; k < neighborGroups.size() && i != k; k++)
                if(parts[i][j] >= group_limits[k] || parts[i][j] < group_limits[k+1]) // The plane has changed the group
                {
                assert(groups[neighborGroups[k]][parts[i][j]-group_limits[k]] == planesVicinity_order[parts[i][j]]);
                  newGroups[neighborGroups[k]].push_back(mPbMap.vPlanes[groups[neighborGroups[k]][parts[i][j]-group_limits[k]]].id);
                }
            }
            else // The plane remains in its group
            {
              newGroups[neighborGroups[i]].push_back(mPbMap.vPlanes[groups[neighborGroups[i]][parts[i][j]-group_limits[i]]].id);
            }
          }
          else
          {
            for(unsigned k=0; k < neighborGroups.size(); k++)
              if(parts[i][j] >= group_limits[k] || parts[i][j] < group_limits[k+1]) // The plane has changed the group
              {
              assert(groups[neighborGroups[k]][parts[i][j]-group_limits[k]] == planesVicinity_order[parts[i][j]]);
                newGroups[neighborGroups[k]].push_back(mPbMap.vPlanes[groups[neighborGroups[k]][parts[i][j]-group_limits[k]]].id);
              }
          }

        }
      }

      for(unsigned i=0; i < newGroups.size(); i++)
        groups[neighborGroups[i]] = newGroups[neighborGroups[i]];

      if(group_diff < 0)
      {
        int sizeVicinity = neighborGroups.size();
        for(int i=-1; i >= group_diff; i--)
        {
          if(neighborGroups[sizeVicinity+i] == groups.size()-1)
            groups.erase(neighborGroups[sizeVicinity+i]);
          else
          {
            for(unsigned j=neighborGroups[sizeVicinity+i]; j < groups.size()-1; j++)
              groups[j] = groups[j+1];
            groups.erase(groups.size()-1);
          }
        }
      }

      // Re-define currentSemanticGroup and current vicinity
      std::vector<unsigned> newNeighborGroups;

      for(std::map<unsigned, std::vector<unsigned> >::iterator it1=newGroups.begin(); it1 != newGroups.end(); it1++)
        vicinity[it1->first] = DEFAULT_VECTOR_U;

      for(std::map<unsigned, std::vector<unsigned> >::iterator it1=newGroups.begin(); it1 != newGroups.end(); it1++)
      {
        std::map<unsigned, std::vector<unsigned> >::iterator it2 = it1;
        for( it2++; it2 != newGroups.end(); it2++)
          for(unsigned i=0; i < it1->second.size(); i++)
          {
            bool linked = false;
            for(unsigned j=0; j < it2->second.size(); j++)
            {
              if(mPbMap.vPlanes[it1->second[i]].neighborPlanes.count(mPbMap.vPlanes[it2->second[j]].id))
              {
                vicinity[it1->first].push_back(it2->first);
                vicinity[it2->first].push_back(it1->first);
                linked = true;
                break;
              }
            }
            if(linked)
              break;
          }
      }

      // Re-define second order vicinity

    }

   public:

    friend class PbMapMaker;

    // Constructor
    SemanticClustering(PbMap &mPbM) :
        mPbMap(mPbM)
    {
      vicinity[0] = DEFAULT_VECTOR_U;
    };

    /*!
    * Evaluate the partition of the current semantic groups into new ones with minNcut
    */
    int evalPartition(unsigned &current_group)
    {
//      mrpt::utils::CTicTac time;
//      time.Tic();
//
      buildCoVisibilityMatrix();
      std::vector<mrpt::vector_uint> parts;  // Vector of vectors to keep the KFs index of the different partitions (submaps)
      mrpt::graphs::CGraphPartitioner<mrpt::math::CMatrix>::RecursiveSpectralPartition(co_visibility, parts, 0.8, false, true, true);

//    cout << "Time RecursiveSpectralPartition " << time.Tac()*1000 << "ms" << endl;

      // Check if this partition produces any change over the previous group structure
      bool different_partition = false;
      unsigned prev_size = 0;
      for(unsigned i=0; i < neighborGroups.size(); i++)
      {
        if(groups[neighborGroups[i] ].size() != parts[i].size())
        {
          different_partition = true;
          break;
        }

        for(unsigned j=0; j < parts[i].size(); j++)
        {
          if(parts[i][j] != j+prev_size)
          {
            different_partition = true;
            break;
          }
        }
        prev_size += parts[i].size();
      }

      if(different_partition)
        arrangeNewGroups(parts);

      return parts.size();
    };

  };

} } // End of namespaces

#endif
#endif
