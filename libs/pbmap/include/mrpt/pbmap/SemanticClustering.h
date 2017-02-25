/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
#include <vector>
#include <set>
#include <map>

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

    unsigned currentSemanticGroup;

    PbMap &mPbMap;

    std::vector<unsigned> neighborGroups;

    std::map<unsigned, std::vector<unsigned> > groups; //[group][planeID]

    std::map<unsigned, std::vector<unsigned> > vicinity; //[group][neighborGroup]

    mrpt::math::CMatrix connectivity_matrix;

    std::vector<unsigned> planesVicinity_order;

    /*!
    * Build the proximity matrix
    */
    void buildProximityMatrix()
    {
      size_t neigSize = 0;
      planesVicinity_order.clear();

      // Set the Vicinity (planes of the current group and its neighbors)
      neighborGroups = vicinity[currentSemanticGroup];
      neighborGroups.push_back(currentSemanticGroup);
//    cout << "neighborGroups: ";
      for(unsigned i=0; i < neighborGroups.size(); i++)
      {
//      cout << neighborGroups[i] << " ";
        neigSize += groups[neighborGroups[i] ].size();
        planesVicinity_order.insert(planesVicinity_order.end(), groups[neighborGroups[i] ].begin(), groups[neighborGroups[i] ].end());
      }
//    cout << endl;

      // Fill the matrix
    assert(neigSize <= mPbMap.vPlanes.size());
      connectivity_matrix.resize(neigSize,neigSize);
      connectivity_matrix.zeros();
      for(unsigned i=0; i < planesVicinity_order.size(); i++)
      {
        unsigned plane_i = planesVicinity_order[i];
        for(unsigned j=i+1; j < planesVicinity_order.size(); j++)
        {
          unsigned plane_j = planesVicinity_order[j];
          if(mPbMap.vPlanes[plane_i].nearbyPlanes.count(plane_j))
            connectivity_matrix(i,j) = connectivity_matrix(j,i) = 1;
        }

      }
//cout << "Planes in the vicinity: ";
//for(unsigned i=0; i < planesVicinity_order.size(); i++)
//  cout << planesVicinity_order[i] << " ";
//cout << endl;

//for(unsigned i=0; i < planesVicinity_order.size(); i++)
//{
//  cout << "\nNeighbors of " << planesVicinity_order[i] << " obs " << mPbMap.vPlanes[planesVicinity_order[i]].numObservations << ":\n";
//  for(map<unsigned,unsigned>::iterator it=mPbMap.vPlanes[planesVicinity_order[i]].neighborPlanes.begin();
//        it != mPbMap.vPlanes[planesVicinity_order[i]].neighborPlanes.end(); it++)
//    cout << it->first << " " << it->second << endl;
//}
//cout << endl;

cout << "connectivity_matrix matrix\n" << connectivity_matrix << endl;

    };

    /*!
    * Build the co-visibility matrix
    */
    void buildCoVisibilityMatrix()
    {
      size_t neigSize = 0;
      planesVicinity_order.clear();

      // Set the Vicinity (planes of the current group and its neighbors)
      neighborGroups = vicinity[currentSemanticGroup];
      neighborGroups.push_back(currentSemanticGroup);
    cout << "neighborGroups: ";
      for(unsigned i=0; i < neighborGroups.size(); i++)
      {
      cout << neighborGroups[i] << " ";
        neigSize += groups[neighborGroups[i] ].size();
        planesVicinity_order.insert(planesVicinity_order.end(), groups[neighborGroups[i] ].begin(), groups[neighborGroups[i] ].end());
      }
    cout << endl;

      // Fill the matrix
    assert(neigSize <= mPbMap.vPlanes.size());
      connectivity_matrix.resize(neigSize,neigSize);
      connectivity_matrix.zeros();
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
            connectivity_matrix(i,j) = connectivity_matrix(j,i) = sso;
          }
        }

      }
cout << "Planes in the vicinity: ";
for(unsigned i=0; i < planesVicinity_order.size(); i++)
  cout << planesVicinity_order[i] << " ";
cout << endl;

//for(unsigned i=0; i < planesVicinity_order.size(); i++)
//{
//  cout << "\nNeighbors of " << planesVicinity_order[i] << " obs " << mPbMap.vPlanes[planesVicinity_order[i]].numObservations << ":\n";
//  for(map<unsigned,unsigned>::iterator it=mPbMap.vPlanes[planesVicinity_order[i]].neighborPlanes.begin();
//        it != mPbMap.vPlanes[planesVicinity_order[i]].neighborPlanes.end(); it++)
//    cout << it->first << " " << it->second << endl;
//}
//cout << endl;

cout << "connectivity_matrix matrix\n" << connectivity_matrix << endl;

    };

    /*!
    * Arrange the semantic groups
    */
    void arrangeNewGroups(std::vector<mrpt::vector_uint> &parts)
    {
	  using namespace std;
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
//cout << "Size new groups " << newGroups.size();
      std::vector<unsigned> group_limits(1,0);
//cout << "group_limits_: ";
//  for(unsigned i=0; i < group_limits.size(); i++)
//    cout << group_limits[i] << " ";
//cout << endl;
      for(unsigned i=0; i < neighborGroups.size(); i++)
        group_limits.push_back(group_limits.back() + groups[neighborGroups[i]].size());
//cout << "group_limits: ";
//  for(unsigned i=0; i < group_limits.size(); i++)
//    cout << group_limits[i] << " ";
//cout << endl;

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
//                cout << parts[i][j] << " swithces to group " << neighborGroups[k] << endl;
                  newGroups[neighborGroups[k]].push_back(mPbMap.vPlanes[groups[neighborGroups[k]][parts[i][j]-group_limits[k]]].id);
                  mPbMap.vPlanes[groups[neighborGroups[k]][parts[i][j]-group_limits[k]]].semanticGroup = neighborGroups[k];
                }
            }
            else // The plane remains in its group
            {
//            cout << parts[i][j] << " remains in group " << neighborGroups[i] << endl;
              newGroups[neighborGroups[i]].push_back(mPbMap.vPlanes[groups[neighborGroups[i]][parts[i][j]-group_limits[i]]].id);
//              mPbMap.vPlanes[groups[neighborGroups[k]][parts[i][j]-group_limits[k]]].semanticGroup = neighborGroups[k];
            }
          }
          else
          {
            newGroups[groups.size()+i-neighborGroups.size()].push_back(planesVicinity_order[parts[i][j]]);
            mPbMap.vPlanes[planesVicinity_order[parts[i][j]]].semanticGroup = groups.size()+i-neighborGroups.size();
//          cout << parts[i][j] << " swithces to NEW group " << groups.size()+i-neighborGroups.size() << endl;

          }
        }
      }

      for(map<unsigned,vector<unsigned> >::iterator it=newGroups.begin(); it != newGroups.end(); it++)
        groups[it->first] = it->second;

      if(group_diff < 0)
      {
        int sizeVicinity = neighborGroups.size();
        for(int i=-1; i >= group_diff; i--)
        {
          if(neighborGroups[sizeVicinity+i] == groups.size()-1)
            groups.erase(neighborGroups[sizeVicinity+i]);
          else
          {
            for(unsigned j=0; j < mPbMap.vPlanes.size(); j++)
              if(mPbMap.vPlanes[j].semanticGroup > neighborGroups[sizeVicinity+i])
                mPbMap.vPlanes[j].semanticGroup--;
            for(unsigned j=neighborGroups[sizeVicinity+i]; j < groups.size()-1; j++)
              groups[j] = groups[j+1];
            groups.erase(groups.size()-1);
          }
        }
      }

//cout << "Updated arrangement of groups\n";
//for(map<unsigned,vector<unsigned> >::iterator it=groups.begin(); it != groups.end(); it++)
//{
//  cout << "group " << it->first << ": ";
//  for(unsigned i=0; i < it->second.size(); i++)
//    cout << it->second[i] << " ";
//  cout << endl;
//}

      // Re-define currentSemanticGroup and current vicinity
//      std::vector<unsigned> newNeighborGroups;

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

      // TODO: Re-define second order vicinity

    }

   public:

    friend class PbMapMaker;

    // Constructor
    SemanticClustering(PbMap &mPbM) :
        currentSemanticGroup(0),
        mPbMap(mPbM)
    {
      vicinity[0] = DEFAULT_VECTOR_U;
    };

    /*!
    * Evaluate the partition of the current semantic groups into new ones with minNcut
    */
    int evalPartition(std::set<unsigned> &observedPlanes)
    {
    	using namespace std;
//      mrpt::utils::CTicTac time;
//      time.Tic();
//

      // Select current group
      unsigned current_group_votes = 0;
      map<unsigned, unsigned> observed_group;
      for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
        if(observed_group.count(mPbMap.vPlanes[*it].semanticGroup))
          observed_group[mPbMap.vPlanes[*it].semanticGroup ]++;
        else
          observed_group[mPbMap.vPlanes[*it].semanticGroup ] = 1;
      for(map<unsigned, unsigned>::iterator it = observed_group.begin(); it != observed_group.end(); it++)
        if(it->second > current_group_votes)
        {
          currentSemanticGroup = it->first;
          current_group_votes = it->second;
        }
//      cout << "currentSemanticGroup " << currentSemanticGroup << endl;

//      buildCoVisibilityMatrix();
      buildProximityMatrix();
      std::vector<mrpt::vector_uint> parts;  // Vector of vectors to keep the KFs index of the different partitions (submaps)
      mrpt::graphs::CGraphPartitioner<mrpt::math::CMatrix>::RecursiveSpectralPartition(connectivity_matrix, parts, 0.8, false, true, true, 1);

//    cout << "Time RecursiveSpectralPartition " << time.Tac()*1000 << "ms" << endl;

      if(neighborGroups.size() == 1 && parts.size() == 1)
        return 1;

      // Check if this partition produces any change over the previous group structure
      bool different_partition = false;
      if(neighborGroups.size() != parts.size())
        different_partition = true;
      else
      {
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
      }

      if(!different_partition)
        return -1;

      arrangeNewGroups(parts);

      // Update currentSemanticGroup
      current_group_votes = 0;
      observed_group.clear();
      for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
        if(observed_group.count(mPbMap.vPlanes[*it].semanticGroup))
          observed_group[mPbMap.vPlanes[*it].semanticGroup ]++;
        else
          observed_group[mPbMap.vPlanes[*it].semanticGroup ] = 1;
      for(map<unsigned, unsigned>::iterator it = observed_group.begin(); it != observed_group.end(); it++)
        if(it->second > current_group_votes)
        {
          currentSemanticGroup = it->first;
          current_group_votes = it->second;
        }
//cout << "Updated currentSemanticGroup " << currentSemanticGroup << endl;
//
//cout << "Planes' semantics2: ";
//for(unsigned i=0; i < mPbMap.vPlanes.size(); i++)
//  cout << mPbMap.vPlanes[i].semanticGroup << " ";
//cout << endl;
      return parts.size();
    };

  };

} } // End of namespaces

#endif
#endif
