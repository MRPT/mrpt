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

#include <mrpt/pbmap.h> // precomp. hdr

#if MRPT_HAS_PCL

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/threads.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace mrpt::pbmap;

config_heuristics configLocaliser;

PbMapLocaliser::PbMapLocaliser(PbMap &mPbM) :
    mPbMap(mPbM),
    m_pbMapLocaliser_must_stop(false),
    m_pbMapLocaliser_finished(false)
{
std::cout << "PbMapLocaliser::PbMapLocaliser min_planes_recognition " << configLocaliser.min_planes_recognition << endl;

  pbMapLocaliser_hd = mrpt::system::createThreadFromObjectMethod(this, &PbMapLocaliser::run);
}

// Check 2nd order neighbors of the reference plane
void PbMapLocaliser::compareSubgraphNeighbors(SubgraphMatcher &matcher)
{
cout << "PbMapLocaliser::compareSubgraphNeighbors\n";
  PbMap &matchedPbMap = previousPbMaps[bestMap];

  for(map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
  {
  // TODO: use the graph to grow
    if(!configLocaliser.graph_rule) // Nearby neighbors - Check the 2nd order neighbors of the matched planes
      for(set<unsigned>::iterator it1 = mPbMap.vPlanes[it->first].nearbyPlanes.begin(); it1 != mPbMap.vPlanes[it->first].nearbyPlanes.end(); it1++)
      {
        if(matcher.subgraphSrc->subgraphPlanesIdx.count(*it1) )
          continue;

        for(set<unsigned>::iterator it2 = matchedPbMap.vPlanes[it->second].nearbyPlanes.begin(); it2 != matchedPbMap.vPlanes[it->second].nearbyPlanes.end(); it2++)
        {
          if(matcher.subgraphTrg->subgraphPlanesIdx.count(*it2) )
            continue;

          if( !matcher.evalUnaryConstraints(mPbMap.vPlanes[*it1], matchedPbMap.vPlanes[*it2], matchedPbMap, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
            continue;
          for(map<unsigned, unsigned>::iterator it_matched = bestMatch.begin(); it_matched != bestMatch.end(); it_matched++)
            if( !matcher.evalBinaryConstraints(mPbMap.vPlanes[*it1], mPbMap.vPlanes[it_matched->first], matchedPbMap.vPlanes[*it2], matchedPbMap.vPlanes[it_matched->second]) )
              continue;

          // Asign new neighbor
          bestMatch[*it1] = *it2;
        }
      }
    else // Co-visible neighbors - Check the 2nd order neighbors of the matched planes
      for(map<unsigned, unsigned>::iterator it1 = mPbMap.vPlanes[it->first].neighborPlanes.begin(); it1 != mPbMap.vPlanes[it->first].neighborPlanes.end(); it1++)
      {
        if(matcher.subgraphSrc->subgraphPlanesIdx.count(it1->first) )
          continue;

        for(map<unsigned, unsigned>::iterator it2 = matchedPbMap.vPlanes[it->second].neighborPlanes.begin(); it2 != matchedPbMap.vPlanes[it->second].neighborPlanes.end(); it2++)
        {
          if(matcher.subgraphTrg->subgraphPlanesIdx.count(it2->first) )
            continue;

          if( !matcher.evalUnaryConstraints(mPbMap.vPlanes[it1->first], matchedPbMap.vPlanes[it2->first], matchedPbMap, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
            continue;
          for(map<unsigned, unsigned>::iterator it_matched = bestMatch.begin(); it_matched != bestMatch.end(); it_matched++)
            if( !matcher.evalBinaryConstraints(mPbMap.vPlanes[it1->first], mPbMap.vPlanes[it_matched->first], matchedPbMap.vPlanes[it2->first], matchedPbMap.vPlanes[it_matched->second]) )
              continue;

          // Asign new neighbor
          bestMatch[it1->first] = it2->first;
        }
      }
  }
}

double PbMapLocaliser::getAreaMatch()
{
  double area = 0.0;
  for(map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
    area += mPbMap.vPlanes[it->first].areaVoxels;

  #ifdef _VERBOSE
    cout << "getAreaMatch " << area << endl;
  #endif

  return area;
}


/**!
 * Searches the input plane in the rest of planes of the map taking into account the neighboring relations
*/
bool PbMapLocaliser::searchPlaneContext(Plane &searchPlane)
{
  #ifdef _VERBOSE
    cout << "\nSearching plane P" << searchPlane.id << " numNeigs " << searchPlane.neighborPlanes.size()
        << " in map composed of " << totalPrevPlanes << " planes\n";
  #endif

  // REVISAR: Esto cambia el codigo anterior (antes la vecindad actual contenia todos los vecinos de los planos observados
  Subgraph currentSubgraph(&mPbMap, searchPlane.id);
//  matcher.setSourceSubgraph(currentSubgraph);

  for(size_t mapId=0; mapId < previousPbMaps.size(); mapId++)
  {
    PbMap &prevPbMap = previousPbMaps[mapId];

    for(size_t i=0; i < prevPbMap.vPlanes.size(); i++)
    {
      Plane &targetPlane = prevPbMap.vPlanes[i];

      // Do not consider evaluating planes which do not have more than min_planes_recognition neighbor planes (too much uncertainty)
      if(!configLocaliser.graph_rule)
      {
        if( targetPlane.neighborPlanes.size() < configLocaliser.min_planes_recognition ){
          continue;}
      }
      else
      {
        if( targetPlane.nearbyPlanes.size() < configLocaliser.min_planes_recognition ){
          continue;}
      }


      Subgraph targetSubgraph(&prevPbMap, targetPlane.id);
//      matcher.setTargetSubgraph(targetSubgraph);

//      cout << "\nsource size " << currentSubgraph.subgraphPlanesIdx.size() << endl;
      std::map<unsigned, unsigned> resultingMatch = matcher.compareSubgraphs(currentSubgraph, targetSubgraph);

//    cout << "ResultingMatch size " << resultingMatch.size() << endl;
      if( resultingMatch.size() > bestMatch.size())
      {
        bestMap = mapId;
        bestMatch = resultingMatch;
      }

      // Evaluate Score of the matched places
//      if( score > maxScore)
//      {
//        max2ndScore = maxScore;
//        maxScore = score;
//        winnerPlane = targetPlane.label;
//      }
  //    score_results[i] = comparePlanes(searchPlane, targetPlane);
    }
  }

  #ifdef _VERBOSE
    cout << "bestMatch size " << bestMatch.size() << " min " << configLocaliser.min_planes_recognition << endl;
  #endif

  if( bestMatch.size() >= configLocaliser.min_planes_recognition ) // Assign label of previous map plane
  {
    #ifdef _VERBOSE
      cout << "Context matched\n";
    #endif

//    compareSubgraphNeighbors(matcher);

    PbMap &winnerPbMap = previousPbMaps[bestMap];

    // We require that the sum of the areas of the matched planes is above a minimum
    if( getAreaMatch() < 3.0 )
      return false;


    pcl::PointXYZ placePos(0,0,0);
    for(map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
    {
      if(mPbMap.vPlanes[it->first].label != winnerPbMap.vPlanes[it->second].label)
      {
        placePos.x += mPbMap.vPlanes[it->first].v3center[0];
        placePos.y += mPbMap.vPlanes[it->first].v3center[1];
        placePos.z += mPbMap.vPlanes[it->first].v3center[2];
        if(planeRecognitionLUT.count(winnerPbMap.vPlanes[it->second].label) )
        {
        cout << "Re-assign plane label\n";
          if( bestMatch.size() >= planeRecognitionLUT[winnerPbMap.vPlanes[it->second].label].second )
          {
            mPbMap.vPlanes[planeRecognitionLUT[winnerPbMap.vPlanes[it->second].label].first].label = ""; // Delete previous label asignment
            mPbMap.vPlanes[it->first].label = winnerPbMap.vPlanes[it->second].label;
            planeRecognitionLUT[winnerPbMap.vPlanes[it->second].label] = pair<int,double>(mPbMap.vPlanes[it->first].id, bestMatch.size());
          }
          else
            mPbMap.vPlanes[it->first].label = "";
        }
        else
        {
          #ifdef _VERBOSE
            cout << "Assign plane label\n";
          #endif

          planeRecognitionLUT[winnerPbMap.vPlanes[it->second].label] = pair<int,double>(mPbMap.vPlanes[it->first].id, bestMatch.size());
          mPbMap.vPlanes[it->first].label = winnerPbMap.vPlanes[it->second].label;
        }
      }
    }

    placePos.x /= bestMatch.size();
    placePos.y /= bestMatch.size();
    placePos.z /= bestMatch.size();
    foundPlaces[previousPbMapNames[bestMap]] = placePos;

    // Check previous assignments. TODO. change the external label based system
//    if(searchPlane.label != winnerPlane)
//    {
//      if(planeRecognitionLUT.count(winnerPlane) )
//      {
//        if( bestMatch.size() >= planeRecognitionLUT[winnerPlane].second )
//        {
//          mPbMap.vPlanes[planeRecognitionLUT[winnerPlane].first].label = ""; // Delete previous label asignment
//          searchPlane.label = winnerPlane;
//          planeRecognitionLUT[winnerPlane] = pair<int,double>(searchPlane.id, bestMatch.size());
//        }
//        else
//          searchPlane.label = "";
//      }
//      else
//      {
//        planeRecognitionLUT[winnerPlane] = pair<int,double>(searchPlane.id, bestMatch.size());
//        searchPlane.label = winnerPlane;
//      }
//    }

  // Superimpose model
    Eigen::Matrix4f rigidTransf;    // Pose of map as from current model
    Eigen::Matrix4f rigidTransfInv; // Pose of model as from current map
    ConsistencyTest fitModel(mPbMap, winnerPbMap, bestMatch);
    rigidTransf = fitModel.getRTwithModel(bestMatch);
    rigidTransfInv = inverse(rigidTransf);

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA>);
    string filename = "/home/edu/Projects/PbMaps/" + previousPbMapNames[bestMap] + "/MapPlanes.pcd";
    reader.read (filename, *model);
    alignedModelPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*model,*alignedModelPtr,rigidTransfInv);

    return true;
  }

  // TODO Has this place been matched before? Check it and if so, decide which is the best match
  return false;
}

void PbMapLocaliser::run()
{
  bool placeFound = false;

  while(!m_pbMapLocaliser_must_stop && !placeFound)
  {
    if(vQueueObservedPlanes.empty()) //if(sQueueObservedPlanes.empty())
      mrpt::system::sleep(50);

    else
    {

      while( !vQueueObservedPlanes.empty() )
      {
        // Do not consider searching planes which do not have more than 2 neighbor planes (too much uncertainty)
        if(!configLocaliser.graph_rule) // Nearby neighbors
        {
          if(mPbMap.vPlanes[vQueueObservedPlanes[0]].nearbyPlanes.size() < configLocaliser.min_planes_recognition)
          {
            vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());
            continue;
          }
        }
        else
          if(mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size() < configLocaliser.min_planes_recognition)
          {
            vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());
            continue;
          }
        matcher.nCheckConditions = 0;
        #ifdef _VERBOSE
        double search_plane_start = pcl::getTime ();
        #endif

        if( searchPlaneContext(mPbMap.vPlanes[vQueueObservedPlanes[0]]) )
        {
          placeFound = true;
          #ifdef _VERBOSE
            double search_plane_end = pcl::getTime ();
            std::cout << "PLACE FOUND. Search took " << double (search_plane_end - search_plane_start) << " s\n";
          #endif
          break;
        }

        vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());

        #ifdef _VERBOSE
          double search_plane_end = pcl::getTime ();
          std::cout << "Search a plane took " << double (search_plane_end - search_plane_start) << std::endl; //<< " or " << clock.Tac() << std::endl;
        #endif
      }
    }
  }
  m_pbMapLocaliser_finished = true;
}

bool PbMapLocaliser::stop_pbMapLocaliser()
{
  m_pbMapLocaliser_must_stop = true;
  while(!m_pbMapLocaliser_finished)
    mrpt::system::sleep(1);
  cout << "Waiting for PbMapMaker thread to die.." << endl;

  mrpt::system::joinThread(pbMapLocaliser_hd);
	pbMapLocaliser_hd.clear();

	return true;
}

PbMapLocaliser::~PbMapLocaliser()
{
  stop_pbMapLocaliser();
}

#endif
