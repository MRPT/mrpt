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
#include "pbmap-precomp.h"  // Precompiled headers


#if MRPT_HAS_PCL

#include <mrpt/pbmap/PbMapLocaliser.h>
#include <mrpt/pbmap/SubgraphMatcher.h>
#include <mrpt/pbmap/heuristicParams.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/threads.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace mrpt::pbmap;

double time1, time2;

PbMapLocaliser::PbMapLocaliser(PbMap &mPbM, const string &config_file) :
    mPbMap(mPbM),
    m_pbMapLocaliser_must_stop(false),
    m_pbMapLocaliser_finished(false)
{
  matcher.configLocaliser.load_params(config_file);
std::cout << "PbMapLocaliser::PbMapLocaliser min_planes_recognition " << matcher.configLocaliser.min_planes_recognition << " color thresholds " << matcher.configLocaliser.color_threshold << " " << matcher.configLocaliser.intensity_threshold << " " << matcher.configLocaliser.hue_threshold << endl;

  LoadPreviousPbMaps("/home/edu/newPbMaps/PbMaps.txt");

  pbMapLocaliser_hd = mrpt::system::createThreadFromObjectMethod(this, &PbMapLocaliser::run);
}


// Load Previous PbMaps
void PbMapLocaliser::LoadPreviousPbMaps(std::string fileMaps)
{
cout << "PbMapLocaliser::LoadPreviousPbMaps(...)\n";
  std::ifstream configFile;
  configFile.open(fileMaps.c_str());


  string mapFile, filepath = "/home/edu/newPbMaps/";
  totalPrevPlanes = 0;
  while( !configFile.eof() )
  {
//      floorPlane = -1;
    getline(configFile, mapFile);
    if(mapFile == "")
      break;

    if(mapFile.at(0) == '%')
      continue;
  cout << "Load map: " << filepath << mapFile << endl;

    PbMap previousPbMap;
    previousPbMap.loadPbMap(filepath + mapFile);

    previousPbMapNames.push_back(mapFile);
    previousPbMaps.push_back(previousPbMap);

//      prevPbMap.vPlaness.push_back(vPlanesTemp);
//      previousPbMapNames.push_back(mapFile);
//      vFloors.push_back(floorPlane);
    totalPrevPlanes += previousPbMap.vPlanes.size();
//  cout << "PbMap loaded has " << previousPbMap.vPlanes.size() << " planes" << endl;
  }
//cout << "Size: " << totalPrevPlanes << " " << vFloors.size() << endl;
//cout << "Floors: ";
//for(unsigned i=0; i< vFloors.size(); i++)
//  cout << vFloors[i] << " ";
//cout << endl;

  configFile.close();
cout << "PbMapLocaliser:: previous PbMaps loaded\n";
}

// Check 2nd order neighbors of the reference plane
void PbMapLocaliser::compareSubgraphNeighbors(SubgraphMatcher &matcher)
{
cout << "PbMapLocaliser::compareSubgraphNeighbors\n";
  PbMap &matchedPbMap = previousPbMaps[bestMap];

  for(map<unsigned, unsigned>::iterator it = bestMatch.begin(); it != bestMatch.end(); it++)
  {
  // TODO: use the graph to grow
    if(matcher.configLocaliser.graph_mode == 0) // Nearby neighbors - Check the 2nd order neighbors of the matched planes
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
    else //if(matcher.configLocaliser.graph_mode == 1)// Co-visible neighbors - Check the 2nd order neighbors of the matched planes
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
      if(matcher.configLocaliser.graph_mode == 0)
      {
        if( targetPlane.nearbyPlanes.size() < matcher.configLocaliser.min_planes_recognition ){
          continue;}
      }
      else // matcher.configLocaliser.graph_mode = 1
      {
        if( targetPlane.neighborPlanes.size() < matcher.configLocaliser.min_planes_recognition ){
          continue;}
      }


      Subgraph targetSubgraph(&prevPbMap, targetPlane.id);
//      matcher.setTargetSubgraph(targetSubgraph);

//  cout << "\nsource size " << currentSubgraph.subgraphPlanesIdx.size() << endl;
//  cout << "target size " << targetSubgraph.subgraphPlanesIdx.size() << endl;
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
    cout << "bestMatch size " << bestMatch.size() << " min " << matcher.configLocaliser.min_planes_recognition << endl;
  #endif

  if( bestMatch.size() >= matcher.configLocaliser.min_planes_recognition ) // Assign label of previous map plane
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
    ConsistencyTest fitModel(mPbMap, winnerPbMap);
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
//cout << "run PbMapLocaliser\n";
  bool placeFound = false;

  std::map<unsigned,vector<double> > timeLocalizer;
  std::map<unsigned,vector<int> > nCheckLocalizer;
  time1 = 0.0;
  time2 = 0.0;

  while(!m_pbMapLocaliser_must_stop && !placeFound)
  {
//    cout << "while...\n";
    if(vQueueObservedPlanes.empty()) //if(sQueueObservedPlanes.empty())
      mrpt::system::sleep(50);

    else
    {

      while( !vQueueObservedPlanes.empty() )
      {
        // Do not consider searching planes which do not have more than 2 neighbor planes (too much uncertainty)
        if(matcher.configLocaliser.graph_mode == 0) // Nearby neighbors
        {
//        cout << "vecinos1 " << mPbMap.vPlanes[vQueueObservedPlanes[0]].nearbyPlanes.size() << endl;
          if(mPbMap.vPlanes[vQueueObservedPlanes[0]].nearbyPlanes.size() < matcher.configLocaliser.min_planes_recognition)
          {
            vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());
            continue;
          }
        }
        else //matcher.configLocaliser.graph_mode = 1
        {
//        cout << "vecinos2 " << mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size() << endl;
          if(mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size() < matcher.configLocaliser.min_planes_recognition)
          {
            vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());
            continue;
          }
        }

//    cout << "Search context\n";
        matcher.nCheckConditions = 0;
//        #ifdef _VERBOSE
        double search_plane_start = pcl::getTime ();
//        #endif

        if( searchPlaneContext(mPbMap.vPlanes[vQueueObservedPlanes[0]]) )
        {
          placeFound = true;
//          #ifdef _VERBOSE
            double search_plane_end = pcl::getTime ();
            std::cout << "PLACE FOUND. Search took " << double (search_plane_end - search_plane_start) << " s\n";
//          #endif
          break;
        }

        vQueueObservedPlanes.erase(vQueueObservedPlanes.begin());

//    cout << "nChecks " << matcher.nCheckConditions << endl;
        double search_plane_end = pcl::getTime ();
        if(timeLocalizer.count(mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size()) == 0)
        {
          timeLocalizer[mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size()].push_back( double(search_plane_end - search_plane_start) );
          nCheckLocalizer[mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size()].push_back(matcher.nCheckConditions);
        }
        else
        {
            vector<double> firstElement(1,double(search_plane_end - search_plane_start));
            vector<int> firstElement_(1,matcher.nCheckConditions);
            timeLocalizer[mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size()] = firstElement;
            nCheckLocalizer[mPbMap.vPlanes[vQueueObservedPlanes[0]].neighborPlanes.size()] = firstElement_;
        }

        #ifdef _VERBOSE
          double search_plane_end = pcl::getTime ();
          std::cout << "Search a plane took " << double (search_plane_end - search_plane_start) << std::endl; //<< " or " << clock.Tac() << std::endl;
        #endif
      }
    }
  }
  m_pbMapLocaliser_finished = true;

cout << "Print TIME PbLocalizer\n";
  cout << "Tiempo1 " << time1 << " tiempo2 " << time2 << endl;;
  std::map<unsigned,vector<int> >::iterator itChecks = nCheckLocalizer.begin();
  for(std::map<unsigned,vector<double> >::iterator it=timeLocalizer.begin(); it != timeLocalizer.end(); it++)
  {
    double sum_times = 0;
    double sum_nChecks = 0;
    for(unsigned j=0; j < it->second.size(); j++)
    {
        sum_times += it->second[j];
        sum_nChecks += itChecks->second[j];
    }
    itChecks++;

    cout << "Size " << it->first << " time " << sum_times/it->second.size() << endl;
    cout << "... nChecks " << sum_nChecks/it->second.size() << endl;
  }

}

bool PbMapLocaliser::stop_pbMapLocaliser()
{
  m_pbMapLocaliser_must_stop = true;
  while(!m_pbMapLocaliser_finished)
    mrpt::system::sleep(1);
  cout << "Waiting for PbMapLocaliser thread to die.." << endl;

  mrpt::system::joinThread(pbMapLocaliser_hd);
	pbMapLocaliser_hd.clear();

	return true;
}

PbMapLocaliser::~PbMapLocaliser()
{
  stop_pbMapLocaliser();
}

#endif
