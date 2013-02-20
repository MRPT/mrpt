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

#include "../include/SubgraphMatcher.h"

using namespace pbmap;

//#define RAD2DEG 57.2958

extern config_heuristics configLocaliser;

/**!
 * Check if the two input planes could be the same
*/
bool SubgraphMatcher::evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure)
{
  // Main color
  if( fabs(plane1.v3colorNrgb[0] - plane2.v3colorNrgb[0]) > configLocaliser.color_threshold ||
      fabs(plane1.v3colorNrgb[1] - plane2.v3colorNrgb[1]) > configLocaliser.color_threshold ||
      fabs(plane1.v3colorNrgb[2] - plane2.v3colorNrgb[2]) > configLocaliser.color_threshold )
    return false;

  ++nCheckConditions;

  if(plane1.bFromStructure && plane2.bFromStructure)
    return true;

  double rel_areas = plane1.areaVoxels/ plane2.areaVoxels;
//cout << "rel_areas " << rel_areas << endl;
  double rel_ratios = plane1.elongation / plane2.elongation;

  // If the plane has been fully detected use a narrower threshold for the comparison
  if(plane1.bFullExtent && plane2.bFullExtent)
  {
    if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
//      cout << "rel_areas full " << rel_areas << endl;
      return false;}
  ++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_ratios < configLocaliser.elongation_threshold_inv || rel_ratios > configLocaliser.elongation_threshold ){
//      cout << "rel_ratios full " << rel_ratios << endl;
      return false;}
  ++nCheckConditions;
  }
  else
  {
    if(plane1.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_threshold ){
//        cout << "rel_areas RefFull " << rel_areas << endl;
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
//        cout << "rel_areas CheckFull " << rel_areas << endl;
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
//      cout << "rel_areas simple " << rel_areas << endl;
      return false;}
  ++nCheckConditions;

    if( rel_ratios < configLocaliser.elongation_threshold_inv || rel_ratios > configLocaliser.elongation_threshold ){
//      cout << "rel_ratios simple " << rel_ratios << endl;
      return false;}
  ++nCheckConditions;
  }

  // TODO. Use the inferred semantic information to control the search

  return true;
}

/**!
 * Compares the relation between Ref-neigRef with the relation between Check-neigCheck. Returns true if both geometries are similar
*/
bool SubgraphMatcher::evalBinaryConstraints(Plane &Ref, Plane &neigRef, Plane &Check, Plane &neigCheck)
{
  // Check height
  double dif_height = Ref.v3normal .dot (neigRef.v3center - Ref.v3center) - Check.v3normal .dot (neigCheck.v3center - Check.v3center);
  if(dif_height > configLocaliser.height_threshold){
//      cout << "dif_height parallel " << dif_height << endl;
    return false;}
  ++nCheckConditions;

  double dif_height2 = neigRef.v3normal .dot (Ref.v3center - neigRef.v3center) - neigCheck.v3normal .dot (Check.v3center - neigCheck.v3center);
  if(dif_height2 > configLocaliser.height_threshold){
//      cout << "dif_height parallel " << dif_height << endl;
    return false;}
  ++nCheckConditions;

  // Normal
  double dif_normal = fabs(RAD2DEG( acos( Ref.v3normal .dot (neigRef.v3normal)) - acos( Check.v3normal .dot (neigCheck.v3normal)) ) );
//cout << "dif_normal " << dif_normal << endl;
  if( dif_normal > configLocaliser.angle_threshold ){
//    cout << "dif_normal " << dif_normal << endl;
    return false;}
  ++nCheckConditions;

  // Relative distance
  double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center) .dot (Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center) .dot (Check.v3center - neigCheck.v3center)) );
//cout << "rel_dist_centers " << rel_dist_centers << endl;

  // If the plane has been fully detected use a narrower threshold for the comparison
  bool RefBothFull = (Ref.bFullExtent && neigRef.bFullExtent);// ? true : false;
  bool CheckBothFull = (Check.bFullExtent && neigCheck.bFullExtent);// ? true : false;
  bool RefBothShape = (Ref.elongation > 1.3 && neigRef.elongation > 1.3);
  bool CheckBothShape = (Check.elongation > 1.3 && neigCheck.elongation > 1.3);

  if(configLocaliser.use_completeness)
  {
    if(RefBothFull && CheckBothFull)
    {
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
//        cout << "rel_dist all-full" << rel_dist_centers << endl;
        return false;}
  ++nCheckConditions;
    }
    else if(RefBothFull || CheckBothFull)
    {
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
//        cout << "rel_dist half-full " << rel_dist_centers << endl;
        return false;}
  ++nCheckConditions;
    }
    else if( !Ref.bFromStructure && !Check.bFromStructure && !neigRef.bFromStructure && !neigCheck.bFromStructure )
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
//        cout << "rel_dist no-pair-full " << rel_dist_centers << endl;
        return false;}
  ++nCheckConditions;
  }
  else
  {
    if( Ref.areaVoxels< 1 && neigCheck.areaVoxels< 1 && Check.areaVoxels< 1 && neigRef.areaVoxels< 1 ){ // Use the restriction only when all the planes involved are smaller than 1m2
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold){
//        cout << "rel_dist areas<1 " << rel_dist_centers << endl;
        return false;}
  ++nCheckConditions;
    }
//    else if( !Ref.bFromStructure && !Check.bFromStructure && !neigRef.bFromStructure && !neigCheck.bFromStructure )
//      if( rel_dist_centers < configLocaliser.area_threshold_inv || rel_dist_centers > configLocaliser.area_threshold){
////        cout << "rel_dist no-pair-full " << rel_dist_centers << endl;
//        return false;}
  }

  // We do not check ppal direction constraint -> It has demonstrated to be very little distinctive

//cout << "evalBinaryConstraints TRUE\n";

  return true;
}

bool isSubgraphContained(map<unsigned, unsigned> &contained, map<unsigned, unsigned> &container)
{
  if( contained.size() > container.size() )
    return false;

//cout << "isSubgraphContained\n";
  for(map<unsigned, unsigned>::iterator it = contained.begin(); it != contained.end(); it++)
    if(container.count(it->first) == 0)
      return false;
    else if(container[it->first] != it->second)
      return false;

  #ifdef _VERBOSE
    cout << "Repeated sequence. Comparing:\n";
    for(map<unsigned, unsigned>::iterator it = contained.begin(); it != contained.end(); it++)
      cout << it->first << " " << it->second << endl;
    cout << "with\n";
    for(map<unsigned, unsigned>::iterator it = container.begin(); it != container.end(); it++)
      cout << it->first << " " << it->second << endl;
  #endif

  return true;
}


/**!
 * Recursive function that checks all the relations (direct and crossed) in the neighborhood of a plane.
 * This function make redundant checks and therefore is NOT efficient at all
 */
// TODO. A possible way to make this more efficient is by making an exclusion LUT according to single relations between planes
void SubgraphMatcher::exploreSubgraphTreeR(set<unsigned> &sourcePlanes, set<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
{
  #ifdef _VERBOSE
    cout << "exploreSubgraphTreeR...\n";// << subgraphSrc->subgraphPlanesIdx.size() << endl;
    cout << "matched:\n";
    for(map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
      cout << it->first << " - " << it->second << " =" << subgraphTrg->pPBM->vPlanes[it->second].label << endl;
    cout << "sourcePlanes: ";
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      cout << *it1 << " ";
    cout << "\ntargetPlanes: ";
    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
      cout << subgraphTrg->pPBM->vPlanes[*it2].label << " ";
    cout << endl;
  #endif

  // Stop the search when we find a match of maximum size
  if( winnerMatch.size() == subgraphSrc->subgraphPlanesIdx.size() || winnerMatch.size() == subgraphTrg->subgraphPlanesIdx.size() )
    return;

  int requiredMatchable = (int)configLocaliser.min_planes_recognition - matched.size();
  if( sourcePlanes.empty() || targetPlanes.empty()
     || sourcePlanes.size() < requiredMatchable || targetPlanes.size() < requiredMatchable ) // New condition to speed up the search when there are not a minimum number of candidates
  {
//  cout << "End branch recursive search. matched " << matched.size() << " prev winner " << winnerMatch.size() << endl;
    if(matched.size() > winnerMatch.size())
      winnerMatch = matched;
    return;
  }

  while(!sourcePlanes.empty())
  {
    set<unsigned>::iterator it1 = sourcePlanes.begin();
    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
    {
      bool alreadyEval = false;
      for(unsigned i=0; i<alreadyExplored.size(); i++)
      {
        map<unsigned, unsigned> checkMatch = matched;
        checkMatch[*it1] = *it2;
        if( isSubgraphContained(checkMatch, alreadyExplored[i]) )
        {
//          cout << "Combination already evaluated\n";
          alreadyEval = true;
          break;
        }
      }
      if(alreadyEval)
        continue;

//    cout << "Test edge " << *it1 << " - " << *it2 << endl;
      // Check that it1 and it2 correspond to the same plane
      if( !evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
        continue;

      bool binaryFail = false;
      for(map<unsigned, unsigned>::iterator it_matched = matched.begin(); it_matched != matched.end(); it_matched++)
        if( !evalBinaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphSrc->pPBM->vPlanes[it_matched->first], subgraphTrg->pPBM->vPlanes[*it2], subgraphTrg->pPBM->vPlanes[it_matched->second]) )
        {
          binaryFail = true;
          break;
        }
      if(binaryFail)
        continue;

//    cout << "Match edge\n";
      // If this point is reached, the planes it1 and it2 are candidates to be the same
      set<unsigned> nextSrcPlanes = sourcePlanes;
      nextSrcPlanes.erase(*it1);
      set<unsigned> nextTrgPlanes = targetPlanes;
      nextTrgPlanes.erase(*it2);
      map<unsigned, unsigned> nextMatched = matched;
      nextMatched[*it1] = *it2;

      alreadyExplored.push_back(nextMatched);

      exploreSubgraphTreeR(nextSrcPlanes, nextTrgPlanes, nextMatched);

//      // CHANGE to be As in the algorithm of our article
//      if(matched.size() > bestCombination.size())
//        return bestCombination;

    }
    sourcePlanes.erase(it1);
  }
}

std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget)
{
  subgraphSrc = &subgraphSource;
  subgraphTrg = &subgraphTarget;
  map<unsigned, unsigned> matched;

  winnerMatch.clear();
  alreadyExplored.clear(); // MODIFICAR: Tener en cuenta caminos explorados cuando exploramos grafos vecinos
  std::set<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
  std::set<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;
  exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);

  return winnerMatch;
}
