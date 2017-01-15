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
#include <mrpt/utils/utils_defs.h>
#include <mrpt/pbmap/SubgraphMatcher.h>

//#define _VERBOSE 1

extern double time1, time2;

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::pbmap;

// Bhattacharyya histogram distance function
double BhattacharyyaDist_(std::vector<float> &hist1, std::vector<float> &hist2)
{
  assert(hist1.size() == hist2.size());
  double BhattachDist;
  double BhattachDist_aux = 0.0;
  for(unsigned i=0; i < hist1.size(); i++)
    BhattachDist_aux += sqrt(hist1[i]*hist2[i]);

  BhattachDist = sqrt(1 - BhattachDist_aux);

  return BhattachDist;
}

SubgraphMatcher::SubgraphMatcher()
{
//  configLocaliser.load_params("/home/edu/Libraries/mrpt_edu/share/mrpt/config_files/pbmap/configPbMap.ini");
}

/**!
 * Check if the two input planes could be the same
*/
bool SubgraphMatcher::evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure)
{
//Eigen::Vector3f color_dif = plane1.v3colorNrgb - plane2.v3colorNrgb;
//if(plane1.id==6 && plane2.id==8)
//cout << "color_dif \n" << color_dif << endl;
//double tCondition1, tCondition2, tCondition3;
  #if _VERBOSE
  cout << "unary between " << plane1.id << " " << plane2.id << "\n";
  #endif

  //++nCheckConditions;
//  ++totalUnary;
//  // Semantic label condition
//  if( !plane1.label.empty() && !plane2.label.empty() ){
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

//  if( !plane1.label.empty() || !plane2.label.empty() ){
//    if( plane1.label.empty() || plane2.label.empty() )
//      return false;
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

  // Main color
//  if(plane1.bDominantColor)
  {
//cout << "thresholds " << configLocaliser.color_threshold << " " << configLocaliser.intensity_threshold << endl;
    //++nCheckConditions;
//  if(configLocaliser.color_threshold > 0)
//    tCondition1 = pcl::getTime();

//    if( fabs(plane1.v3colorNrgb[0] - plane2.v3colorNrgb[0]) > configLocaliser.color_threshold ||
//        fabs(plane1.v3colorNrgb[1] - plane2.v3colorNrgb[1]) > configLocaliser.color_threshold ||
////        fabs(plane1.v3colorNrgb[2] - plane2.v3colorNrgb[2]) > configLocaliser.color_threshold ||//)
//        fabs(plane1.dominantIntensity - plane2.dominantIntensity) > configLocaliser.intensity_threshold)
//    ;
//    tCondition2 = pcl::getTime();
//
    if( BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) > configLocaliser.hue_threshold ){
    #if _VERBOSE
    cout << "Hist_H false " << BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) << " > " << configLocaliser.hue_threshold << "\n";
    #endif
//    ;
//    tCondition3 = pcl::getTime();

//  time1 += tCondition2 - tCondition1;
//  time2 += tCondition3 - tCondition2;
      return false;}
  }

  //++nCheckConditions;
  if(plane1.bFromStructure && plane2.bFromStructure){//cout << "True: both structure\n";
    return true;}

  double rel_areas = plane1.areaHull/ plane2.areaHull;
//  double rel_areas = plane1.areaVoxels/ plane2.areaVoxels;
//if(plane1.id==6 && plane2.id==8)
//cout << "rel_areas " << rel_areas << endl;
  double rel_elong = plane1.elongation / plane2.elongation;
//  if(plane1.id==6 && plane2.id==8)
//cout << "rel_elong " << rel_elong << endl;

  // If the plane has been fully detected use a narrower threshold for the comparison
  if(plane1.bFullExtent && plane2.bFullExtent)
  {
    if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
    #if _VERBOSE
    cout << "False: rel_areas full " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "False: rel_elong full " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }
  else
  {
    if(plane1.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_threshold ){
      #if _VERBOSE
      cout << "rel_areas RefFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
      #if _VERBOSE
      cout << "rel_areas CheckFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
    #if _VERBOSE
    cout << "rel_areas simple " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "rel_elong simple " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }

  // TODO. Use the inferred semantic information to control the search
  #if _VERBOSE
  cout << "UNARY TRUE" << endl;
  #endif

  return true;
}

bool SubgraphMatcher::evalUnaryConstraintsOdometry(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure)
{
//Eigen::Vector3f color_dif = plane1.v3colorNrgb - plane2.v3colorNrgb;
//if(plane1.id==6 && plane2.id==8)
//cout << "color_dif \n" << color_dif << endl;
//double tCondition1, tCondition2, tCondition3;
  #if _VERBOSE
  cout << "unaryOdometry between " << plane1.id << " " << plane2.id << "\n";
  #endif

  //++nCheckConditions;

    // Constraints for odometry
  if( fabs(plane1.d - plane2.d) > configLocaliser.dist_d){
    #if _VERBOSE
    cout << "depthUnaryConstraint false " << fabs(plane1.d - plane2.d) << " > " << configLocaliser.dist_d << "\n";
    #endif
    return false;
  }

  if( plane1.v3normal .dot (plane2.v3normal) < configLocaliser.angle){
    #if _VERBOSE
    cout << "angleUnaryConstraint false " << plane1.v3normal .dot (plane2.v3normal) << " < " << configLocaliser.angle << "\n";
    #endif
    return false;
  }

//  ++totalUnary;
//  // Semantic label condition
//  if( !plane1.label.empty() && !plane2.label.empty() ){
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

//  if( !plane1.label.empty() || !plane2.label.empty() ){
//    if( plane1.label.empty() || plane2.label.empty() )
//      return false;
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

  // Main color
//  if(plane1.bDominantColor)
  {
//cout << "thresholds " << configLocaliser.color_threshold << " " << configLocaliser.intensity_threshold << endl;
    //++nCheckConditions;
//  if(configLocaliser.color_threshold > 0)
//    tCondition1 = pcl::getTime();

//    if( fabs(plane1.v3colorNrgb[0] - plane2.v3colorNrgb[0]) > configLocaliser.color_threshold ||
//        fabs(plane1.v3colorNrgb[1] - plane2.v3colorNrgb[1]) > configLocaliser.color_threshold ||
////        fabs(plane1.v3colorNrgb[2] - plane2.v3colorNrgb[2]) > configLocaliser.color_threshold ||//)
//        fabs(plane1.dominantIntensity - plane2.dominantIntensity) > configLocaliser.intensity_threshold)
//    ;
//    tCondition2 = pcl::getTime();
//
    if( BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) > configLocaliser.hue_threshold ){
    #if _VERBOSE
    cout << "Hist_H false_ " << BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) << " > " << configLocaliser.hue_threshold << "\n";
    #endif
//    ;
//    tCondition3 = pcl::getTime();

//  time1 += tCondition2 - tCondition1;
//  time2 += tCondition3 - tCondition2;
      return false;
    }
  }

  //++nCheckConditions;
  if(plane1.bFromStructure && plane2.bFromStructure){//cout << "True: both structure\n";
    return true;}

  double rel_areas = plane1.areaHull/ plane2.areaHull;
//  double rel_areas = plane1.areaVoxels/ plane2.areaVoxels;
//if(plane1.id==6 && plane2.id==8)
//cout << "rel_areas " << rel_areas << endl;
  double rel_elong = plane1.elongation / plane2.elongation;
//  if(plane1.id==6 && plane2.id==8)
//cout << "rel_elong " << rel_elong << endl;
  #if _VERBOSE
    cout << "elong " << plane1.elongation << " " << plane2.elongation << endl;
  #endif

  // If the plane has been fully detected use a narrower threshold for the comparison
  if(plane1.bFullExtent && plane2.bFullExtent)
  {
    if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
    #if _VERBOSE
    cout << "False: rel_areas full " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "False: rel_elong full " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }
  else
  {
    if(plane1.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_threshold ){
      #if _VERBOSE
      cout << "rel_areas RefFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
      #if _VERBOSE
      cout << "rel_areas CheckFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
    #if _VERBOSE
    cout << "rel_areas simple " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "rel_elong simple " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }

  // TODO. Use the inferred semantic information to control the search
  #if _VERBOSE
  cout << "UNARY TRUE" << endl;
  #endif

  return true;
}

bool SubgraphMatcher::evalUnaryConstraints2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure)
{
//Eigen::Vector3f color_dif = plane1.v3colorNrgb - plane2.v3colorNrgb;
//if(plane1.id==6 && plane2.id==8)
//cout << "color_dif \n" << color_dif << endl;
//double tCondition1, tCondition2, tCondition3;
  #if _VERBOSE
  cout << "unary2D between " << plane1.id << " " << plane2.id << "\n";
  #endif

  if( fabs(plane1.v3normal(0) - plane2.v3normal(0) ) > 0.08 ){
    return false;
  }

  if( plane1.v3normal(0) > 0.98 ){
    if( fabs(plane1.d - plane2.d) < 0.15 ){
//      ++rejectSemantic;
      return false;
    }
  }

  //++nCheckConditions;
//  ++totalUnary;
//  // Semantic label condition
//  if( !plane1.label.empty() && !plane2.label.empty() ){
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

//  if( !plane1.label.empty() || !plane2.label.empty() ){
//    if( plane1.label.empty() || plane2.label.empty() )
//      return false;
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

  // Main color
//  if(plane1.bDominantColor)
  {
//cout << "thresholds " << configLocaliser.color_threshold << " " << configLocaliser.intensity_threshold << endl;
    //++nCheckConditions;
//  if(configLocaliser.color_threshold > 0)
//    tCondition1 = pcl::getTime();

//    if( fabs(plane1.v3colorNrgb[0] - plane2.v3colorNrgb[0]) > configLocaliser.color_threshold ||
//        fabs(plane1.v3colorNrgb[1] - plane2.v3colorNrgb[1]) > configLocaliser.color_threshold ||
////        fabs(plane1.v3colorNrgb[2] - plane2.v3colorNrgb[2]) > configLocaliser.color_threshold ||//)
//        fabs(plane1.dominantIntensity - plane2.dominantIntensity) > configLocaliser.intensity_threshold)
//    ;
//    tCondition2 = pcl::getTime();
//
    if( BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) > configLocaliser.hue_threshold ){
    #if _VERBOSE
    cout << "Hist_H false " << BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) << " > " << configLocaliser.hue_threshold << "\n";
    #endif
//    ;
//    tCondition3 = pcl::getTime();

//  time1 += tCondition2 - tCondition1;
//  time2 += tCondition3 - tCondition2;
      return false;}
  }

  //++nCheckConditions;
  if(plane1.bFromStructure && plane2.bFromStructure){//cout << "True: both structure\n";
    return true;}

  double rel_areas = plane1.areaHull/ plane2.areaHull;
//  double rel_areas = plane1.areaVoxels/ plane2.areaVoxels;
//if(plane1.id==6 && plane2.id==8)
//cout << "rel_areas " << rel_areas << endl;
  double rel_elong = plane1.elongation / plane2.elongation;
//  if(plane1.id==6 && plane2.id==8)
//cout << "rel_elong " << rel_elong << endl;

  // If the plane has been fully detected use a narrower threshold for the comparison
  if(plane1.bFullExtent && plane2.bFullExtent)
  {
    if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
    #if _VERBOSE
    cout << "False: rel_areas full " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "False: rel_elong full " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }
  else
  {
    if(plane1.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_threshold ){
      #if _VERBOSE
      cout << "rel_areas RefFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
      #if _VERBOSE
      cout << "rel_areas CheckFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
    #if _VERBOSE
    cout << "rel_areas simple " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "rel_elong simple " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }

  // TODO. Use the inferred semantic information to control the search
  #if _VERBOSE
  cout << "UNARY TRUE" << endl;
  #endif

  return true;
}

bool SubgraphMatcher::evalUnaryConstraintsOdometry2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure)
{
//Eigen::Vector3f color_dif = plane1.v3colorNrgb - plane2.v3colorNrgb;
//if(plane1.id==6 && plane2.id==8)
//cout << "color_dif \n" << color_dif << endl;
//double tCondition1, tCondition2, tCondition3;
  #if _VERBOSE
  cout << "unaryOdometry2D between " << plane1.id << " " << plane2.id << "\n";
  #endif
  //++nCheckConditions;

  if( fabs(plane1.v3normal(0) - plane2.v3normal(0) ) > 0.05 ){
    return false;
  }

  if( plane1.v3normal(0) > 0.98 ){
    if( fabs(plane1.d - plane2.d) < 0.12 ){
      return false;
    }
  }

  #if _VERBOSE
  cout << "pass 2D \n";
  #endif

    // Constraints for odometry
  if( fabs(plane1.d - plane2.d) > configLocaliser.dist_d){
    #if _VERBOSE
    cout << "depthUnaryConstraint false " << fabs(plane1.d - plane2.d) << " > " << configLocaliser.dist_d << "\n";
    #endif
    return false;
  }

  if( plane1.v3normal .dot (plane2.v3normal) < configLocaliser.angle){
    #if _VERBOSE
    cout << "angleUnaryConstraint false " << plane1.v3normal .dot (plane2.v3normal) << " < " << configLocaliser.angle << "\n";
    #endif
    return false;
  }

//  ++totalUnary;
//  // Semantic label condition
//  if( !plane1.label.empty() && !plane2.label.empty() ){
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

//  if( !plane1.label.empty() || !plane2.label.empty() ){
//    if( plane1.label.empty() || plane2.label.empty() )
//      return false;
////    ++semanticPair;
//    if( plane1.label != plane2.label){
////      ++rejectSemantic;
//      return false;
//    }
//  }

  // Main color
//  if(plane1.bDominantColor)
  {
//cout << "thresholds " << configLocaliser.color_threshold << " " << configLocaliser.intensity_threshold << endl;
    //++nCheckConditions;
//  if(configLocaliser.color_threshold > 0)
//    tCondition1 = pcl::getTime();

//    if( fabs(plane1.v3colorNrgb[0] - plane2.v3colorNrgb[0]) > configLocaliser.color_threshold ||
//        fabs(plane1.v3colorNrgb[1] - plane2.v3colorNrgb[1]) > configLocaliser.color_threshold ||
////        fabs(plane1.v3colorNrgb[2] - plane2.v3colorNrgb[2]) > configLocaliser.color_threshold ||//)
//        fabs(plane1.dominantIntensity - plane2.dominantIntensity) > configLocaliser.intensity_threshold)
//    ;
//    tCondition2 = pcl::getTime();
//
    if( BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) > configLocaliser.hue_threshold ){
    #if _VERBOSE
    cout << "Hist_H false_ " << BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) << " > " << configLocaliser.hue_threshold << "\n";
    #endif
//    ;
//    tCondition3 = pcl::getTime();

//  time1 += tCondition2 - tCondition1;
//  time2 += tCondition3 - tCondition2;
      return false;
    }
  }

  //++nCheckConditions;
  if(plane1.bFromStructure && plane2.bFromStructure){//cout << "True: both structure\n";
    return true;}

  double rel_areas = plane1.areaHull/ plane2.areaHull;
//  double rel_areas = plane1.areaVoxels/ plane2.areaVoxels;
//if(plane1.id==6 && plane2.id==8)
//cout << "rel_areas " << rel_areas << endl;
  double rel_elong = plane1.elongation / plane2.elongation;
//  if(plane1.id==6 && plane2.id==8)
//cout << "rel_elong " << rel_elong << endl;
  #if _VERBOSE
    cout << "elong " << plane1.elongation << " " << plane2.elongation << endl;
  #endif

  // If the plane has been fully detected use a narrower threshold for the comparison
  if(plane1.bFullExtent && plane2.bFullExtent)
  {
    if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
    #if _VERBOSE
    cout << "False: rel_areas full " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "False: rel_elong full " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }
  else
  {
    if(plane1.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_full_threshold_inv || rel_areas > configLocaliser.area_threshold ){
      #if _VERBOSE
      cout << "rel_areas RefFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
      #if _VERBOSE
      cout << "rel_areas CheckFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
    #if _VERBOSE
    cout << "rel_areas simple " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "rel_elong simple " << rel_elong << endl;
    #endif
      return false;}
  //++nCheckConditions;
  }

  // TODO. Use the inferred semantic information to control the search
  #if _VERBOSE
  cout << "UNARY TRUE" << endl;
  #endif

  return true;
}

/**!
 * Compares the relation between Ref-neigRef with the relation between Check-neigCheck. Returns true if both geometries are similar
*/
bool SubgraphMatcher::evalBinaryConstraints(Plane &Ref, Plane &neigRef, Plane &Check, Plane &neigCheck)
{
  // Check intensity relations
//  float relIntensity = (neigRef.dominantIntensity/Ref.dominantIntensity) / (neigCheck.dominantIntensity/Check.dominantIntensity);
//  cout << "Binary intensity relations " << dif_height << endl;
//  if( relIntensity > 0.5 && relIntensity < 2)
//  {
//    return false;
//  }

  // Check height
  if(neigRef.areaHull < 2 && neigCheck.areaHull < 2)
  {
    double dif_height = fabs(Ref.v3normal.dot(neigRef.v3center - Ref.v3center) - Check.v3normal.dot(neigCheck.v3center - Check.v3center));
//  if(Ref.id==6 && Check.id==8)
//  cout << "dif_height " << dif_height << endl;
    float threshold_dist_dependent = configLocaliser.height_threshold * (Ref.v3center - neigRef.v3center).norm() * 0.4;
    float height_threshold = std::max(configLocaliser.height_threshold, threshold_dist_dependent);
    if(dif_height > height_threshold){
//    if(dif_height > configLocaliser.height_threshold){
    #if _VERBOSE
    cout << "Binary false: Ref dif_height " << dif_height << endl;
    #endif
      return false;}
    //++nCheckConditions;
  }
  else
  {
    double dif_height = fabs(Ref.v3normal.dot(neigRef.v3center - Ref.v3center) - Check.v3normal.dot(neigCheck.v3center - Check.v3center));
//  if(Ref.id==6 && Check.id==8)
//  cout << "dif_height " << dif_height << endl;
//    float threshold_dist_dependent = configLocaliser.height_threshold * (Ref.v3center - neigRef.v3center).norm() * 0.4;
    float height_threshold = (neigRef.areaHull + neigCheck.areaHull)*configLocaliser.height_threshold;
    if(dif_height > height_threshold){
//    if(dif_height > configLocaliser.height_threshold){
    #if _VERBOSE
    cout << "Binary false BIG Area: Ref dif_height " << dif_height << endl;
    #endif
      return false;}
    //++nCheckConditions;
  }

  if(Ref.areaHull < 2 && Check.areaHull < 2)
  {
    double dif_height2 = fabs(neigRef.v3normal.dot(Ref.v3center - neigRef.v3center) - neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
//  if(Ref.id==6 && Check.id==8)
//  cout << "dif_height2 " << dif_height2 << endl;
    float threshold_dist_dependent = configLocaliser.height_threshold * (Ref.v3center - neigRef.v3center).norm() * 0.4;
    float height_threshold = std::max(configLocaliser.height_threshold, threshold_dist_dependent);
    if(dif_height2 > height_threshold){
//    if(dif_height2 > configLocaliser.height_threshold){
    #if _VERBOSE
    cout << "Binary false: neigRef dif_height2 " << dif_height2 << endl;
    #endif
      return false;}
    //++nCheckConditions;
  }
  else
  {
    double dif_height2 = fabs(neigRef.v3normal.dot(Ref.v3center - neigRef.v3center) - neigCheck.v3normal.dot(Check.v3center - neigCheck.v3center));
//  if(Ref.id==6 && Check.id==8)
//  cout << "dif_height " << dif_height << endl;
//    float threshold_dist_dependent = configLocaliser.height_threshold * (Ref.v3center - neigRef.v3center).norm() * 0.4;
    float height_threshold = (Ref.areaHull + Check.areaHull)*configLocaliser.height_threshold;
    if(dif_height2 > height_threshold){
//    if(dif_height > configLocaliser.height_threshold){
    #if _VERBOSE
    cout << "Binary false: neigRef dif_height2 " << dif_height2 << endl;
    #endif
      return false;}
    //++nCheckConditions;
  }

  // Normal
  double dif_normal = fabs(RAD2DEG( acos( Ref.v3normal.dot(neigRef.v3normal)) - acos( Check.v3normal.dot(neigCheck.v3normal)) ) );
//  if( dif_normal > configLocaliser.angle_threshold ){
  if( dif_normal > std::max(configLocaliser.angle_threshold,2*(Ref.v3center - neigRef.v3center).norm()) ){
    #if _VERBOSE
      cout << "Binary false:  angle " << dif_normal << " with " << neigRef.id << endl;
    #endif
    return false;}
  //++nCheckConditions;

  // Relative distance
  double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );

  // If the plane has been fully detected use a narrower threshold for the comparison
  bool RefBothFull = (Ref.bFullExtent && neigRef.bFullExtent);// ? true : false;
  bool CheckBothFull = (Check.bFullExtent && neigCheck.bFullExtent);// ? true : false;

  if(configLocaliser.use_completeness)
  {
    if(RefBothFull && CheckBothFull)
    {
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
      #if _VERBOSE
        cout << "Binary false:  dist_centers1 with " << neigRef.id << endl;
      #endif
        return false;}
  //++nCheckConditions;
    }
    else if(RefBothFull || CheckBothFull)
    {
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
      #if _VERBOSE
        cout << "Binary false:  dist_centers2 with " << neigRef.id << endl;
      #endif
        return false;}
  //++nCheckConditions;
    }
    else if( !Ref.bFromStructure && !Check.bFromStructure && !neigRef.bFromStructure && !neigCheck.bFromStructure )
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
      #if _VERBOSE
      cout << "Binary false:  dist_centers3 with " << neigRef.id << endl;
      #endif
        return false;}
  //++nCheckConditions;
  }
  else
  {
    if( Ref.areaVoxels< 1 && neigCheck.areaVoxels< 1 && Check.areaVoxels< 1 && neigRef.areaVoxels< 1 ){ // Use the restriction only when all the planes involved are smaller than 1m2
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold){
      #if _VERBOSE
      cout << "Binary false:  dist_centers4 with " << neigRef.id << endl;
      #endif
        return false;}
      //++nCheckConditions;
    }
  }

  // We do not check ppal direction constraint -> It has demonstrated to be very little distinctive
  #if _VERBOSE
  cout << "BINARY TRUE\n" << endl;
  #endif

  return true;
}

bool isSubgraphContained(map<unsigned, unsigned> &contained, map<unsigned, unsigned> &container)
{
  if( contained.size() > container.size() )
    return false;

  //is Subgraph Contained?
  for(map<unsigned, unsigned>::iterator it = contained.begin(); it != contained.end(); it++)
    if(container.count(it->first) == 0)
      return false;
    else if(container[it->first] != it->second)
      return false;

  #if _VERBOSE
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
  #if _VERBOSE
    cout << "matched: " << matched.size() << "\n";
    for(map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
      cout << it->first << " - " << it->second << " =" << subgraphTrg->pPBM->vPlanes[it->second].label << endl;
    cout << "sourcePlanes: " << sourcePlanes.size() << ": ";
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      cout << *it1 << " ";
    cout << "\ntargetPlanes " << targetPlanes.size() << ": ";
    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
      cout << *it2 << " ";
//      cout << subgraphTrg->pPBM->vPlanes[*it2].label << " ";
    cout << endl;
  #endif

//  // Stop the search when we find a match of maximum size
//  if( winnerMatch.size() == subgraphSrc->subgraphPlanesIdx.size() || winnerMatch.size() == subgraphTrg->subgraphPlanesIdx.size() )
//    return;

  unsigned requiredMatches = max(configLocaliser.min_planes_recognition, static_cast<unsigned>(winnerMatch.size()));
//  if( sourcePlanes.empty() ||
//      targetPlanes.empty() ||
//    if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
////     static_cast<int>(sourcePlanes.size() ) < requiredMatches ||
////     static_cast<int>(targetPlanes.size() ) < requiredMatches ) // New condition to speed up the search when there are not a minimum number of candidates
//  {
////  cout << "End branch recursive search. matched " << matched.size() << " prev winner " << winnerMatch.size() << endl;
////    if(matched.size() > winnerMatch.size())
////      winnerMatch = matched;
//    #if _VERBOSE
//      cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << winnerMatch.size() << endl;
//    #endif
//    return;
//  }

  while(!sourcePlanes.empty())
  {
    set<unsigned>::iterator it1 = sourcePlanes.begin();
    #if _VERBOSE
      cout << "Compare " << *it1 << " Compare Compare Compare Compare Compare " << endl;
    #endif

    if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
    {
      #if _VERBOSE
        cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << winnerMatch.size() << endl;
      #endif
      return;
    }

//    if( (calcAreaMatched(matched) + calcAreaUnmatched(sourcePlanes)) < areaWinnerMatch )
//    {
//      #if _VERBOSE
//        cout << "End branch recursive search. Small Area " << matched.size() << " prev winner " << winnerMatch.size() << endl;
//      #endif
//      return;
//    }

    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
    {
      #if _VERBOSE
        cout << " " << *it1 << " with " << *it2 << endl;
      #endif
//      bool alreadyEval = false;
//      for(unsigned i=0; i<alreadyExplored.size(); i++)
//      {
//        map<unsigned, unsigned> checkMatch = matched;
//        checkMatch[*it1] = *it2;
//        if( matched.size() + min(sourcePlanes.size(),targetPlanes.size()) < alreadyExplored[i].size())
//        {
//          if( isSubgraphContained(checkMatch, alreadyExplored[i]) )
//          {
////          cout << "Combination already evaluated\n";
//            alreadyEval = true;
//            break;
//          }
//        }
//
//      }
//      if(alreadyEval)
//        continue;

      // Check that it1 and it2 correspond to the same plane
//      if(hashUnaryConstraints[*it1][*it2] == -1)
//        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//      if( !evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
      if( hashUnaryConstraints[*it1][*it2] != 1 )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
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
  } // End while

  if(matched.size() > winnerMatch.size()){
  float areaMatched = calcAreaMatched(matched);
//  if(areaMatched > areaWinnerMatch){
    #if _VERBOSE
      cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << winnerMatch.size() << " A " << areaWinnerMatch << endl;
    #endif
    areaWinnerMatch = areaMatched;
    winnerMatch = matched;}
}

void SubgraphMatcher::exploreSubgraphTreeR_Area(set<unsigned> &sourcePlanes, set<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
{
  #if _VERBOSE
    cout << "matched: " << matched.size() << "\n";
    for(map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
      cout << it->first << " - " << it->second << " =" << subgraphTrg->pPBM->vPlanes[it->second].label << endl;
    cout << "sourcePlanes: " << sourcePlanes.size() << ": ";
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      cout << *it1 << " ";
    cout << "\ntargetPlanes " << targetPlanes.size() << ": ";
    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
      cout << *it2 << " ";
//      cout << subgraphTrg->pPBM->vPlanes[*it2].label << " ";
    cout << endl;
  #endif

//  // Stop the search when we find a match of maximum size
//  if( winnerMatch.size() == subgraphSrc->subgraphPlanesIdx.size() || winnerMatch.size() == subgraphTrg->subgraphPlanesIdx.size() )
//    return;

  float matchedArea = calcAreaMatched(matched);

  float unmatchedArea = 0; // Pass as value parameter
  for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
    unmatchedArea += subgraphSrc->pPBM->vPlanes[*it1].areaHull;

//  unsigned requiredMatches = max(configLocaliser.min_planes_recognition, static_cast<unsigned>(winnerMatch.size()));

  while(!sourcePlanes.empty())
  {
    set<unsigned>::iterator it1 = sourcePlanes.begin();
    #if _VERBOSE
      cout << "Compare " << *it1 << " Compare Compare Compare Compare Compare " << endl;
    #endif

    if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= configLocaliser.min_planes_recognition )
      if( (matchedArea + unmatchedArea) < areaWinnerMatch )
      {
        #if _VERBOSE
          cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << winnerMatch.size() << endl;
        #endif
        return;
      }

//    if( (calcAreaMatched(matched) + calcAreaUnmatched(sourcePlanes)) < areaWinnerMatch )
//    {
//      #if _VERBOSE
//        cout << "End branch recursive search. Small Area " << matched.size() << " prev winner " << winnerMatch.size() << endl;
//      #endif
//      return;
//    }

    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
    {
      #if _VERBOSE
        cout << " " << *it1 << " with " << *it2 << endl;
      #endif
//      bool alreadyEval = false;
//      for(unsigned i=0; i<alreadyExplored.size(); i++)
//      {
//        map<unsigned, unsigned> checkMatch = matched;
//        checkMatch[*it1] = *it2;
//        if( matched.size() + min(sourcePlanes.size(),targetPlanes.size()) < alreadyExplored[i].size())
//        {
//          if( isSubgraphContained(checkMatch, alreadyExplored[i]) )
//          {
////          cout << "Combination already evaluated\n";
//            alreadyEval = true;
//            break;
//          }
//        }
//
//      }
//      if(alreadyEval)
//        continue;

      // Check that it1 and it2 correspond to the same plane
//      if(hashUnaryConstraints[*it1][*it2] == -1)
//        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//      if( !evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
      if( hashUnaryConstraints[*it1][*it2] != 1 )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
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

      exploreSubgraphTreeR_Area(nextSrcPlanes, nextTrgPlanes, nextMatched);

//      // CHANGE to be As in the algorithm of our article
//      if(matched.size() > bestCombination.size())
//        return bestCombination;

    }
    sourcePlanes.erase(it1);
  } // End while

//  if(matched.size() > winnerMatch.size()){
  float areaMatched = calcAreaMatched(matched);
  if(areaMatched > areaWinnerMatch){
    #if _VERBOSE
      cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << winnerMatch.size() << " A " << areaWinnerMatch << endl;
    #endif
    areaWinnerMatch = areaMatched;
    winnerMatch = matched;}
}

float SubgraphMatcher::calcAreaMatched(std::map<unsigned,unsigned> &matched_planes)
{
  float areaMatched = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    areaMatched += subgraphSrc->pPBM->vPlanes[it->first].areaHull;

  return areaMatched;
}

float SubgraphMatcher::calcAreaUnmatched(std::set<unsigned> &unmatched_planes)
{
  float areaUnatched = 0;
  for(set<unsigned>::iterator it = unmatched_planes.begin(); it != unmatched_planes.end(); it++)
    areaUnatched += subgraphSrc->pPBM->vPlanes[*it].areaHull;

  return areaUnatched;
}

//std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget)
//{
////cout << "SubgraphMatcher::compareSubgraphs... \n";
//  subgraphSrc = &subgraphSource;
//  subgraphTrg = &subgraphTarget;
//  map<unsigned, unsigned> matched;
//
//  areaWinnerMatch = 0;
//  winnerMatch.clear();
//  alreadyExplored.clear(); // MODIFICAR: Tener en cuenta caminos explorados cuando exploramos grafos vecinos
//  std::set<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
//  std::set<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;
//
//  hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->pPBM->vPlanes.size(), std::vector<int8_t>(subgraphTrg->pPBM->vPlanes.size()) );
//  for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
//    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//      hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//
//  exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);
////  exploreSubgraphTreeR_Area(sourcePlanes, targetPlanes, matched);
//#if _VERBOSE
//  cout << "Area winnerMatch " << areaWinnerMatch << endl;
//#endif
//
//  return winnerMatch;
//}

std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, const int option)
{
//cout << "SubgraphMatcher::compareSubgraphs... \n";
  subgraphSrc = &subgraphSource;
  subgraphTrg = &subgraphTarget;
  map<unsigned, unsigned> matched;

  areaWinnerMatch = 0;
  winnerMatch.clear();
  alreadyExplored.clear(); // MODIFICAR: Tener en cuenta caminos explorados cuando exploramos grafos vecinos
  std::set<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
  std::set<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;

#if _VERBOSE
  cout << "Source planes: ";
  for(set<unsigned>::iterator it2 = sourcePlanes.begin(); it2 != sourcePlanes.end(); it2++)
    cout << " " << *it2;
  cout << endl;
  cout << "Target planes: ";
  for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
    cout << " " << *it2;
  cout << endl;
#endif

  // Fill Hash table of unary constraints
  hashUnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->pPBM->vPlanes.size(), std::vector<int8_t>(subgraphTrg->pPBM->vPlanes.size()) );
  if(option == 0) // Default subgraph matcher
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 1) // Odometry graph matcher
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsOdometry(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 2) // Default graph matcher restricted to planar movement (fix plane x=const)
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraints2D(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 3) // Odometry graph matcher restricted to planar movement (fix plane x=const)
    for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        hashUnaryConstraints[*it1][*it2] = (evalUnaryConstraintsOdometry2D(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);

  exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);
//  exploreSubgraphTreeR_Area(sourcePlanes, targetPlanes, matched);
#if _VERBOSE
  cout << "Area winnerMatch " << areaWinnerMatch << endl;
#endif

  return winnerMatch;
}
