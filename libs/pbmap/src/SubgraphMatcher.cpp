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
#include <mrpt/system/os.h>

#define _VERBOSE 0

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
    cout << "FALSE  Hist_H " << BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) << " > " << configLocaliser.hue_threshold << "\n";
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
    cout << "FALSE: rel_areas full " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    // We can use a narrower limit for elong
    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "FALSE: rel_elong full " << rel_elong << endl;
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
      cout << "FALSE rel_areas RefFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if(plane2.bFullExtent)
    {
      if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_full_threshold ){
      #if _VERBOSE
      cout << "FALSE rel_areas CheckFull " << rel_areas << endl;
      #endif
        return false;}
    }
    else if( rel_areas < configLocaliser.area_threshold_inv || rel_areas > configLocaliser.area_threshold ){
    #if _VERBOSE
    cout << "FALSE rel_areas simple " << rel_areas << endl;
    #endif
      return false;}
  //++nCheckConditions;

    if( rel_elong < configLocaliser.elongation_threshold_inv || rel_elong > configLocaliser.elongation_threshold ){
    #if _VERBOSE
    cout << "FALSE rel_elong simple " << rel_elong << endl;
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
  if( dif_normal > std::max(configLocaliser.angle_threshold,2*(Ref.v3center - neigRef.v3center).norm()) )
  {
    #if _VERBOSE
      cout << "Binary false:  angle " << dif_normal << " with " << neigRef.id << endl;
    #endif
    return false;
  }
  //++nCheckConditions;

  // Relative distance
  double rel_dist_centers = sqrt( (Ref.v3center - neigRef.v3center).dot(Ref.v3center - neigRef.v3center) / ((Check.v3center - neigCheck.v3center).dot(Check.v3center - neigCheck.v3center)) );

  // If the plane has been fully detected use a narrower threshold for the comparison
  bool RefBothFull = (Ref.bFullExtent && neigRef.bFullExtent);// ? true : false;
  bool CheckBothFull = (Check.bFullExtent && neigCheck.bFullExtent);// ? true : false;

  if(configLocaliser.use_completeness)
  {
    if(RefBothFull && CheckBothFull){
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
      #if _VERBOSE
        cout << "Binary false:  dist_centers1 with " << neigRef.id << endl;
      #endif
        return false;
      }
  //++nCheckConditions;
    }
    else if(RefBothFull || CheckBothFull){
      if( rel_dist_centers < configLocaliser.dist_threshold_inv || rel_dist_centers > configLocaliser.dist_threshold ){
      #if _VERBOSE
        cout << "Binary false:  dist_centers2 with " << neigRef.id << endl;
      #endif
        return false;
      }
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
    if( Ref.areaHull< 1 && neigCheck.areaHull< 1 && Check.areaHull< 1 && neigRef.areaHull< 1 ){ // Use the restriction only when all the planes involved are smaller than 1m2
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
void SubgraphMatcher::exploreSubgraphTreeR(std::vector<unsigned> &sourcePlanes, std::vector<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
{
  #if _VERBOSE
    cout << "matched: " << matched.size() << "\n";
    for(map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
      cout << it->first << " - " << it->second << " =" << subgraphTrg->pPBM->vPlanes[it->second].label << endl;
    cout << "sourcePlanes: " << sourcePlanes.size() << ": ";
    for(std::vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      cout << *it1 << " ";
    cout << "\ntargetPlanes " << targetPlanes.size() << ": ";
    for(std::vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
      cout << *it2 << " ";
//      cout << subgraphTrg->pPBM->vPlanes[*it2].label << " ";
    cout << endl;
    cout << "computeScore(matched) " << computeScore(matched) << " computeScore(sourcePlanes) " << computeScore(sourcePlanes) << " score_best_match " << score_best_match << endl;
    mrpt::system::pause();
  #endif

//  // Stop the search when we find a match of maximum size
//  if( best_match.size() == subgraphSrc->subgraphPlanesIdx.size() || best_match.size() == subgraphTrg->subgraphPlanesIdx.size() )
//    return;

  //unsigned requiredMatches = configLocaliser.min_planes_recognition;
  //unsigned requiredMatches = max(configLocaliser.min_planes_recognition, static_cast<unsigned>(best_match.size()));
//  if( sourcePlanes.empty() ||
//      targetPlanes.empty() ||
//    if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
////     static_cast<int>(sourcePlanes.size() ) < requiredMatches ||
////     static_cast<int>(targetPlanes.size() ) < requiredMatches ) // New condition to speed up the search when there are not a minimum number of candidates
//  {
////  cout << "End branch recursive search. matched " << matched.size() << " prev winner " << best_match.size() << endl;
////    if(matched.size() > best_match.size())
////      best_match = matched;
//    #if _VERBOSE
//      cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << best_match.size() << endl;
//    #endif
//    return;
//  }


  // For every plane in sourcePlanes
  while(!sourcePlanes.empty())
  {
    std::vector<unsigned>::iterator it1 = sourcePlanes.begin();
//  for(std::vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
//  {
    #if _VERBOSE
      cout << "Compare " << *it1 << " with... " << endl;
      cout << "computeScore(matched) " << computeScore(matched) << " computeScore(sourcePlanes) " << computeScore(sourcePlanes) << " score_best_match " << score_best_match << endl;
    #endif

    //if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= requiredMatches )
    if( computeScore(matched) + computeScore(sourcePlanes) < score_best_match )
    {
      #if _VERBOSE
        cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << best_match.size() << endl;
      #endif
      return;
    }

//    if( (calcAreaMatched(matched) + calcAreaUnmatched(sourcePlanes)) < area_best_match )
//    {
//      #if _VERBOSE
//        cout << "End branch recursive search. Small Area " << matched.size() << " prev winner " << best_match.size() << endl;
//      #endif
//      return;
//    }

    for(std::vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
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
//      if(LUT_UnaryConstraints[*it1][*it2] == -1)
//        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//      if( !evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
      if( LUT_UnaryConstraints[*it1][*it2] != 1 ){//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
        #if _VERBOSE
          cout << "Fail Unary constraint " << endl;
        #endif
        continue;
      }

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
      map<unsigned, unsigned> nextMatched = matched;
      nextMatched[*it1] = *it2;
      std::vector<unsigned> nextSrcPlanes = sourcePlanes;
      for(std::vector<unsigned>::iterator it = nextSrcPlanes.begin(); it != nextSrcPlanes.end(); it++)
          if(*it == *it1)
          {
              nextSrcPlanes.erase(it);
              break;
          }
      std::vector<unsigned> nextTrgPlanes = targetPlanes;
      for(std::vector<unsigned>::iterator it = nextTrgPlanes.begin(); it != nextTrgPlanes.end(); it++)
          if(*it == *it2)
          {
              nextTrgPlanes.erase(it);
              break;
          }

      alreadyExplored.push_back(nextMatched);

      exploreSubgraphTreeR(nextSrcPlanes, nextTrgPlanes, nextMatched);

//      // CHANGE to be As in the algorithm of our article
//      if(matched.size() > bestCombination.size())
//        return bestCombination;

    }
    sourcePlanes.erase(it1);
  } // End while

  if( computeScore(matched) > score_best_match )
  {
    float areaMatched = calcAreaMatched(matched);
    score_best_match = computeScore(matched);
    area_best_match = areaMatched;
    best_match = matched;
    //  if(areaMatched > area_best_match){
    #if _VERBOSE
      cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << best_match.size() << " A " << area_best_match << " score_best_match " << score_best_match << endl;
    #endif
  }
//  if(matched.size() > best_match.size())
//  {
//    float areaMatched = calcAreaMatched(matched);
////  if(areaMatched > area_best_match){
//    #if _VERBOSE
//      cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << best_match.size() << " A " << area_best_match << endl;
//    #endif
//    area_best_match = areaMatched;
//    best_match = matched;
//  }

}

void SubgraphMatcher::exploreSubgraphTreeR_Area(vector<unsigned> &sourcePlanes, vector<unsigned> &targetPlanes, map<unsigned, unsigned> &matched)
{
  #if _VERBOSE
    cout << "matched: " << matched.size() << "\n";
    for(map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
      cout << it->first << " - " << it->second << " =" << subgraphTrg->pPBM->vPlanes[it->second].label << endl;
    cout << "sourcePlanes: " << sourcePlanes.size() << ": ";
    for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      cout << *it1 << " ";
    cout << "\ntargetPlanes " << targetPlanes.size() << ": ";
    for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
      cout << *it2 << " ";
//      cout << subgraphTrg->pPBM->vPlanes[*it2].label << " ";
    cout << endl;
  #endif

//  // Stop the search when we find a match of maximum size
//  if( best_match.size() == subgraphSrc->subgraphPlanesIdx.size() || best_match.size() == subgraphTrg->subgraphPlanesIdx.size() )
//    return;

  float matchedArea = calcAreaMatched(matched);

  float unmatchedArea = 0; // Pass as value parameter
  for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
    unmatchedArea += subgraphSrc->pPBM->vPlanes[*it1].areaHull;

//  unsigned requiredMatches = max(configLocaliser.min_planes_recognition, static_cast<unsigned>(best_match.size()));

  while(!sourcePlanes.empty())
  {
    vector<unsigned>::iterator it1 = sourcePlanes.begin();
    #if _VERBOSE
      cout << "Compare " << *it1 << " Compare Compare Compare Compare Compare " << endl;
    #endif

    if( (matched.size() + min(sourcePlanes.size(),targetPlanes.size())) <= configLocaliser.min_planes_recognition )
      if( (matchedArea + unmatchedArea) < area_best_match )
      {
        #if _VERBOSE
          cout << "End branch recursive search. Too short " << matched.size() << " prev winner " << best_match.size() << endl;
        #endif
        return;
      }

//    if( (calcAreaMatched(matched) + calcAreaUnmatched(sourcePlanes)) < area_best_match )
//    {
//      #if _VERBOSE
//        cout << "End branch recursive search. Small Area " << matched.size() << " prev winner " << best_match.size() << endl;
//      #endif
//      return;
//    }

    for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
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
//      if(LUT_UnaryConstraints[*it1][*it2] == -1)
//        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//      if( !evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
      if( LUT_UnaryConstraints[*it1][*it2] != 1 )//(FloorPlane != -1 && FloorPlaneMap != -1) ? true : false ) )
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
      map<unsigned, unsigned> nextMatched = matched;
      nextMatched[*it1] = *it2;
      vector<unsigned> nextSrcPlanes = sourcePlanes;
      nextSrcPlanes.erase(it1);
      vector<unsigned> nextTrgPlanes = targetPlanes;
      nextTrgPlanes.erase(it2);

      alreadyExplored.push_back(nextMatched);

      exploreSubgraphTreeR_Area(nextSrcPlanes, nextTrgPlanes, nextMatched);

//      // CHANGE to be As in the algorithm of our article
//      if(matched.size() > bestCombination.size())
//        return bestCombination;

    }
    sourcePlanes.erase(it1);
  } // End while

//  if(matched.size() > best_match.size()){
  float areaMatched = calcAreaMatched(matched);
  if(areaMatched > area_best_match){
    #if _VERBOSE
      cout << "End branch recursive search. matched " << matched.size() << " A " << areaMatched << " prev winner " << best_match.size() << " A " << area_best_match << endl;
    #endif
    area_best_match = areaMatched;
    best_match = matched;}
}

float SubgraphMatcher::calcAreaMatched(std::map<unsigned,unsigned> &matched_planes)
{
  float areaMatched = 0;
  for(map<unsigned, unsigned>::iterator it = matched_planes.begin(); it != matched_planes.end(); it++)
    areaMatched += subgraphSrc->pPBM->vPlanes[it->first].areaHull;

  return areaMatched;
}

float SubgraphMatcher::calcAreaUnmatched(std::vector<unsigned> &unmatched_planes)
{
  float areaUnatched = 0;
  for(vector<unsigned>::iterator it = unmatched_planes.begin(); it != unmatched_planes.end(); it++)
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
//  area_best_match = 0;
//  best_match.clear();
//  alreadyExplored.clear(); // MODIFICAR: Tener en cuenta caminos explorados cuando exploramos grafos vecinos
//  std::set<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
//  std::set<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;
//
//  LUT_UnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->pPBM->vPlanes.size(), std::vector<int8_t>(subgraphTrg->pPBM->vPlanes.size()) );
//  for(set<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
//    for(set<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
//      LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
//
//  exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);
////  exploreSubgraphTreeR_Area(sourcePlanes, targetPlanes, matched);
//#if _VERBOSE
//  cout << "Area best_match " << area_best_match << endl;
//#endif
//
//  return best_match;
//}

/*! Compute the total informatin of the scene following the RAS paper.*/
void SubgraphMatcher::computeSceneInformation()
{
    //assert( !subgraphSrc->empty() );
    scene_information_src = Eigen::Matrix3f::Zero();
    for(vector<unsigned>::iterator it = subgraphSrc->subgraphPlanesIdx.begin(); it != subgraphSrc->subgraphPlanesIdx.end(); it++)
    {
        float info_plane = sqrt(subgraphSrc->pPBM->vPlanes[*it].info_3rd_eigenval * subgraphSrc->pPBM->vPlanes[*it].areaHull);
        Eigen::Vector3f & vNormal = subgraphSrc->pPBM->vPlanes[*it].v3normal;
        scene_information_src += info_plane * vNormal * vNormal.transpose();
    }

    scene_information_trg = Eigen::Matrix3f::Zero();
    for(vector<unsigned>::iterator it = subgraphTrg->subgraphPlanesIdx.begin(); it != subgraphTrg->subgraphPlanesIdx.end(); it++)
    {
        float info_plane = sqrt(subgraphTrg->pPBM->vPlanes[*it].info_3rd_eigenval * subgraphTrg->pPBM->vPlanes[*it].areaHull);
        Eigen::Vector3f & vNormal = subgraphTrg->pPBM->vPlanes[*it].v3normal;
        scene_information_trg += info_plane * vNormal * vNormal.transpose();
    }
}

/*! Sort a vector and retrieve the indexes of teh sorted values.*/
std::vector<size_t> sort_indexes_(const std::vector<float> & v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;

  // sort indexes based on comparing values in v
  std::sort( idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] > v[i2];} );

  return idx;
}


/*! Compute the scene information factors (the weights for each plane) following the RAS paper.*/
void SubgraphMatcher::computeSceneInformationWeights()
{
    computeSceneInformation();

    int it_count = 0;
    max_score_src = 0.f;
    //weights_src = Eigen::VectorXf::Ones( subgraphSrc->subgraphPlanesIdx.size() );
    weights_src.resize( subgraphSrc->subgraphPlanesIdx.size() );
    for(vector<unsigned>::iterator it = subgraphSrc->subgraphPlanesIdx.begin(); it != subgraphSrc->subgraphPlanesIdx.end(); it++)
    {
        float info_plane = sqrt(subgraphSrc->pPBM->vPlanes[*it].info_3rd_eigenval * subgraphSrc->pPBM->vPlanes[*it].areaHull);
        Eigen::Vector3f & vNormal = subgraphSrc->pPBM->vPlanes[*it].v3normal;
        weights_src[it_count] = info_plane / (vNormal.transpose() * scene_information_src * vNormal);
        max_score_src +=  weights_src[it_count];
//        cout << it_count << " weights_src " << weights_src[it_count] << " " ;
//        cout << " vNormal " << vNormal.transpose() << " inliers " << subgraphSrc->pPBM->vPlanes[*it].planePointCloudPtr->size() << " areaHull " << subgraphSrc->pPBM->vPlanes[*it].areaHull << " info_3rd_eigenval " << subgraphSrc->pPBM->vPlanes[*it].info_3rd_eigenval << " info " << info_plane << endl;
        ++it_count;
    }
    w_src_decreasing_idx = sort_indexes( weights_src );
    std::vector<float> weights_arranged_src( w_src_decreasing_idx.size() );
    std::vector<unsigned> subgraph_arranged_src( w_src_decreasing_idx.size() );
    for (size_t i = 0; i != w_src_decreasing_idx.size(); ++i)
    {
      subgraph_arranged_src[i] = subgraphSrc->subgraphPlanesIdx[w_src_decreasing_idx[i]];
      weights_arranged_src[i] = weights_src[w_src_decreasing_idx[i]];
    }
    weights_src = weights_arranged_src;
    subgraphSrc->subgraphPlanesIdx = subgraph_arranged_src;

//    //w_src_decreasing_idx = sort_vector( weights_src );
//    cout << "\n Arranged weights: ";
//    for(size_t i = 0; i < weights_src.size(); i++)
//        cout << weights_src[i] << " ";
//    cout << "\n sort_indexes: ";
//    for(size_t i = 0; i < w_src_decreasing_idx.size(); i++)
//        cout << w_src_decreasing_idx[i] << " ";
//    cout << "\n";
//    cout << "\n subgraphSrc->subgraphPlanesIdx: ";
//    for(size_t i = 0; i < subgraphSrc->subgraphPlanesIdx.size(); i++)
//        cout << subgraphSrc->subgraphPlanesIdx[i] << " ";
//    cout << "\n";

    it_count = 0;
    weights_trg.resize( subgraphTrg->subgraphPlanesIdx.size() );
    for(vector<unsigned>::iterator it = subgraphTrg->subgraphPlanesIdx.begin(); it != subgraphTrg->subgraphPlanesIdx.end(); it++)
    {
        float info_plane = sqrt(subgraphTrg->pPBM->vPlanes[*it].info_3rd_eigenval * subgraphTrg->pPBM->vPlanes[*it].areaHull);
        Eigen::Vector3f & vNormal = subgraphTrg->pPBM->vPlanes[*it].v3normal;
        weights_trg[it_count] = info_plane / (vNormal.transpose() * scene_information_trg * vNormal);
        ++it_count;
    }
    w_trg_decreasing_idx = sort_indexes( weights_trg );
    std::vector<float> weights_arranged_trg( w_trg_decreasing_idx.size() );
    std::vector<unsigned> subgraph_arranged_trg( w_trg_decreasing_idx.size() );
    for (size_t i = 0; i != w_trg_decreasing_idx.size(); ++i)
    {
      subgraph_arranged_trg[i] = subgraphTrg->subgraphPlanesIdx[w_trg_decreasing_idx[i]];
      weights_arranged_trg[i] = weights_trg[w_trg_decreasing_idx[i]];
    }
    weights_trg = weights_arranged_trg;
    subgraphTrg->subgraphPlanesIdx = subgraph_arranged_trg;
}

///*! Compute the scene information factors (the weights for each plane) following the RAS paper.*/
//std::set<unsigned> SubgraphMatcher::rearrangeSubgraph(const std::set<unsigned> &subgraph_planes, const std::vector<size_t> & arrangement)
//{
//#if _VERBOSE
//    cout << "SubgraphMatcher::rearrangeSubgraph... \n";
//#endif

//    assert( subgraph_planes.size() == arrangement.size() || arrangement.size() > 0 );
//    //assert( std::max_element(arrangement) < arrangement.size() );

//    std::set<unsigned> subgraph_new_arrangement;
//    std::vector<unsigned> old_arrangement( arrangement.size() );
//    unsigned i = 0;
//    cout << " size: " << arrangement.size() << endl;
//    for(std::set<unsigned>::iterator it = subgraph_planes.begin(); it != subgraph_planes.end(); it++){
//        old_arrangement[i++] = *it;
//        cout << *it << " ";}

//    cout << "\n old_arrangement: ";
//    for(i=0; i < old_arrangement.size(); i++)
//        cout << old_arrangement[i] << " ";
//    cout << "\n";

//    for(i=0; i < old_arrangement.size(); i++)
//        subgraph_new_arrangement.insert(old_arrangement[arrangement[i]]);

//    cout << "\n subgraph_new_arrangement: ";
//    for(std::set<unsigned>::iterator it = subgraph_new_arrangement.begin(); it != subgraph_new_arrangement.end(); it++)
//        cout << *it << " ";
//    cout << "\n";

//    return subgraph_new_arrangement;
//}

float SubgraphMatcher::computeScore (std::vector<unsigned> & unmatched)
{
    float score = 0.f;

    std::vector<unsigned>::iterator it = unmatched.begin();
    std::vector<unsigned>::iterator it2;
    std::vector<float>::iterator it2_w = weights_src.begin();
    for(it2 = subgraphSrc->subgraphPlanesIdx.begin(); it2 != subgraphSrc->subgraphPlanesIdx.end(); it2++, it2_w++)
        if(*it == *it2)
            break;
    for(; it2 != subgraphSrc->subgraphPlanesIdx.end(); it2++, it2_w++)
        score += *it2_w;

//    for(std::vector<unsigned>::iterator it = unmatched.begin(); it != unmatched.end(); it++)
//    {
//        std::vector<unsigned>::iterator it2, it2_w;
//        for(it2 = subgraphSrc->subgraphPlanesIdx.begin(), it2_w = weights_src.begin(); it2 != subgraphSrc->subgraphPlanesIdx.end(); it2++, it2_w++)
//            if(*it == *it2)
//                break;

//        size_t prev_index = std::distance (subgraphSrc->subgraphPlanesIdx.begin(), subgraphSrc->subgraphPlanesIdx.find(*it) );
//        size_t current_index;
//        for(current_index=0; current_index < unmatched.size(); current_index++)
//            if(w_src_decreasing_idx[current_index] == prev_index)
//                break;
//        score += weights_src[current_index];
//    }

    return score;
}

float SubgraphMatcher::computeScore (std::map<unsigned,unsigned> & matched)
{
    float score = 0.f;
    for(std::map<unsigned, unsigned>::iterator it = matched.begin(); it != matched.end(); it++)
    {
        std::vector<float>::iterator it2_w = weights_src.begin();
        for(std::vector<unsigned>::iterator it2 = subgraphSrc->subgraphPlanesIdx.begin(); it2 != subgraphSrc->subgraphPlanesIdx.end(); it2++, it2_w++)
            if(it->first == *it2)
                score += *it2_w;
    }

    return score;
}


/*! Get the best possible match between the reference and taget subgraphs according to the total information of the matched scene (scene's information factors).*/
std::map<unsigned,unsigned> SubgraphMatcher::compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, const int option)
{
#if _VERBOSE
    cout << "SubgraphMatcher::compareSubgraphs... \n";
#endif

  // Initialize search
  best_match.clear();
  score_best_match = 0.f;
  area_best_match = 0.f;

  subgraphSrc = &subgraphSource;
  subgraphTrg = &subgraphTarget;
  map<unsigned, unsigned> matched;
  alreadyExplored.clear(); // MODIFICAR: Tener en cuenta caminos explorados cuando exploramos grafos vecinos

  // Compute the amount of information each plane provides to the total scene description
  computeSceneInformationWeights();
  std::vector<unsigned> sourcePlanes = subgraphSrc->subgraphPlanesIdx;
  std::vector<unsigned> targetPlanes = subgraphTrg->subgraphPlanesIdx;

  // The matrices representing the information of the source and target scenes must have full rank (rank=3) in order to allow the registration of the scenes
  if( scene_information_src.rank() != 3 || scene_information_trg.rank() != 3 )
      return best_match;

#if _VERBOSE
  cout << "Source planes: ";
  for(vector<unsigned>::iterator it2 = sourcePlanes.begin(); it2 != sourcePlanes.end(); it2++)
    cout << " " << *it2;
  cout << endl;
  cout << "Target planes: ";
  for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
    cout << " " << *it2;
  cout << endl;
#endif

  // Fill Hash table of unary constraints
  LUT_UnaryConstraints = std::vector<std::vector<int8_t> >(subgraphSrc->pPBM->vPlanes.size(), std::vector<int8_t>(subgraphTrg->pPBM->vPlanes.size()) );
  if(option == 0) // Default subgraph matcher
    for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraints(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 1) // Odometry graph matcher
    for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraintsOdometry(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 2) // Default graph matcher restricted to planar movement (fix plane x=const)
    for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraints2D(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);
  else if(option == 3) // Odometry graph matcher restricted to planar movement (fix plane x=const)
    for(vector<unsigned>::iterator it1 = sourcePlanes.begin(); it1 != sourcePlanes.end(); it1++)
      for(vector<unsigned>::iterator it2 = targetPlanes.begin(); it2 != targetPlanes.end(); it2++)
        LUT_UnaryConstraints[*it1][*it2] = (evalUnaryConstraintsOdometry2D(subgraphSrc->pPBM->vPlanes[*it1], subgraphTrg->pPBM->vPlanes[*it2], *subgraphTrg->pPBM, false ) ? 1 : 0);

//#if _VERBOSE
  cout << "Max Score " << computeScore(sourcePlanes) << endl;
//#endif

  exploreSubgraphTreeR(sourcePlanes, targetPlanes, matched);
//  exploreSubgraphTreeR_Area(sourcePlanes, targetPlanes, matched);
#if _VERBOSE
  cout << "Area best_match " << area_best_match << endl;
  cout << "score_best_match " << score_best_match << endl;
#endif

  return best_match;
}
