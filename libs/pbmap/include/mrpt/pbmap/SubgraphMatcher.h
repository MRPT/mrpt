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

#ifndef __SUBGRAPHMATCHER_H
#define __SUBGRAPHMATCHER_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>
#include <mrpt/pbmap/link_pragmas.h>

#include <mrpt/pbmap/heuristicParams.h>
#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/Subgraph.h>

namespace mrpt {
namespace pbmap {


  /*! This class finds the best correspondence between the planes of two subgraphs (i.e. sets of neighbor planes).
   * It relies on an interpretation tree employing geometric restrictions that are represented as a set of unary and binary constraints.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class SubgraphMatcher
  {
    private:

    /*! LUT table for unary constraints.*/
    std::vector<std::vector<int8_t> > LUT_UnaryConstraints;

    float calcAreaUnmatched(std::vector<unsigned> &unmatched_planes);

    /*! Compute the total informatin of the scene following the RAS paper.*/
    void computeSceneInformation();

    /*! Compute the scene information factors (the weights: the information each plane provides to the total scene description) following the RAS paper.*/
    void computeSceneInformationWeights();

    //*! Arrange the subgraph of planes according to the given arrangement (descending order of weights).*/
    //std::set<unsigned> rearrangeSubgraph(const std::set<unsigned> &subgraph_planes, const std::vector<size_t> & arrangement);

    /*! Compute the score of a set of unmatched planes in the source subgraph.*/
    float computeScore (std::vector<unsigned> & unmatched);

    /*! Compute the score of the set of matched planes in the source subgraph.*/
    float computeScore (std::map<unsigned,unsigned> & matched);

   public:

    SubgraphMatcher();

    /*! Check if the two input planes fulfill a set of geometric constraints, and so, if they are candidates to be the same plane.*/
    bool evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraints2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraintsOdometry(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraintsOdometry2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);

    /*! Check if the two pair of planes plane1-plane2 ans planeA-planeB fulfill the same geometric relationship, and so,
    if they are candidates to be the same planes.*/
    bool evalBinaryConstraints(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);
    bool evalBinaryConstraintsOdometry(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);

    /*! List of combinations that have been explored in the interpretation tree.*/  // Cambiar nombre
    std::vector<std::map<unsigned,unsigned> > alreadyExplored;

    /*! Find the best combination of planes correspondences given two subgraphs represeting local neighborhoods of planes.*/  // Cambiar nombre o Quitar!
    void exploreSubgraphTreeR(std::vector<unsigned> &evalRef, std::vector<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);
    void exploreSubgraphTreeR_Area(std::vector<unsigned> &evalRef, std::vector<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);

    /*! Set source (current) subgraph.*/
    void inline setSourceSubgraph(Subgraph &subgSrc){subgraphSrc = &subgSrc;}

    /*! Set target subgraph.*/
    void inline setTargetSubgraph(Subgraph &subgTrg){subgraphTrg = &subgTrg;}

    /*! Reference's scene subgraph to be matched.*/
    Subgraph *subgraphSrc;

    /*! Target's scene subgraph to be matched.*/
    Subgraph *subgraphTrg;

    int nCheckConditions;

    int totalUnary;
    int semanticPair;
    int rejectSemantic;

    /*! Return the total area of the matched planes in the frame source.*/
    float calcAreaMatched(std::map<unsigned,unsigned> &matched_planes);

    /*! Scene Information Factors (see RAS paper) */
    Eigen::Matrix3f scene_information_src;
    Eigen::Matrix3f scene_information_trg;
    //Eigen::VectorXf weights_src, weights_trg;
    std::vector<float> weights_src, weights_trg;
    std::vector<size_t> w_src_decreasing_idx, w_trg_decreasing_idx;
    float max_score_src;

    /*! Set of thresholds for PbMap matching.*/
    config_heuristics configLocaliser;

    /*! List of planes correspondences.*/
    std::map<unsigned, unsigned> best_match;
    float area_best_match;
    float score_best_match;

    /*! Get the best possible match between the reference and taget subgraphs according to the total information of the matched scene (scene's information factors).
     *  Returns a list with plane matches from subgraphSrc to subgraphTrg.*/
//    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget);
    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, const int option=0); // Options are ...

  };

} } // End of namespaces

#endif
#endif
