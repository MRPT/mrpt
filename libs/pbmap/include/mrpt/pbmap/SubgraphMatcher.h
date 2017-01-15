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


  /*!This class finds the best correspondence between the planes of two subgraphs (i.e. sets of neighbor planes).
   * It relies on an interpretation tree employing geometric restrictions that are represented as a set of unary and binary constraints.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class SubgraphMatcher
  {
   public:

    SubgraphMatcher();

    /*!Check if the two input planes fulfill a set of geometric constraints, and so, if they are candidates to be the same plane.*/
    bool evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraints2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraintsOdometry(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);
    bool evalUnaryConstraintsOdometry2D(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);

    /*!Check if the two pair of planes plane1-plane2 ans planeA-planeB fulfill the same geometric relationship, and so,
    if they are candidates to be the same planes.*/
    bool evalBinaryConstraints(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);
    bool evalBinaryConstraintsOdometry(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);

    /*!List of combinations that have been explored in the interpretation tree.*/  // Cambiar nombre
    std::vector<std::map<unsigned,unsigned> > alreadyExplored;

    /*!Find the best combination of planes correspondences given two subgraphs represeting local neighborhoods of planes.*/  // Cambiar nombre o Quitar!
    void exploreSubgraphTreeR(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);
    void exploreSubgraphTreeR_Area(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);

    /*!Set source (current) subgraph.*/
    void inline setSourceSubgraph(Subgraph &subgSrc){subgraphSrc = &subgSrc;}

    /*!Set target subgraph.*/
    void inline setTargetSubgraph(Subgraph &subgTrg){subgraphTrg = &subgTrg;}

    /*!Returns a list with plane matches from subgraphSrc to subgraphTrg.*/
//    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget);
    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget, const int option=0); // Options are

    /*!One subgraph to be matched.*/
    Subgraph *subgraphSrc;

    /*!The other subgraph to be matched.*/
    Subgraph *subgraphTrg;

    int nCheckConditions;

    int totalUnary;
    int semanticPair;
    int rejectSemantic;

    /*!Return the total area of the matched planes in the frame source.*/
    float calcAreaMatched(std::map<unsigned,unsigned> &matched_planes);

    /*!Set of thresholds for PbMap matching.*/
    config_heuristics configLocaliser;

   private:

    /*!List of planes correspondences.*/
    std::map<unsigned, unsigned> winnerMatch;
    float areaWinnerMatch;

    /*!Hash table for unary constraints.*/
    std::vector<std::vector<int8_t> > hashUnaryConstraints;

    float calcAreaUnmatched(std::set<unsigned> &unmatched_planes);

  };

} } // End of namespaces

#endif
#endif
