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

#ifndef __SUBGRAPHMATCHER_H
#define __SUBGRAPHMATCHER_H

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>

#include "heuristicParams.h"
#include "PbMap.h"
#include "Subgraph.h"

namespace mrpt {
namespace pbmap {

  /** This class finds the best correspondence between the planes of two subgraphs (i.e. sets of neighbor planes).
   * It relies on an interpretation tree employing geometric restrictions that are represented as a set of unary and binary constraints.
   * \ingroup mrpt_pbmap_grp
   */
  class SubgraphMatcher
  {
   public:

    SubgraphMatcher(){};

    /*!Check if the two input planes fulfill a set of geometric constraints, and so, if they are candidates to be the same plane.*/
    bool evalUnaryConstraints(Plane &plane1, Plane &plane2, PbMap &trgPbMap, bool useStructure = false);

    /*!Check if the two pair of planes plane1-plane2 ans planeA-planeB fulfill the same geometric relationship, and so,
    if they are candidates to be the same planes.*/
    bool evalBinaryConstraints(Plane &plane1, Plane &plane2, Plane &planeA, Plane &planeB);

    /*!List of combinations that have been explored in the interpretation tree.*/  // Cambiar nombre
    std::vector<std::map<unsigned,unsigned> > alreadyExplored;

    /*!Find the best combination of planes correspondences given two subgraphs represeting local neighborhoods of planes.*/  // Cambiar nombre o Quitar!
    void exploreSubgraphTreeR(std::set<unsigned> &evalRef, std::set<unsigned> &evalCheck, std::map<unsigned, unsigned> &matched);

    /*!Set source (current) subgraph.*/
    void inline setSourceSubgraph(Subgraph &subgSrc){subgraphSrc = &subgSrc;}

    /*!Set target subgraph.*/
    void inline setTargetSubgraph(Subgraph &subgTrg){subgraphTrg = &subgTrg;}

    /*!Returns a list with plane matches from subgraphSrc to subgraphTrg.*/
    std::map<unsigned,unsigned> compareSubgraphs(Subgraph &subgraphSource, Subgraph &subgraphTarget);

    /*!One subgraph to be matched.*/
    Subgraph *subgraphSrc;

    /*!The other subgraph to be matched.*/
    Subgraph *subgraphTrg;

    int nCheckConditions;
   private:

    /*!List of planes correspondences.*/
    std::map<unsigned, unsigned> winnerMatch;

  };

} } // End of namespaces

#endif
