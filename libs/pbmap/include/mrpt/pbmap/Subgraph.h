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

#ifndef __SUBGRAPH_H
#define __SUBGRAPH_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>
#include <mrpt/pbmap/link_pragmas.h>

#include <mrpt/pbmap/PbMap.h>

namespace mrpt {
namespace pbmap {

  /*! This class defines a subgraph inside a PbMap.
   *  This subgraph represents the entity to be matched in order to recognize a previous place.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PBMAP_IMPEXP Subgraph
  {
   public:

    /*!Construct a subgraph, inside a given PbMap, as a set of neighborh planes (1-connected) of a reference plane.*/
    Subgraph(PbMap *pPbMap, const unsigned &refPlaneId) :
      pPBM(pPbMap)
    {
      subgraphPlanesIdx.insert(refPlaneId);
      // Add proximity neighbors
      for(std::set<unsigned>::iterator it = pPBM->vPlanes[refPlaneId].nearbyPlanes.begin(); it != pPBM->vPlanes[refPlaneId].nearbyPlanes.end(); it++)
        subgraphPlanesIdx.insert(*it);

      #ifdef _VERBOSE
        // Add neighbors co-visible neighbors
        for(std::map<unsigned,unsigned>::iterator it = pPBM->vPlanes[refPlaneId].neighborPlanes.begin(); it != pPBM->vPlanes[refPlaneId].neighborPlanes.end(); it++)
          subgraphPlanesIdx.insert(it->first);
        std::cout << "Subgraph constructor: ";
        for(std::set<unsigned>::iterator it = subgraphPlanesIdx.begin(); it != subgraphPlanesIdx.end(); it++)
          std::cout << *it << " ";
        std::cout << std::endl;
      #endif
    };

    PbMap *pPBM;
    std::set<unsigned> subgraphPlanesIdx;

  };

} } // End of namespaces

#endif
#endif
