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
    /*! Subgraph constructor with no parameters*/
    Subgraph(){};

    /*! Construct a subgraph, inside a given PbMap, as a set of neighborh planes (1-connected) of a reference plane.*/
    Subgraph(PbMap *pPbMap, const unsigned &refPlaneId) :
      pPBM(pPbMap)
    {
        // This only works when there is only one reference plane and we select only the 1-connected planes
        subgraphPlanesIdx.push_back(refPlaneId);
        for(std::map<unsigned,unsigned>::iterator it = pPBM->vPlanes[refPlaneId].neighborPlanes.begin(); it != pPBM->vPlanes[refPlaneId].neighborPlanes.end(); it++)
          subgraphPlanesIdx.push_back(it->first);

//      std::set<unsigned> subgraph_neigs;
//      subgraph_neigs.insert(refPlaneId);
//      // Add proximity neighbors
////      for(std::set<unsigned>::iterator it = pPBM->vPlanes[refPlaneId].nearbyPlanes.begin(); it != pPBM->vPlanes[refPlaneId].nearbyPlanes.end(); it++)
////        subgraph_neigs.insert(*it);

//        // Add neighbors co-visible neighbors
//        for(std::map<unsigned,unsigned>::iterator it = pPBM->vPlanes[refPlaneId].neighborPlanes.begin(); it != pPBM->vPlanes[refPlaneId].neighborPlanes.end(); it++)
//          subgraph_neigs.insert(it->first);

//        subgraphPlanesIdx.resize( subgraph_neigs.size() );
//        int i=0;
//        for(std::set<unsigned>::iterator it = subgraph_neigs.begin(); it != subgraph_neigs.end(); it++)
//            subgraphPlanesIdx[i++] == *it;

      #ifdef _VERBOSE
        std::cout << "Subgraph constructor: ";
        for(std::vector<unsigned>::iterator it = subgraphPlanesIdx.begin(); it != subgraphPlanesIdx.end(); it++)
          std::cout << *it << " ";
        std::cout << std::endl;
      #endif
    };

    PbMap *pPBM;
    std::vector<unsigned> subgraphPlanesIdx;

    /*! Return true if the Subgraph is empty */
    inline bool empty() const
    {
        return subgraphPlanesIdx.empty();
    };

  };

} } // End of namespaces

#endif
#endif
