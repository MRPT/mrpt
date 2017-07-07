/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef OCTOMAP_OCTREE_H
#define OCTOMAP_OCTREE_H

// $Id: OcTree.h 391 2012-06-21 10:07:53Z ahornung $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "OccupancyOcTreeBase.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"
#include <mrpt/maps/link_pragmas.h>  // For DLL export within mrpt-maps via the MAPS_IMPEXP macro

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class OcTree : public OccupancyOcTreeBase <OcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    OcTree(double resolution) : OccupancyOcTreeBase<OcTreeNode>(resolution) {};

    /**
     * Reads an OcTree from a binary file 
    * @param _filename
     *
     */
    OcTree(std::string _filename);

    virtual ~OcTree(){};

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTree* create() const {return new OcTree(resolution); }

    std::string getTreeType() const {return "OcTree";}


  protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTree* tree = new OcTree(0.1);
        AbstractOcTree::registerTreeType(tree);
      }
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeMemberInit;
  };

} // end namespace

#endif
