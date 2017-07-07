/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef OCTOMAP_COUNTING_OCTREE_HH
#define OCTOMAP_COUNTING_OCTREE_HH

// $Id: CountingOcTree.h 391 2012-06-21 10:07:53Z ahornung $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009-2011, K. M. Wurm, A. Hornung, University of Freiburg
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


#include <stdio.h>
#include "OcTreeBase.h"
#include "OcTreeDataNode.h"
#include <mrpt/maps/link_pragmas.h>  // For DLL export within mrpt-maps via the MAPS_IMPEXP macro

namespace octomap {

  /**
   * An Octree-node which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note In our mapping system this data structure is used in
   *       CountingOcTree in the sensor model only
   */
  class /*MAPS_IMPEXP*/ CountingOcTreeNode : public OcTreeDataNode<unsigned int> {

  public:

    CountingOcTreeNode();
    ~CountingOcTreeNode();
    bool createChild(unsigned int i);

    inline CountingOcTreeNode* getChild(unsigned int i) {
      return static_cast<CountingOcTreeNode*> (OcTreeDataNode<unsigned int>::getChild(i));
    }

    inline const CountingOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const CountingOcTreeNode*> (OcTreeDataNode<unsigned int>::getChild(i));
    }

    inline unsigned int getCount() const { return getValue(); }
    inline void increaseCount() { value++; }
    inline void setCount(unsigned c) {this->setValue(c); }

    // overloaded:
    void expandNode();
  };



  /**
   * An AbstractOcTree which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note In our mapping system this data structure is used in
   *       the sensor model only. Do not use, e.g., insertScan.
   */
  class /*MAPS_IMPEXP*/ CountingOcTree : public OcTreeBase <CountingOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    CountingOcTree(double resolution) : OcTreeBase<CountingOcTreeNode>(resolution) {};    
    virtual CountingOcTreeNode* updateNode(const point3d& value);
    CountingOcTreeNode* updateNode(const OcTreeKey& k);
    void getCentersMinHits(point3d_list& node_centers, unsigned int min_hits) const;

  protected:

    void getCentersMinHitsRecurs( point3d_list& node_centers,
                                  unsigned int& min_hits,
                                  unsigned int max_depth,
                                  CountingOcTreeNode* node, unsigned int depth,
                                  const OcTreeKey& parent_key) const;

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           CountingOcTree* tree = new CountingOcTree(0.1);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer countingOcTreeMemberInit;
  };


}


#endif
