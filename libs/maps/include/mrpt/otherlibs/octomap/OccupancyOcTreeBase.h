/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef OCTOMAP_OCCUPANCY_OCTREE_BASE_H
#define OCTOMAP_OCCUPANCY_OCTREE_BASE_H

// $Id: OccupancyOcTreeBase.h 436 2012-10-15 10:18:16Z ahornung $

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

#include <list>
#include <stdlib.h>

#include "octomap_types.h"
#include "octomap_utils.h"
#include "OcTreeBaseImpl.h"
#include "AbstractOccupancyOcTree.h"


namespace octomap {

  /**
   * Base implementation for Occupancy Octrees (e.g. for mapping).
   * AbstractOccupancyOcTree serves as a common
   * base interface for all these classes.
   * Each class used as NODE type needs to be derived from
   * OccupancyOcTreeNode.
   *
   * This tree implementation has a maximum depth of 16. 
   * At a resolution of 1 cm, values have to be < +/- 327.68 meters (2^15)
   *
   * This limitation enables the use of an efficient key generation 
   * method which uses the binary representation of the data.
   *
   * \note The tree does not save individual points.
   *
   * \tparam NODE Node class to be used in tree (usually derived from
   *    OcTreeDataNode)
   */
  template <class NODE>
  class OccupancyOcTreeBase : public OcTreeBaseImpl<NODE,AbstractOccupancyOcTree> {

  public:
    /// Default constructor, sets resolution of leafs
    OccupancyOcTreeBase(double resolution);
    virtual ~OccupancyOcTreeBase();

    /// Copy constructor
    OccupancyOcTreeBase(const OccupancyOcTreeBase<NODE>& rhs);

     /**
     * Integrate a Pointcloud (in global reference frame)
     *
     * @param scan Pointcloud (measurement endpoints), in global reference frame
     * @param sensor_origin measurement origin in global reference frame
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     */
    virtual void insertScan(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                    double maxrange=-1., bool pruning=true, bool lazy_eval = false);

	template <class SCAN>
	void insertScan_templ(const SCAN& scan, const octomap::point3d& sensor_origin,
                    double maxrange=-1., bool pruning=true, bool lazy_eval = false);

     /**
     * Integrate a 3d scan, transform scan before tree update
     *
     * @param scan Pointcloud (measurement endpoints) relative to frame origin
     * @param sensor_origin origin of sensor relative to frame origin
     * @param frame_origin origin of reference frame, determines transform to be applied to cloud and sensor origin
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     */
    virtual void insertScan(const Pointcloud& scan, const point3d& sensor_origin, const pose6d& frame_origin,
                    double maxrange=-1., bool pruning = true, bool lazy_eval = false);

    /**
     * Insert a 3d scan (given as a ScanNode) into the tree.
     *
     * @param scan ScanNode contains Pointcloud data and frame/sensor origin
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     * @param lazy_eval whether the tree is left 'dirty' after the update (default: false).
     *   This speeds up the insertion by not updating inner nodes, but you need to call updateInnerOccupancy() when done.
     */
    virtual void insertScan(const ScanNode& scan, double maxrange=-1., bool pruning = true, bool lazy_eval = false);

    /// for testing only
    virtual void insertScanNaive(const Pointcloud& pc, const point3d& origin, double maxrange, bool pruning = true, bool lazy_eval = false);

    /**
     * Manipulate log_odds value of voxel directly
     *
     * @param key OcTreeKey of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const OcTreeKey& key, float log_odds_update, bool lazy_eval = false);

    /**
     * Manipulate log_odds value of voxel directly.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const point3d& value, float log_odds_update, bool lazy_eval = false);

    /**
     * Manipulate log_odds value of voxel directly.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param x
     * @param y
     * @param z
     * @param log_odds_update value to be added (+) to log_odds value of node
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     *
     * @param key OcTreeKey of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const point3d& value, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param x
     * @param y
     * @param z
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(double x, double y, double z, bool occupied, bool lazy_eval = false);


    /**
     * Creates the maximum likelihood map by calling toMaxLikelihood on all
     * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
     * This enables a very efficient compression if you call prune() afterwards.
     */
    virtual void toMaxLikelihood();

    /**
     * Insert one ray between origin and end into the tree.
     * integrateMissOnRay() is called for the ray, the end point is updated as occupied.
     *
     * @param origin origin of sensor in global coordinates
     * @param end endpoint of measurement in global coordinates
     * @param maxrange maximum range after which the raycast should be aborted
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return success of operation
     */
    virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange=-1.0, bool lazy_eval = false);
    
    /**
     * Performs raycasting in 3d, similar to computeRay().
     *
     * A ray is cast from origin with a given direction, the first occupied
     * cell is returned (as center coordinate). If the starting coordinate is already
     * occupied in the tree, this coordinate will be returned as a hit.
     *
     * @param origin starting coordinate of ray
     * @param direction A vector pointing in the direction of the raycast. Does not need to be normalized.
     * @param end returns the center of the cell that was hit by the ray, if successful
     * @param ignoreUnknownCells whether unknown cells are ignored. If false (default), the raycast aborts when an unkown cell is hit.
     * @param maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
     * @return whether or not an occupied cell was hit
     */
    virtual bool castRay(const point3d& origin, const point3d& direction, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0) const;
   
    /**
     * Convenience function to return all occupied nodes in the OcTree.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!
     *
     * @param node_centers list of occpupied nodes (as point3d)
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    DEPRECATED( void getOccupied(point3d_list& node_centers, unsigned int max_depth = 0) const);
    
    /**
     * Convenience function to return all occupied nodes in the OcTree.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!     *
     *
     * @param occupied_volumes list of occpupied nodes (as point3d and size of the volume)
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    DEPRECATED(void getOccupied(std::list<OcTreeVolume>& occupied_volumes, unsigned int max_depth = 0) const);

    /**
     * Traverses the tree and collects all OcTreeVolumes regarded as occupied.
     * Inner nodes with both occupied and free children are regarded as occupied. 
     * This should be for internal use only, use getOccupied(occupied_volumes) instead.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!     *
     *
     * @param binary_nodes list of binary OcTreeVolumes which are occupied
     * @param delta_nodes list of delta OcTreeVolumes which are occupied
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    DEPRECATED(void getOccupied(std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes,
                     unsigned int max_depth = 0) const);


    /**
     * returns occupied leafs within a bounding box defined by min and max.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!
     */
    DEPRECATED(void getOccupiedLeafsBBX(point3d_list& node_centers, point3d min, point3d max) const);

    /**
     * Convenience function to return all free nodes in the OcTree.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!
     *
     * @param free_volumes list of free nodes (as point3d and size of the volume)
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    DEPRECATED(void getFreespace(std::list<OcTreeVolume>& free_volumes, unsigned int max_depth = 0) const);

    /**
     * Traverses the tree and collects all OcTreeVolumes regarded as free.
     * Inner nodes with both occupied and free children are regarded as occupied.
     * @note Deprecated, will be removed in the future.
     * Direcly access the nodes with iterators instead!
     *
     * @param binary_nodes list of binary OcTreeVolumes which are free
     * @param delta_nodes list of delta OcTreeVolumes which are free
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    DEPRECATED(void getFreespace(std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes,
                      unsigned int max_depth = 0) const);



    //-- set BBX limit (limits tree updates to this bounding box  

    ///  use or ignore BBX limit (default: ignore)
    void useBBXLimit(bool enable) { use_bbx_limit = enable; }
    bool bbxSet() const { return use_bbx_limit; }
    /// sets the minimum for a query bounding box to use
    void setBBXMin (point3d& min);
    /// sets the maximum for a query bounding box to use
    void setBBXMax (point3d& max);
    /// @return the currently set minimum for bounding box queries, if set
    point3d getBBXMin () const { return bbx_min; }
    /// @return the currently set maximum for bounding box queries, if set
    point3d getBBXMax () const { return bbx_max; }
    point3d getBBXBounds () const;
    point3d getBBXCenter () const;
    /// @return true if point is in the currently set bounding box
    bool inBBX(const point3d& p) const;
    /// @return true if key is in the currently set bounding box
    bool inBBX(const OcTreeKey& key) const;

    //-- change detection on occupancy:
    /// track or ignore changes while inserting scans (default: ignore)
    void enableChangeDetection(bool enable) { use_change_detection = enable; }
    /// Reset the set of changed keys. Call this after you obtained all changed nodes.
    void resetChangeDetection() { changed_keys.clear(); }

    /**
     * Iterator to traverse all keys of changed nodes.
     * you need to enableChangeDetection() first. Here, an OcTreeKey always
     * refers to a node at the lowest tree level (its size is the minimum tree resolution)
     */
    KeyBoolMap::const_iterator changedKeysBegin() {return changed_keys.begin();}

    /// Iterator to traverse all keys of changed nodes.
    KeyBoolMap::const_iterator changedKeysEnd() {return changed_keys.end();}


    /**
     * Helper for insertScan. Computes all octree nodes affected by the point cloud
     * integration at once. Here, occupied nodes have a preference over free
     * ones.
     *
     * @param scan point cloud measurement to be integrated
     * @param origin origin of the sensor for ray casting
     * @param free_cells keys of nodes to be cleared
     * @param occupied_cells keys of nodes to be marked occupied
     * @param maxrange maximum range for raycasting (-1: unlimited)
     */
    void computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);

	template <class POINTCLOUD>
	void computeUpdate_templ(const POINTCLOUD& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);


	inline void computeUpdate_onePoint(const point3d& p, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);

    // -- I/O  -----------------------------------------

    /**
     * Reads only the data (=tree structure) from the input stream.
     * The tree needs to be constructed with the proper header information
     * beforehand, see readBinary().
     */
    std::istream& readBinaryData(std::istream &s);

    /**
     * Read node from binary stream (max-likelihood value), recursively
     * continue with all children.
     *
     * This will set the log_odds_occupancy value of
     * all leaves to either free or occupied.
     *
     * @param s
     * @return
     */
    std::istream& readBinaryNode(std::istream &s, NODE* node) const;

    /**
     * Write node to binary stream (max-likelihood value),
     * recursively continue with all children.
     *
     * This will discard the log_odds_occupancy value, writing
     * all leaves as either free or occupied.
     *
     * @param s
     * @param node OcTreeNode to write out, will recurse to all children
     * @return
     */
    std::ostream& writeBinaryNode(std::ostream &s, const NODE* node) const;

    /**
     * Writes the data of the tree (without header) to the stream, recursively
     * calling writeBinaryNode (starting with root)
     */
    std::ostream& writeBinaryData(std::ostream &s) const;


    void calcNumThresholdedNodes(unsigned int& num_thresholded, unsigned int& num_other) const;

    /**
     * Updates the occupancy of all inner nodes to reflect their children's occupancy.
     * If you performed batch-updates with lazy evaluation enabled, you must call this
     * before any queries to ensure correct multi-resolution behavior.
     **/
    void updateInnerOccupancy();


    /// integrate a "hit" measurement according to the tree's sensor model
    virtual void integrateHit(NODE* occupancyNode) const;
    /// integrate a "miss" measurement according to the tree's sensor model
    virtual void integrateMiss(NODE* occupancyNode) const;
    // update logodds value of node, given update is added to current value.
    virtual void updateNodeLogOdds(NODE* occupancyNode, const float& update) const;

    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE* occupancyNode) const;
    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE& occupancyNode) const;

  protected:
    /// Constructor to enable derived classes to change tree constants.
    /// This usually requires a re-implementation of some core tree-traversal functions as well!
    OccupancyOcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

    /**
     * Traces a ray from origin to end and updates all voxels on the
     *  way as free.  The volume containing "end" is not updated.
     */
    inline bool integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval = false);


    // recursive calls ----------------------------

    NODE* updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const float& log_odds_update, bool lazy_eval = false);
    
    void updateInnerOccupancyRecurs(NODE* node, unsigned int depth);

    void getOccupiedLeafsBBXRecurs( point3d_list& node_centers, unsigned int max_depth, NODE* node, 
                                    unsigned int depth, const OcTreeKey& parent_key, 
                                    const OcTreeKey& min, const OcTreeKey& max) const;
    
    void toMaxLikelihoodRecurs(NODE* node, unsigned int depth, unsigned int max_depth);

    void calcNumThresholdedNodesRecurs (NODE* node,
                                        unsigned int& num_thresholded,
                                        unsigned int& num_other) const;

  protected:
    bool use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
    point3d bbx_min;
    point3d bbx_max;
    OcTreeKey bbx_min_key;
    OcTreeKey bbx_max_key;

    bool use_change_detection;
    /// Set of leaf keys (lowest level) which changed since last resetChangeDetection
    KeyBoolMap changed_keys;
    

  };

} // namespace

#include "mrpt/otherlibs/octomap/OccupancyOcTreeBase.hxx"

#endif
