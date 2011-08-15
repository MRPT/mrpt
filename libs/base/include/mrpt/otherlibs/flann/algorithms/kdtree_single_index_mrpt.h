/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#ifndef  KDTREESINGLE_ADAPTOR_MRPT_H
#define  KDTREESINGLE_ADAPTOR_MRPT_H

#include <algorithm>
#include <map>
#include <cassert>
#include <cstring>

#include "mrpt/otherlibs/flann/general.h"
#include "mrpt/otherlibs/flann/algorithms/nn_index.h"
#include "mrpt/otherlibs/flann/util/matrix.h"
#include "mrpt/otherlibs/flann/util/result_set.h"
#include "mrpt/otherlibs/flann/util/heap.h"
#include "mrpt/otherlibs/flann/util/allocator.h"
#include "mrpt/otherlibs/flann/util/random.h"
#include "mrpt/otherlibs/flann/util/saving.h"
#include "mrpt/otherlibs/flann/util/logger.h"

namespace mrpt_flann  // Modified namespace to avoid naming conflicts with 3rd party code
{


/**
 * Squared Euclidean distance functor - modified version for MRPT that doesn't assume data comes in a "T*", but
 *  relies on a generic "DataSource".
 */
template<class T, class DataSource>
struct L2_Simple_Adaptor
{
    typedef T     ElementType;
    typedef T     ResultType;

	const DataSource &data_source;

	L2_Simple_Adaptor(const DataSource &_data_source) : data_source(_data_source) { }

    inline ResultType operator()(const T* a, const size_t b_idx, size_t size) const {
		return data_source.kdtree_distance(a,b_idx,size);
    }

    template <typename U, typename V>
    inline ResultType accum_dist(const U a, const V b, int dim) const
    {
        return (a-b)*(a-b);
    }
};

/// DIM>0 forces the dimensionality; -1: dynamic dimensionality
struct KDTreeSingleIndexAdaptorParams : public IndexParams
{
    KDTreeSingleIndexAdaptorParams(int leaf_max_size_ = 10, int dim_ = -1) :
        IndexParams(FLANN_INDEX_KDTREE_SINGLE), leaf_max_size(leaf_max_size_),
        dim(dim_) {}

    int leaf_max_size;
    int dim;

    flann_algorithm_t getIndexType() const { return algorithm; }

    void fromParameters(const FLANNParameters& p)
    {
        assert(p.algorithm==algorithm);
    }

    void toParameters(FLANNParameters& p) const
    {
        p.algorithm = algorithm;
    }

    void print() const
    {
        logger.info("Index type: %d\n",(int)algorithm);
    }

};


/**
 * Randomized kd-tree index
 *
 * Contains the k-d trees and other information for indexing a set of points
 * for nearest-neighbor matching.
 *
 *  The class "DatasetAdaptor" must provide the following interface (can be non-virtual, inlined methods):
 *
 *  \code
 *   // Must return the number of data points
 *   inline size_t kdtree_get_point_count() const { ... }
 *
 *   // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
 *   inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const { ... }
 *
 *   // Returns the dim'th component of the idx'th point in the class:
 *   inline num_t kdtree_get_pt(const size_t idx, int dim) const { ... }
 *
 *   // Optional bounding-box computation: return false to default to a standard bbox computation loop.
 *   //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
 *   //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
 *   template <class BBOX>
 *   bool kdtree_get_bbox(BBOX &bb) const
 *   {
 *      bb[0].low = ...; bb[0].high = ...;  // "x" limits
 *      return true;
 *   }
 *
 *  \endcode
 *
 */
template <typename Distance, class DatasetAdaptor,int DIM = -1>
class KDTreeSingleIndexAdaptor : public NNIndex<Distance>
{
    typedef typename Distance::ElementType ElementType;
    typedef typename Distance::ResultType DistanceType;

    /**
     *  Array of indices to vectors in the dataset.
     */
    std::vector<int> vind;

    int leaf_max_size_;


    /**
     * The dataset used by this index
     */
    //const Matrix<ElementType> dataset;
	const DatasetAdaptor &dataset; //!< The source of our data

    const KDTreeSingleIndexAdaptorParams index_params;

    size_t size_;
    size_t dim;


    /*--------------------- Internal Data Structures --------------------------*/
    struct Node
    {
        union {
            struct
            {
                /**
                 * Indices of points in leaf node
                 */
                int left, right;
            } lr;
            struct
            {
                /**
                 * Dimension used for subdivision.
                 */
                int divfeat;
                /**
                 * The values used for subdivision.
                 */
                DistanceType divlow, divhigh;
            } sub;
        };
        /**
         * The child nodes.
         */
        Node* child1, * child2;
    };
    typedef Node* NodePtr;


    struct Interval
    {
        ElementType low, high;
    };

    typedef std::vector<Interval> BoundingBox;

    /**
     * Array of k-d trees used to find neighbours.
     */
    NodePtr root_node;
    typedef BranchStruct<NodePtr, DistanceType> BranchSt;
    typedef BranchSt* Branch;

    BoundingBox root_bbox;

    /**
     * Pooled memory allocator.
     *
     * Using a pooled memory allocator is more efficient
     * than allocating memory directly when there is a large
     * number small of memory allocations.
     */
    PooledAllocator pool;

public:

    Distance distance;
    int count_leaf;

    flann_algorithm_t getType() const
    {
        return FLANN_INDEX_KDTREE_SINGLE;
    }

    /**
     * KDTree constructor
     *
     * Params:
     *          inputData = dataset with the input features
     *          params = parameters passed to the kdtree algorithm
     */
    KDTreeSingleIndexAdaptor(const int dimensionality, const DatasetAdaptor& inputData, const KDTreeSingleIndexAdaptorParams& params = KDTreeSingleIndexAdaptorParams() ) :
        dataset(inputData), index_params(params), distance(inputData.derived())
    {
        size_ = dataset.derived().kdtree_get_point_count();
        dim = dimensionality;
        if (DIM>0) dim=DIM; else { if (params.dim>0) dim = params.dim; }
        leaf_max_size_ = params.leaf_max_size;

        // Create a permutable array of indices to the input vectors.
        vind.resize(size_);
        for (size_t i = 0; i < size_; i++) {
            vind[i] = i;
        }

        count_leaf = 0;
    }

    /**
     * Standard destructor
     */
    ~KDTreeSingleIndexAdaptor()
    {
    }

    /**
     * Builds the index
     */
    void buildIndex()
    {
        computeBoundingBox(root_bbox);
        root_node = divideTree(0, size_, root_bbox );   // construct the tree
    }

    /**
     *  Returns size of index.
     */
    size_t size() const
    {
        return size_;
    }

    /**
     * Returns the length of an index feature.
     */
    size_t veclen() const
    {
        return (DIM>0 ? DIM : dim);
    }

    /**
     * Computes the inde memory usage
     * Returns: memory used by the index
     */
    int usedMemory() const
    {
        return pool.usedMemory+pool.wastedMemory+dataset.derived().kdtree_get_point_count()*sizeof(int);  // pool memory and vind array memory
    }

    /**
     * Find set of nearest neighbors to vec. Their indices are stored inside
     * the result object.
     *
     * Params:
     *     result = the result object in which the indices of the nearest-neighbors are stored
     *     vec = the vector for which to search the nearest neighbors
     *     maxCheck = the maximum number of restarts (in a best-bin-first manner)
     *
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     */
	template <typename RESULTSET>
    void findNeighbors(RESULTSET& result, const ElementType* vec, const SearchParams& searchParams)
    {
        float epsError = 1+searchParams.eps;

        std::vector<DistanceType> dists( (DIM>0 ? DIM : dim) ,0);
        DistanceType distsq = computeInitialDistances(vec, dists);
        searchLevel(result, vec, root_node, distsq, dists, epsError);
    }

    const IndexParams* getParameters() const
    {
        return &index_params;
    }

private:

	/// Helper accessor to the dataset points:
	inline ElementType dataset_get(size_t idx, int component) const {  return dataset.derived().kdtree_get_pt(idx,component); }


    void save_tree(FILE* stream, NodePtr tree)
    {
        save_value(stream, *tree);
        if (tree->child1!=NULL) {
            save_tree(stream, tree->child1);
        }
        if (tree->child2!=NULL) {
            save_tree(stream, tree->child2);
        }
    }


    void load_tree(FILE* stream, NodePtr& tree)
    {
        tree = pool.allocate<Node>();
        load_value(stream, *tree);
        if (tree->child1!=NULL) {
            load_tree(stream, tree->child1);
        }
        if (tree->child2!=NULL) {
            load_tree(stream, tree->child2);
        }
    }


    void computeBoundingBox(BoundingBox& bbox)
    {
		bbox.resize((DIM>0 ? DIM : dim));
		if (dataset.derived().kdtree_get_bbox(bbox))
		{
			// Done! It was implemented in derived class
		}
		else
		{
			for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
				bbox[i].low =
				bbox[i].high = dataset_get(0,i);
			}
			const size_t N = dataset.derived().kdtree_get_point_count();
			for (size_t k=1; k<N; ++k) {
				for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
					if (dataset_get(k,i)<bbox[i].low) bbox[i].low = dataset_get(k,i);
					if (dataset_get(k,i)>bbox[i].high) bbox[i].high = dataset_get(k,i);
				}
			}
		}
    }


    /**
     * Create a tree node that subdivides the list of vecs from vind[first]
     * to vind[last].  The routine is called recursively on each sublist.
     * Place a pointer to this new tree node in the location pTree.
     *
     * Params: pTree = the new node to create
     *                  first = index of the first vector
     *                  last = index of the last vector
     */
    NodePtr divideTree(int left, int right, BoundingBox& bbox)
    {
        NodePtr node = pool.allocate<Node>(); // allocate memory

        /* If too few exemplars remain, then make this a leaf node. */
        if ( (right-left) <= leaf_max_size_) {
            node->child1 = node->child2 = NULL;    /* Mark as leaf node. */
            node->lr.left = left;
            node->lr.right = right;

            // compute bounding-box of leaf points
            for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
                bbox[i].low = dataset_get(vind[left],i);
                bbox[i].high = dataset_get(vind[left],i);
            }
            for (int k=left+1; k<right; ++k) {
                for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
                    if (bbox[i].low>dataset_get(vind[k],i)) bbox[i].low=dataset_get(vind[k],i);
                    if (bbox[i].high<dataset_get(vind[k],i)) bbox[i].high=dataset_get(vind[k],i);
                }
            }
        }
        else {
            int idx;
            int cutfeat;
            DistanceType cutval;
            middleSplit_(&vind[0]+left, right-left, idx, cutfeat, cutval, bbox);

            node->sub.divfeat = cutfeat;

            BoundingBox left_bbox(bbox);
            left_bbox[cutfeat].high = cutval;
            node->child1 = divideTree(left, left+idx, left_bbox);

            BoundingBox right_bbox(bbox);
            right_bbox[cutfeat].low = cutval;
            node->child2 = divideTree(left+idx, right, right_bbox);

            node->sub.divlow = left_bbox[cutfeat].high;
            node->sub.divhigh = right_bbox[cutfeat].low;

            for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
                bbox[i].low = std::min(left_bbox[i].low, right_bbox[i].low);
                bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
            }
        }

        return node;
    }

    void computeMinMax(int* ind, int count, int element, ElementType& min_elem, ElementType& max_elem)
    {
        min_elem = dataset_get(ind[0],element);
        max_elem = dataset_get(ind[0],element);
        for (int i=1; i<count; ++i) {
            ElementType val = dataset_get(ind[i],element);
            if (val<min_elem) min_elem = val;
            if (val>max_elem) max_elem = val;
        }
    }

    void middleSplit(int* ind, int count, int& index, int& cutfeat, DistanceType& cutval, const BoundingBox& bbox)
    {
        // find the largest span from the approximate bounding box
        ElementType max_span = bbox[0].high-bbox[0].low;
        cutfeat = 0;
        cutval = (bbox[0].high+bbox[0].low)/2;
        for (size_t i=1; i<(DIM>0 ? DIM : dim); ++i) {
            ElementType span = bbox[i].low-bbox[i].low;
            if (span>max_span) {
                max_span = span;
                cutfeat = i;
                cutval = (bbox[i].high+bbox[i].low)/2;
            }
        }

        // compute exact span on the found dimension
        ElementType min_elem, max_elem;
        computeMinMax(ind, count, cutfeat, min_elem, max_elem);
        cutval = (min_elem+max_elem)/2;
        max_span = max_elem - min_elem;

        // check if a dimension of a largest span exists
        size_t k = cutfeat;
        for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
            if (i==k) continue;
            ElementType span = bbox[i].high-bbox[i].low;
            if (span>max_span) {
                computeMinMax(ind, count, i, min_elem, max_elem);
                span = max_elem - min_elem;
                if (span>max_span) {
                    max_span = span;
                    cutfeat = i;
                    cutval = (min_elem+max_elem)/2;
                }
            }
        }
        int lim1, lim2;
        planeSplit(ind, count, cutfeat, cutval, lim1, lim2);

        if (lim1>count/2) index = lim1;
        else if (lim2<count/2) index = lim2;
        else index = count/2;
    }


    void middleSplit_(int* ind, int count, int& index, int& cutfeat, DistanceType& cutval, const BoundingBox& bbox)
    {
        const float EPS=0.00001;
        ElementType max_span = bbox[0].high-bbox[0].low;
        for (size_t i=1;i<(DIM>0 ? DIM : dim);++i) {
            ElementType span = bbox[i].high-bbox[i].low;
            if (span>max_span) {
                max_span = span;
            }
        }
        ElementType max_spread = -1;
        cutfeat = 0;
        for (size_t i=0;i<(DIM>0 ? DIM : dim);++i) {
            ElementType span = bbox[i].high-bbox[i].low;
            if (span>(1-EPS)*max_span) {
                ElementType min_elem, max_elem;
                computeMinMax(ind, count, cutfeat, min_elem, max_elem);
                ElementType spread = max_elem-min_elem;;
                if (spread>max_spread) {
                    cutfeat = i;
                    max_spread = spread;
                }
            }
        }
        // split in the middle
        DistanceType split_val = (bbox[cutfeat].low+bbox[cutfeat].high)/2;
        ElementType min_elem, max_elem;
        computeMinMax(ind, count, cutfeat, min_elem, max_elem);

        if (split_val<min_elem) cutval = min_elem;
        else if (split_val>max_elem) cutval = max_elem;
        else cutval = split_val;

        int lim1, lim2;
        planeSplit(ind, count, cutfeat, cutval, lim1, lim2);

        if (lim1>count/2) index = lim1;
        else if (lim2<count/2) index = lim2;
        else index = count/2;
    }


    /**
     *  Subdivide the list of points by a plane perpendicular on axe corresponding
     *  to the 'cutfeat' dimension at 'cutval' position.
     *
     *  On return:
     *  dataset[ind[0..lim1-1]][cutfeat]<cutval
     *  dataset[ind[lim1..lim2-1]][cutfeat]==cutval
     *  dataset[ind[lim2..count]][cutfeat]>cutval
     */
    void planeSplit(int* ind, int count, int cutfeat, DistanceType cutval, int& lim1, int& lim2)
    {
        /* Move vector indices for left subtree to front of list. */
        int left = 0;
        int right = count-1;
        for (;; ) {
            while (left<=right && dataset_get(ind[left],cutfeat)<cutval) ++left;
            while (left<=right && dataset_get(ind[right],cutfeat)>=cutval) --right;
            if (left>right) break;
            std::swap(ind[left], ind[right]); ++left; --right;
        }
        /* If either list is empty, it means that all remaining features
         * are identical. Split in the middle to maintain a balanced tree.
         */
        lim1 = left;
        right = count-1;
        for (;; ) {
            while (left<=right && dataset_get(ind[left],cutfeat)<=cutval) ++left;
            while (left<=right && dataset_get(ind[right],cutfeat)>cutval) --right;
            if (left>right) break;
            std::swap(ind[left], ind[right]); ++left; --right;
        }
        lim2 = left;
    }

    DistanceType computeInitialDistances(const ElementType* vec, std::vector<DistanceType>& dists)
    {
        DistanceType distsq = 0.0;

        for (size_t i = 0; i < (DIM>0 ? DIM : dim); ++i) {
            if (vec[i] < root_bbox[i].low) {
                dists[i] = distance.accum_dist(vec[i], root_bbox[i].low, i);
                distsq += dists[i];
            }
            if (vec[i] > root_bbox[i].high) {
                dists[i] = distance.accum_dist(vec[i], root_bbox[i].high, i);
                distsq += dists[i];
            }
        }

        return distsq;
    }

    /**
     * Performs an exact search in the tree starting from a node.
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     */
	template <class RESULTSET>
    void searchLevel(RESULTSET& result_set, const ElementType* vec, const NodePtr node, DistanceType mindistsq,
                     std::vector<DistanceType>& dists, const float epsError)
    {
        /* If this is a leaf node, then do check and return. */
        if ((node->child1 == NULL)&&(node->child2 == NULL)) {
            count_leaf += (node->lr.right-node->lr.left);
            DistanceType worst_dist = result_set.worstDist();
            for (int i=node->lr.left; i<node->lr.right; ++i) {
				int index = vind[i];// reorder... : i;
                DistanceType dist = distance(vec, index, (DIM>0 ? DIM : dim));
                if (dist<worst_dist) {
                    result_set.addPoint(dist,vind[i]);
                }
            }
            return;
        }

        /* Which child branch should be taken first? */
        int idx = node->sub.divfeat;
        ElementType val = vec[idx];
        DistanceType diff1 = val - node->sub.divlow;
        DistanceType diff2 = val - node->sub.divhigh;

        NodePtr bestChild;
        NodePtr otherChild;
        DistanceType cut_dist;
        if ((diff1+diff2)<0) {
            bestChild = node->child1;
            otherChild = node->child2;
            cut_dist = distance.accum_dist(val, node->sub.divhigh, idx);
        }
        else {
            bestChild = node->child2;
            otherChild = node->child1;
            cut_dist = distance.accum_dist( val, node->sub.divlow, idx);
        }

        /* Call recursively to search next level down. */
        searchLevel(result_set, vec, bestChild, mindistsq, dists, epsError);

        DistanceType dst = dists[idx];
        mindistsq = mindistsq + cut_dist - dst;
        dists[idx] = cut_dist;
        if (mindistsq*epsError<=result_set.worstDist()) {
            searchLevel(result_set, vec, otherChild, mindistsq, dists, epsError);
        }
        dists[idx] = dst;
    }


    void saveIndex(FILE* stream)
    {
        save_value(stream, size_);
        save_value(stream, dim);
        save_value(stream, root_bbox);
        save_value(stream, leaf_max_size_);
        save_value(stream, vind);
        save_tree(stream, root_node);
    }

    void loadIndex(FILE* stream)
    {
        load_value(stream, size_);
        load_value(stream, dim);
        load_value(stream, root_bbox);
        load_value(stream, leaf_max_size_);
        load_value(stream, vind);
        load_tree(stream, root_node);
    }

};   // class KDTree

}

#endif //KDTREESINGLE_H
