/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// BEWARE: BETA VERSION
// --------------------
//
// A k-d tree that vastly speeds up an iteration of k-means (in any number of dimensions). The main
// idea for this data structure is from Kanungo/Mount. This is used internally by Kmeans.cpp, and
// will most likely not need to be used directly.
//
// The stucture works as follows:
//   - All data points are placed into a tree where we choose child nodes by partitioning all data
//     points along a plane parallel to the axis.
//   - We maintain for each node, the bounding box of all data points stored at that node.
//   - To do a k-means iteration, we need to assign points to clusters and calculate the sum and
//     the number of points assigned to each cluster. For each node in the tree, we can rule out
//     some cluster centers as being too far away from every single point in that bounding box.
//     Once only one cluster is left, all points in the node can be assigned to that cluster in
//     batch.
//
// Author: David Arthur (darthur@gmail.com), 2009

#ifndef KM_TREE_H__
#define KM_TREE_H__

// Includes
#include "KmUtils.h"

// KmTree class definition
class KmTree {
 public:
  // Constructs a tree out of the given n data points living in R^d.
  KmTree(int n, int d, Scalar *points);
  ~KmTree();

  // Given k cluster centers, this runs a full k-means iterations, choosing the next set of
  // centers and returning the cost function for this set of centers. If assignment is not null,
  // it should be an array of size n that will be filled with the index of the cluster (0 - k-1)
  // that each data point is assigned to. The new center values will overwrite the old ones.
  Scalar DoKMeansStep(int k, Scalar *centers, int *assignment) const;

  // Choose k initial centers for k-means using the kmeans++ seeding procedure. The resulting
  // centers are returned via the centers variable, which should be pre-allocated to size k*d.
  // The cost of the initial clustering is returned.
  Scalar SeedKMeansPlusPlus(int k, Scalar *centers) const;

 private:
  struct Node {
    int num_points;                 // Number of points stored in this node
    int first_point_index;          // The smallest point index stored in this node
    Scalar *median, *radius;        // Bounding box center and half side-lengths
    Scalar *sum;                    // Sum of the points stored in this node
    Scalar opt_cost;                // Min cost for putting all points in this node in 1 cluster
    Node *lower_node, *upper_node;  // Child nodes
    mutable int kmpp_cluster_index; // The cluster these points are assigned to or -1 if variable
  };

  // Helper functions for constructor
  Node *BuildNodes(Scalar *points, int first_index, int last_index, char **next_node_data);
  Scalar GetNodeCost(const Node *node, Scalar *center) const;

  // Helper functions for DoKMeans step
  Scalar DoKMeansStepAtNode(const Node *node, int k, int *candidates, Scalar *centers,
                            Scalar *sums, int *counts, int *assignment) const;
  bool ShouldBePruned(Scalar *box_median, Scalar *box_radius, Scalar *centers, int best_index,
                      int test_index) const;

  // Helper functions for SeedKMeansPlusPlus
  void SeedKmppSetClusterIndex(const Node *node, int index) const;
  Scalar SeedKmppUpdateAssignment(const Node *node, int new_cluster, Scalar *centers,
                                  Scalar *dist_sq) const;

  int n_, d_;
  Scalar *points_;
  Node *top_node_;
  char *node_data_;
  int *point_indices_;
};

#endif
