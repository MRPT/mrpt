/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// See KmTree.cpp
//
// Author: David Arthur (darthur@gmail.com), 2009

// Includes
#include "KmTree.h"
#include <iostream>
#include <stdlib.h>
using namespace std;

KmTree::KmTree(int n, int d, Scalar *points): n_(n), d_(d), points_(points) {
  // Initialize memory
  int node_size = sizeof(Node) + d_ * 3 * sizeof(Scalar);
  node_data_ = (char*)malloc((2*n-1) * node_size);
  point_indices_ = (int*)malloc(n * sizeof(int));
  for (int i = 0; i < n; i++)
    point_indices_[i] = i;
  KM_ASSERT(node_data_ != 0 && point_indices_ != 0);

  // Calculate the bounding box for the points
  Scalar *bound_v1 = PointAllocate(d_);
  Scalar *bound_v2 = PointAllocate(d_);
  KM_ASSERT(bound_v1 != 0 && bound_v2 != 0);
  PointCopy(bound_v1, points, d_);
  PointCopy(bound_v2, points, d_);
  for (int i = 1; i < n; i++)
  for (int j = 0; j < d; j++) {
    if (bound_v1[j] > points[i*d_ + j]) bound_v1[j] = points[i*d_ + j];
    if (bound_v2[j] < points[i*d_ + j]) bound_v1[j] = points[i*d_ + j];
  }

  // Build the tree
  char *temp_node_data = node_data_;
  top_node_ = BuildNodes(points, 0, n-1, &temp_node_data);

  // Cleanup
  PointFree(bound_v1);
  PointFree(bound_v2);
}

KmTree::~KmTree() {
  free(point_indices_);
  free(node_data_);
}

Scalar KmTree::DoKMeansStep(int k, Scalar *centers, int *assignment) const {
  // Create an invalid center for comparison purposes
  Scalar *bad_center = PointAllocate(d_);
  KM_ASSERT(bad_center != 0);
  memset(bad_center, 0xff, d_ * sizeof(Scalar));

  // Allocate data
  Scalar *sums = (Scalar*)calloc(k * d_, sizeof(Scalar));
  int *counts = (int*)calloc(k, sizeof(int));
  int num_candidates = 0;
  int *candidates = (int*)malloc(k * sizeof(int));
  KM_ASSERT(sums != 0 && counts != 0 && candidates != 0);
  for (int i = 0; i < k; i++)
  if (memcmp(centers + i*d_, bad_center, d_ * sizeof(Scalar)) != 0)
    candidates[num_candidates++] = i;

  // Find nodes
  Scalar result = DoKMeansStepAtNode(top_node_, num_candidates, candidates, centers, sums,
                                     counts, assignment);

  // Set the new centers
  for (int i = 0; i < k; i++) {
    if (counts[i] > 0) {
      PointScale(sums + i*d_, Scalar(1) / counts[i], d_);
      PointCopy(centers + i*d_, sums + i*d_, d_);
    } else {
      memcpy(centers + i*d_, bad_center, d_ * sizeof(Scalar));
    }
  }

  // Cleanup memory
  PointFree(bad_center);
  free(candidates);
  free(counts);
  free(sums);
  return result;
}

// Helper functions for constructor
// ================================

// Build a kd tree from the given set of points
KmTree::Node *KmTree::BuildNodes(Scalar *points, int first_index, int last_index,
                                 char **next_node_data) {
  // Allocate the node
  Node *node = (Node*)(*next_node_data);
  (*next_node_data) += sizeof(Node);
  node->sum = (Scalar*)(*next_node_data);
  (*next_node_data) += sizeof(Scalar) * d_;
  node->median = (Scalar*)(*next_node_data);
  (*next_node_data) += sizeof(Scalar) * d_;
  node->radius = (Scalar*)(*next_node_data);
  (*next_node_data) += sizeof(Scalar) * d_;

  // Fill in basic info
  node->num_points = (last_index - first_index + 1);
  node->first_point_index = first_index;

  // Calculate the bounding box
  Scalar *first_point = points + point_indices_[first_index] * d_;
  Scalar *bound_p1 = PointAllocate(d_);
  Scalar *bound_p2 = PointAllocate(d_);
  KM_ASSERT(bound_p1 != 0 && bound_p2 != 0);
  PointCopy(bound_p1, first_point, d_);
  PointCopy(bound_p2, first_point, d_);
  for (int i = first_index+1; i <= last_index; i++)
  for (int j = 0; j < d_; j++) {
    Scalar c = points[point_indices_[i]*d_ + j];
    if (bound_p1[j] > c) bound_p1[j] = c;
    if (bound_p2[j] < c) bound_p2[j] = c;
  }

  // Calculate bounding box stats and delete the bounding box memory
  Scalar max_radius = -1;
  int split_d = -1;
  for (int j = 0; j < d_; j++) {
    node->median[j] = (bound_p1[j] + bound_p2[j]) / 2;
    node->radius[j] = (bound_p2[j] - bound_p1[j]) / 2;
    if (node->radius[j] > max_radius) {
      max_radius = node->radius[j];
      split_d = j;
    }
  }
  PointFree(bound_p2);
  PointFree(bound_p1);

  // If the max spread is 0, make this a leaf node
  if (max_radius == 0) {
    node->lower_node = node->upper_node = 0;
    PointCopy(node->sum, first_point, d_);
    if (last_index != first_index)
      PointScale(node->sum, Scalar(last_index - first_index + 1), d_);
    node->opt_cost = 0;
    return node;
  }

  // Partition the points around the midpoint in this dimension. The partitioning is done in-place
  // by iterating from left-to-right and right-to-left in the same way that partioning is done for
  // quicksort.
  Scalar split_pos = node->median[split_d];
  int i1 = first_index, i2 = last_index, size1 = 0;
  while (i1 <= i2) {
    bool is_i1_good = (points[point_indices_[i1]*d_ + split_d] < split_pos);
    bool is_i2_good = (points[point_indices_[i2]*d_ + split_d] >= split_pos);
    if (!is_i1_good && !is_i2_good) {
      int temp = point_indices_[i1];
      point_indices_[i1] = point_indices_[i2];
      point_indices_[i2] = temp;
      is_i1_good = is_i2_good = true;
    }
    if (is_i1_good) {
      i1++;
      size1++;
    }
    if (is_i2_good) {
      i2--;
    }
  }

  // Create the child nodes
  KM_ASSERT(size1 >= 1 && size1 <= last_index - first_index);
  node->lower_node = BuildNodes(points, first_index, first_index + size1 - 1, next_node_data);
  node->upper_node = BuildNodes(points, first_index + size1, last_index, next_node_data);

  // Calculate the new sum and opt cost
  PointCopy(node->sum, node->lower_node->sum, d_);
  PointAdd(node->sum, node->upper_node->sum, d_);
  Scalar *center = PointAllocate(d_);
  KM_ASSERT(center != 0);
  PointCopy(center, node->sum, d_);
  PointScale(center, Scalar(1) / node->num_points, d_);
  node->opt_cost = GetNodeCost(node->lower_node, center) + GetNodeCost(node->upper_node, center);
  PointFree(center);
  return node;
}

// Returns the total contribution of all points in the given kd-tree node, assuming they are all
// assigned to a center at the given location. We need to return:
//
//   sum_{x \in node} ||x - center||^2.
//
// If c denotes the center of mass of the points in this node and n denotes the number of points in
// it, then this quantity is given by
//
//   n * ||c - center||^2 + sum_{x \in node} ||x - c||^2
//
// The sum is precomputed for each node as opt_cost. This formula follows from expanding both sides
// as dot products. See Kanungo/Mount for more info.
Scalar KmTree::GetNodeCost(const Node *node, Scalar *center) const {
  Scalar dist_sq = 0;
  for (int i = 0; i < d_; i++) {
    Scalar x = (node->sum[i] / node->num_points) - center[i];
    dist_sq += x*x;
  }
  return node->opt_cost + node->num_points * dist_sq;
}

// Helper functions for DoKMeans step
// ==================================

// A recursive version of DoKMeansStep. This determines which clusters all points that are rooted
// node will be assigned to, and updates sums, counts and assignment (if not null) accordingly.
// candidates maintains the set of cluster indices which could possibly be the closest clusters
// for points in this subtree.
Scalar KmTree::DoKMeansStepAtNode(const Node *node, int k, int *candidates, Scalar *centers,
                                  Scalar *sums, int *counts, int *assignment) const {
  // Determine which center the node center is closest to
  Scalar min_dist_sq = PointDistSq(node->median, centers + candidates[0]*d_, d_);
  int closest_i = candidates[0];
  for (int i = 1; i < k; i++) {
    Scalar dist_sq = PointDistSq(node->median, centers + candidates[i]*d_, d_);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_i = candidates[i];
    }
  }

  // If this is a non-leaf node, recurse if necessary
  if (node->lower_node != 0) {
    // Build the new list of candidates
    int new_k = 0;
    int *new_candidates = (int*)malloc(k * sizeof(int));
    KM_ASSERT(new_candidates != 0);
    for (int i = 0; i < k; i++)
    if (!ShouldBePruned(node->median, node->radius, centers, closest_i, candidates[i]))
      new_candidates[new_k++] = candidates[i];

    // Recurse if there's at least two
    if (new_k > 1) {
      Scalar result = DoKMeansStepAtNode(node->lower_node, new_k, new_candidates, centers,
                                         sums, counts, assignment) +
                      DoKMeansStepAtNode(node->upper_node, new_k, new_candidates, centers,
                                         sums, counts, assignment);
      free(new_candidates);
      return result;
    } else {
      free(new_candidates);
    }
  }

  // Assigns all points within this node to a single center
  PointAdd(sums + closest_i*d_, node->sum, d_);
  counts[closest_i] += node->num_points;
  if (assignment != 0) {
    for (int i = node->first_point_index; i < node->first_point_index + node->num_points; i++)
      assignment[point_indices_[i]] = closest_i;
  }
  return GetNodeCost(node, centers + closest_i*d_);
}

// Determines whether every point in the box is closer to centers[best_index] than to
// centers[test_index].
//
// If x is a point, c_0 = centers[best_index], c = centers[test_index], then:
//       (x-c).(x-c) < (x-c_0).(x-c_0)
//   <=> (c-c_0).(c-c_0) < 2(x-c_0).(c-c_0)
//
// The right-hand side is maximized for a vertex of the box where for each dimension, we choose
// the low or high value based on the sign of x-c_0 in that dimension.
bool KmTree::ShouldBePruned(Scalar *box_median, Scalar *box_radius, Scalar *centers,
                            int best_index, int test_index) const {
  if (best_index == test_index)
    return false;
  
  Scalar *best = centers + best_index*d_;
  Scalar *test = centers + test_index*d_;
  Scalar lhs = 0, rhs = 0;
  for (int i = 0; i < d_; i++) {
    Scalar component = test[i] - best[i];
    lhs += component * component;
    if (component > 0)
      rhs += (box_median[i] + box_radius[i] - best[i]) * component;
    else
      rhs += (box_median[i] - box_radius[i] - best[i]) * component;
  }
  return (lhs >= 2*rhs);
}

Scalar KmTree::SeedKMeansPlusPlus(int k, Scalar *centers) const {
  Scalar *dist_sq = (Scalar*)malloc(n_ * sizeof(Scalar));
  KM_ASSERT(dist_sq != 0);

  // Choose an initial center uniformly at random
  SeedKmppSetClusterIndex(top_node_, 0);
  int i = GetRandom(n_);
  memcpy(centers, points_ + point_indices_[i]*d_, d_*sizeof(Scalar));
  Scalar total_cost = 0;
  for (int j = 0; j < n_; j++) {
    dist_sq[j] = PointDistSq(points_ + point_indices_[j]*d_, centers, d_);
    total_cost += dist_sq[j];
  }

  // Repeatedly choose more centers
  for (int new_cluster = 1; new_cluster < k; new_cluster++) {
    while (1) {
      Scalar cutoff = (rand() / Scalar(RAND_MAX)) * total_cost;
      Scalar cur_cost = 0;
      for (i = 0; i < n_; i++) {
        cur_cost += dist_sq[i];
        if (cur_cost >= cutoff)
          break;
      }
      if (i < n_)
        break;
    }
    memcpy(centers + new_cluster*d_, points_ + point_indices_[i]*d_, d_*sizeof(Scalar));
    total_cost = SeedKmppUpdateAssignment(top_node_, new_cluster, centers, dist_sq);
  }

  // Clean up and return
  free(dist_sq);
  return total_cost;
}

// Helper functions for SeedKMeansPlusPlus
// =======================================

// Sets kmpp_cluster_index to 0 for all nodes
void KmTree::SeedKmppSetClusterIndex(const Node *node, int value) const {
  node->kmpp_cluster_index = value;
  if (node->lower_node != 0) {
    SeedKmppSetClusterIndex(node->lower_node, value);
    SeedKmppSetClusterIndex(node->upper_node, value);
  }
}

Scalar KmTree::SeedKmppUpdateAssignment(const Node *node, int new_cluster, Scalar *centers,
                                        Scalar *dist_sq) const {
  // See if we can assign all points in this node to one cluster
  if (node->kmpp_cluster_index >= 0) {
    if (ShouldBePruned(node->median, node->radius, centers, node->kmpp_cluster_index, new_cluster))
      return GetNodeCost(node, centers + node->kmpp_cluster_index*d_);
    if (ShouldBePruned(node->median, node->radius, centers, new_cluster,
                       node->kmpp_cluster_index)) {
      SeedKmppSetClusterIndex(node, new_cluster);
      for (int i = node->first_point_index; i < node->first_point_index + node->num_points; i++)
        dist_sq[i] = PointDistSq(points_ + point_indices_[i]*d_, centers + new_cluster*d_, d_);
      return GetNodeCost(node, centers + new_cluster*d_);
    }
    
    // It may be that the a leaf-node point is equidistant from the new center or old
    if (node->lower_node == 0)
      return GetNodeCost(node, centers + node->kmpp_cluster_index*d_);
  }

  // Recurse
  Scalar cost = SeedKmppUpdateAssignment(node->lower_node, new_cluster, centers, dist_sq) +
                SeedKmppUpdateAssignment(node->upper_node, new_cluster, centers, dist_sq);
  int i1 = node->lower_node->kmpp_cluster_index, i2 = node->upper_node->kmpp_cluster_index;
  if (i1 == i2 && i1 != -1)
    node->kmpp_cluster_index = i1;
  else
    node->kmpp_cluster_index = -1;
  return cost;
}
