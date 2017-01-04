/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
// See KMeans.h
//
// Author: David Arthur (darthur@gmail.com), 2009

// Includes
#include "KMeans.h"
#include "KmTree.h"
#include <mrpt/utils/mrpt_macros.h>
#include <sstream>
#include <time.h>
#include <vector>
using namespace std;

// Logging
static vector<ostream*> gLogOutputs;
static vector<ostream*> gVerboseLogOutputs;
#define LOG(verbose, text) {                                               \
  vector<ostream*> &outputs = (verbose? gVerboseLogOutputs : gLogOutputs); \
  if (outputs.size() > 0) {                                                \
    ostringstream string_stream;                                           \
    string_stream << text;                                                 \
    for (int i = 0; i < (int)outputs.size(); i++)                          \
      *(outputs[i]) << string_stream.str();                                \
  }                                                                        \
}
void AddKMeansLogging(std::ostream *out, bool verbose) {
  if (verbose)
    gVerboseLogOutputs.push_back(out);
  gLogOutputs.push_back(out);
}
void ClearKMeansLogging() {
  gLogOutputs.clear();
  gVerboseLogOutputs.clear();
}

// Returns the number of seconds since the program began execution.
static double GetSeconds() {
  return double(clock()) / CLOCKS_PER_SEC;
}

// See KMeans.h
// Performs one full execution of k-means, logging any relevant information, and tracking meta
// statistics for the run. If min or max values are negative, they are treated as unset.
// best_centers and best_assignment can be 0, in which case they are not set.
static void RunKMeansOnce(const KmTree &tree, int n, int k, int d, Scalar *points, Scalar *centers,
                          Scalar *min_cost, Scalar *max_cost, Scalar *total_cost,
                          double start_time, double *min_time, double *max_time,
                          double *total_time, Scalar *best_centers, int *best_assignment) {
  MRPT_UNUSED_PARAM(n); MRPT_UNUSED_PARAM(points);
  const Scalar kEpsilon = Scalar(1e-8);  // Used to determine when to terminate k-means

  // Do iterations of k-means until the cost stabilizes
  Scalar old_cost = 0;
  bool is_done = false;
  for (int iteration = 0; !is_done; iteration++) {
    Scalar new_cost = tree.DoKMeansStep(k, centers, 0);
    is_done = (iteration > 0 && new_cost >= (1 - kEpsilon) * old_cost);
    old_cost = new_cost;
    LOG(true, "Completed iteration #" << (iteration+1) << ", cost=" << new_cost << "..." << endl);
  }
  double this_time = GetSeconds() - start_time;

  // Log the clustering we found
  LOG(false, "Completed run: cost=" << old_cost << " (" << this_time << " seconds)" << endl);

  // Handle a new min cost, updating best_centers and best_assignment as appropriate
  if (*min_cost < 0 || old_cost < *min_cost) {
    *min_cost = old_cost;
    if (best_assignment != 0)
      tree.DoKMeansStep(k, centers, best_assignment);
    if (best_centers != 0)
      memcpy(best_centers, centers, sizeof(Scalar)*k*d);
  }

  // Update all other aggregate stats
  if (*max_cost < old_cost) *max_cost = old_cost;
  *total_cost += old_cost;
  if (*min_time < 0 || *min_time > this_time)
    *min_time = this_time;
  if (*max_time < this_time) *max_time = this_time;
  *total_time += this_time;
}

// Outputs all meta-stats for a set of k-means or k-means++ runs.
void LogMetaStats(Scalar min_cost, Scalar max_cost, Scalar total_cost,
                  double min_time, double max_time, double total_time, int num_attempts) {
  LOG(false, "Aggregate info over " << num_attempts << " runs:" << endl);
  LOG(false, "  Cost: min=" << min_cost << " average=" << (total_cost / num_attempts)
          << " max=" << max_cost << endl);
  LOG(false, "  Time: min=" << min_time << " average=" << (total_time / num_attempts)
          << " max=" << max_time << endl << endl);
}

// See KMeans.h
Scalar RunKMeans(int n, int k, int d, Scalar *points, int attempts,
                 Scalar *ret_centers, int *ret_assignment) {
  KM_ASSERT(k >= 1);
  
  // Create the tree and log
  LOG(false, "Running k-means..." << endl);
  KmTree tree(n, d, points);
  LOG(false, "Done preprocessing..." << endl);

  // Initialization
  Scalar *centers = (Scalar*)malloc(sizeof(Scalar)*k*d);
  int *unused_centers = (int*)malloc(sizeof(int)*n);
  KM_ASSERT(centers != 0 && unused_centers != 0);
  Scalar min_cost = -1, max_cost = -1, total_cost = 0;
  double min_time = -1, max_time = -1, total_time = 0;
  
  // Handle k > n
  if (k > n) {
    memset(centers + n*d, -1, (k-d)*sizeof(Scalar));
    k = n;
  }

  // Run all the attempts
  for (int attempt = 0; attempt < attempts; attempt++) {
    double start_time = GetSeconds();

    // Choose centers uniformly at random
    for (int i = 0; i < n; i++)
      unused_centers[i] = i;
    int num_unused_centers = n;
    for (int i = 0; i < k; i++) {
      int j = GetRandom(num_unused_centers--);
      memcpy(centers + i*d, points + unused_centers[j]*d, d*sizeof(Scalar));
      unused_centers[j] = unused_centers[num_unused_centers];
    }
    
    // Run k-means
    RunKMeansOnce(tree, n, k, d, points, centers, &min_cost, &max_cost, &total_cost, start_time,
                  &min_time, &max_time, &total_time, ret_centers, ret_assignment);
  }
  LogMetaStats(min_cost, max_cost, total_cost, min_time, max_time, total_time, attempts);

  // Clean up and return
  free(unused_centers);
  free(centers);
  return min_cost;
}

// See KMeans.h
Scalar RunKMeansPlusPlus(int n, int k, int d, Scalar *points, int attempts,
                         Scalar *ret_centers, int *ret_assignment) {
  KM_ASSERT(k >= 1);

  // Create the tree and log
  LOG(false, "Running k-means++..." << endl);
  KmTree tree(n, d, points);
  LOG(false, "Done preprocessing..." << endl);

  // Initialization
  Scalar *centers = (Scalar*)malloc(sizeof(Scalar)*k*d);
  KM_ASSERT(centers != 0);
  Scalar min_cost = -1, max_cost = -1, total_cost = 0;
  double min_time = -1, max_time = -1, total_time = 0;

  // Run all the attempts
  for (int attempt = 0; attempt < attempts; attempt++) {
    double start_time = GetSeconds();

    // Choose centers using k-means++ seeding
    tree.SeedKMeansPlusPlus(k, centers);
    
    // Run k-means
    RunKMeansOnce(tree, n, k, d, points, centers, &min_cost, &max_cost, &total_cost, start_time,
                  &min_time, &max_time, &total_time, ret_centers, ret_assignment);
  }
  LogMetaStats(min_cost, max_cost, total_cost, min_time, max_time, total_time, attempts);

  // Clean up and return
  free(centers);
  return min_cost;
}
