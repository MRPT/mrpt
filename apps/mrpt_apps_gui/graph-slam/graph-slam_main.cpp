/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

/*---------------------------------------------------------------
  APPLICATION: Graph-SLAM hub application
  PURPOSE: Visualization of .graph files and evaluation of
       Graph-SLAM methods.

  See the "--help" output for list of supported operations and
   further instructions, the man-page or online docs at:
     https://www.mrpt.org/list-of-mrpt-apps/application-graph-slam
 ---------------------------------------------------------------*/
#include <mrpt/graphs.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/system/datetime.h>

#include <CLI/CLI.hpp>

using namespace mrpt;
using namespace mrpt::graphslam;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

// Declarations:
#define VERBOSE_COUT \
  if (verbose) std::cout << "[graph-slam] "

#define DECLARE_OP_FUNCTION(_NAME)                                                         \
  void _NAME(                                                                              \
      const std::string& in_file, bool is3D, bool verbose, const std::string& output_file, \
      bool view, int max_iters, double initial_lambda, bool no_span)

#define IMPLEMENT_OP_FUNCTION(_NAME)                                                       \
  template <class GRAPHTYPE>                                                               \
  void _NAME##impl(                                                                        \
      const std::string& in_file, bool is3D, bool verbose, const std::string& output_file, \
      bool view, int max_iters, double initial_lambda, bool no_span);                      \
  void _NAME(                                                                              \
      const std::string& in_file, bool is3D, bool verbose, const std::string& output_file, \
      bool view, int max_iters, double initial_lambda, bool no_span)                       \
  {                                                                                        \
    if (!is3D)                                                                             \
      _NAME##impl<CNetworkOfPoses2D>(                                                      \
          in_file, is3D, verbose, output_file, view, max_iters, initial_lambda, no_span);  \
    else                                                                                   \
      _NAME##impl<CNetworkOfPoses3D>(                                                      \
          in_file, is3D, verbose, output_file, view, max_iters, initial_lambda, no_span);  \
  }                                                                                        \
  template <class GRAPHTYPE>                                                               \
  void _NAME##impl(                                                                        \
      const std::string& in_file, bool is3D, bool verbose, const std::string& output_file, \
      bool view, int max_iters, double initial_lambda, bool no_span)

/**
 * http://stackoverflow.com/questions/3982470/what-does-typedef-void-something-mean
 * Define a pointer-to-function type "TOperationFunctor". TOperationFunctor
 * returns void and takes parameters as explained
 */
using TOperationFunctor = void (*)(
    const std::string& in_file,
    bool is3D,
    bool verbose,
    const std::string& output_file,
    bool view,
    int max_iters,
    double initial_lambda,
    bool no_span);

/** Forward declaration.
 * display_graph function template provided in display_graph.cpp
 */
template <class GRAPHTYPE>
void display_graph(const GRAPHTYPE& g);

DECLARE_OP_FUNCTION(op_view);
DECLARE_OP_FUNCTION(op_dijkstra);
DECLARE_OP_FUNCTION(op_levmarq);
DECLARE_OP_FUNCTION(op_info);

// ======================================================================
//     main() of graph-slam
// ======================================================================
int main(int argc, char** argv)
{
  int ret_val = 0;

  try
  {
    CLI::App app("graph-slam");
    app.set_version_flag("--version", MRPT_getVersion());

    std::string input_file;
    app.add_option("-i,--input", input_file, "Input file (required) (*.graph,*.graphbin)")
        ->required();

    std::string output_file;
    app.add_option("-o,--output", output_file, "Output file (optional) (*.graph,*.graphbin)");

    bool use_2d = false;
    app.add_flag("--2d", use_2d, "Use 2D poses (Must use exactly one of --2d and --3d)");

    bool use_3d = false;
    app.add_flag("--3d", use_3d, "Use 3D poses (Must use exactly one of --2d and --3d)");

    bool view = false;
    app.add_flag(
        "", "view", view,
        "Op: Visualize the graph in a 3D view. If used alone, represent VERTEX2 or "
        "VERTEX3 poses directly as stored in the input file. "
        "If used together with another operation, the final obtained graph after "
        "the operation will be shown, not the input original one.\n");

    bool dijkstra_op = false;
    app.add_flag(
        "", "dijkstra", dijkstra_op,
        "Op: Executes CNetworkOfPoses::dijkstra_nodes_estimate() to "
        "estimate the global pose of nodes from a Dijkstra tree and "
        "the edge relative poses.\n"
        "   Can be used together with: --view, --output");

    bool levmarq_op = false;
    app.add_flag(
        "", "levmarq", levmarq_op,
        "Op: Optimizes the graph with sparse Levenberg-Marquartd using "
        "global coordinates (via "
        "mrpt::graphslam::optimize_graph_spa_levmarq).\n"
        "   Can be used together with: --view, --output, --max-iters, "
        "--no-span, --initial-lambda");

    bool info_op = false;
    app.add_flag(
        "", "info", info_op,
        "Op: Loads the graph and displays statistics and information on it.\n");

    bool quiet = false;
    app.add_flag("-q,--quiet", quiet, "Terse output");

    int max_iters = 100;
    app.add_option("--max-iters", max_iters, "Maximum number of iterations (optional)");

    double initial_lambda = 0;
    app.add_option(
        "--initial-lambda", initial_lambda, "Initial lambda parameter (optional, lev-marq)");

    bool no_span = false;
    app.add_flag("--no-span", no_span, "Don't use dijkstra initial spanning tree guess (optional)");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    // Exactly 1 of --2d & --3d must be specified:
    if ((use_2d && use_3d) || (!use_2d && !use_3d))
      throw std::runtime_error("Exactly one --2d or --3d must be used.");

    const bool is3d = use_3d;

    const bool verbose = !quiet;

    // List of possible operations
    map<string, TOperationFunctor> ops_functors;
    ops_functors["levmarq"] = &op_levmarq;
    ops_functors["dijkstra"] = &op_dijkstra;
    ops_functors["info"] = &op_info;

    // Check the selected operation:
    string selected_op;
    if (levmarq_op) selected_op = "levmarq";
    if (dijkstra_op)
    {
      if (!selected_op.empty())
        throw std::runtime_error("Exactly one operation must be indicated on command line.");
      selected_op = "dijkstra";
    }
    if (info_op)
    {
      if (!selected_op.empty())
        throw std::runtime_error("Exactly one operation must be indicated on command line.");
      selected_op = "info";
    }

    // The "--view" argument needs a bit special treatment:
    if (selected_op.empty())
    {
      if (!view)
        throw std::runtime_error(
            "Don't know what to do: No operation was indicated.\n"
            "Use --help to see the list of possible operations.");
      else
      {
        VERBOSE_COUT << "Operation to perform: "
                        "view"
                     << endl;
        op_view(input_file, is3d, verbose, output_file, view, max_iters, initial_lambda, no_span);
      }
    }
    else
    {
      VERBOSE_COUT << "Operation to perform: " << selected_op << endl;

      // ------------------------------------
      //  EXECUTE THE REQUESTED OPERATION
      // ------------------------------------
      ASSERTMSG_(
          ops_functors.find(selected_op) != ops_functors.end(),
          "Internal error: Unknown operation functor!");

      // Call the selected functor:
      ops_functors[selected_op](
          input_file, is3d, verbose, output_file, view, max_iters, initial_lambda, no_span);
    }

    // successful end of program.
    ret_val = 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    ret_val = -1;
  }

  // end:
  return ret_val;
}

// ===================================================================================
//     Implementation of the functions operations
// ===================================================================================

// -----------------------------------------------------------------------------------
//  op: --view
//   Load and visualize the graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_view)
{
  // Load:
  GRAPHTYPE g;
  g.loadFromTextFile(in_file);

  // Display:
  display_graph(g);
}

// -----------------------------------------------------------------------------------
//  op: --info
//   Load and visualize basic info on graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_info)
{
  // Load:
  GRAPHTYPE g;
  g.loadFromTextFile(in_file);

  set<TNodeID> lstNodeIDs;
  g.getAllNodes(lstNodeIDs);

  // Show graph stats:
  cout << "Edge count                         : " << g.edgeCount() << endl;
  cout << "Nodes count (in VERTEX2/3 entries) : " << g.nodes.size() << endl;
  cout << "Nodes count (in edge entries)      : " << lstNodeIDs.size() << endl;
}

// -----------------------------------------------------------------------------------
//  op: --dijkstra
//   Load and visualize the graph
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_dijkstra)
{
  const bool save_to_file = !output_file.empty();  // Output to file??
  const bool display_3D = view;                    // Output to 3D view??

  if (!save_to_file && !display_3D)
    std::cerr << "\n ** WARNING **: Neither --view nor --output specified. "
                 "The result will not be saved anywhere.\n\n";

  // Load:
  GRAPHTYPE g;
  g.loadFromTextFile(in_file);

  // Find the first node ID and use it as root:
  VERBOSE_COUT << "Making a list with all node IDs...\n";

  set<TNodeID> lstNodeIDs;
  g.getAllNodes(lstNodeIDs);

  ASSERT_(!lstNodeIDs.empty());
  const TNodeID id_root = *lstNodeIDs.begin();
  VERBOSE_COUT << "Using root node ID=" << id_root << endl;

  g.root = id_root;

  VERBOSE_COUT << "Executing Dijkstra...\n";
  CTicTac tictac;
  tictac.Tic();

  g.dijkstra_nodes_estimate();

  VERBOSE_COUT << "Took: " << mrpt::system::formatTimeInterval(tictac.Tac()) << endl;

  if (save_to_file)
  {
    VERBOSE_COUT << "Saving resulting graph to: " << output_file << endl;
    g.saveToTextFile(output_file);
  }
  if (display_3D)
  {
    VERBOSE_COUT << "Displaying resulting graph\n";
    display_graph(g);
  }
}  // end op_dijkstra

// -----------------------------------------------------------------------------------
//  op: --levmarq
//   Load and optimize a graph with the sparse Lev-Marq algorithm.
// -----------------------------------------------------------------------------------
IMPLEMENT_OP_FUNCTION(op_levmarq)
{
  const bool save_to_file = !output_file.empty();  // Output to file??
  const bool display_3D = view;                    // Output to 3D view??
  const bool skip_dijkstra = no_span;

  if (!save_to_file && !display_3D)
    std::cerr << "\n ** WARNING **: Neither --view nor --output specified. "
                 "The result will not be saved anywhere.\n\n";

  // Load:
  GRAPHTYPE g;
  g.loadFromTextFile(in_file);

  // Find the first node ID and use it as root:
  VERBOSE_COUT << "Making a list with all node IDs...\n";

  set<TNodeID> lstNodeIDs;
  g.getAllNodes(lstNodeIDs);

  ASSERT_(!lstNodeIDs.empty());
  const TNodeID id_root = *lstNodeIDs.begin();
  VERBOSE_COUT << "Using root node ID=" << id_root << endl;

  g.root = id_root;

  // Executes dijkstra (spanning tree) for initial guess?
  // --------------------------------
  if (!skip_dijkstra)
  {
    VERBOSE_COUT << "Executing Dijkstra...\n";
    CTicTac tictac;
    tictac.Tic();

    g.dijkstra_nodes_estimate();

    VERBOSE_COUT << "Took: " << mrpt::system::formatTimeInterval(tictac.Tac()) << endl;
  }

  // Executes the optimization:
  // --------------------------------
  mrpt::containers::yaml params;
  params["verbose"] = verbose;
  params["profiler"] = verbose;
  params["max_iterations"] = max_iters;
  params["initial_lambda"] = initial_lambda;

  graphslam::TResultInfoSpaLevMarq info;

  graphslam::optimize_graph_spa_levmarq(g, info, nullptr, params);

  // Output:
  if (save_to_file)
  {
    VERBOSE_COUT << "Saving resulting graph to: " << output_file << endl;
    g.saveToTextFile(output_file);
  }
  if (display_3D)
  {
    VERBOSE_COUT << "Displaying resulting graph\n";
    display_graph(g);
  }
}  // end op_levmarq
