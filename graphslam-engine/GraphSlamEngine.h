/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Sun May 22 12:48:25 EEST 2016, nickkouk
// General TODO list:
// TODO: Make class generic - so that it handles 3D datasets
// TODO: Add functionality to be able to use different optimizers
// TODO: Plot x^2 ,x^2/s, time, iteration in the viz. window

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/graphs.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now

#include <string>
#include <sstream>
#include <map>
#include <cerrno>
#include <cmath> // fabs function

#include "EdgeCounter.h"


using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::graphslam;

using namespace std;

bool verbose = true;
#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "

template <class GRAPH_t>
class GraphSlamEngine_t {
  public:

    typedef std::map<string, CFileOutputStream*> fstreams;
    typedef std::map<string, CFileOutputStream*>::iterator fstreams_it;
    typedef std::map<string, CFileOutputStream*>::const_iterator fstreams_cit;

    typedef typename GRAPH_t::constraint_t constraint_t;
    typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
    typedef CMatrixFixedNumeric<double,
            constraint_t::state_length, constraint_t::state_length> InfMat;


    // Ctors, Dtors
    //////////////////////////////////////////////////////////////
    GraphSlamEngine_t(const string& config_file,
        CDisplayWindow3D* win = NULL );
    ~GraphSlamEngine_t();

    // Public function definitions
    //////////////////////////////////////////////////////////////
    /**
     * Wrapper fun around the GRAPH_t corresponding method
     */
    void saveGraph() const {
      MRPT_START

      if (!m_has_read_config) {
        THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
      }
      string fname = m_output_dir_fname + "/" + m_save_graph_fname;
      saveGraph(fname);

      MRPT_END
    }
    /**
     * Wrapper fun around the GRAPH_t corresponding method
     */
    void saveGraph(const string& fname) const {
      MRPT_START

      m_graph.saveToTextFile(fname);
      VERBOSE_COUT << "Saved graph to text file: " << fname <<
        " successfully." << endl;

      MRPT_END
    }
    /** 
     * Read the configuration file specified and fill in the corresponding
     * class members
     */
   void readConfigFile(const string& fname);
    /**
     * Print the problem parameters (usually fetched from a configuration file)
     * to the console for verification
     *
     * \sa GraphSlamEngine_t::parseRawlogFile
     */
    void printProblemParams() const;
    /**
     * TODO - Make this a function template so that it can handle camera
     * images, laser scan files, etc.
     * Reads the file provided and builds the graph 
     */
    void parseRawlogFile();
    /**
     * GraphslamEngine_t::optmizeGraph
     *
     * Optimize the under-construction graph
     */
    inline void optimizeGraph(GRAPH_t graph) {
      MRPT_START

      cout << "Executing the graph optimization" << endl;

      // TODO: read these throught the .ini file
      // params for the optimization of the graph
      TParametersDouble  optimization_params;
      optimization_params["verbose"]  = 0;
      optimization_params["profiler"] = 0;
      optimization_params["max_iterations"] = 500;
      optimization_params["scale_hessian"] = 0.1;  // If <1, will "exagerate" the scale of the gradient and, normally, will converge much faster.
      optimization_params["tau"] = 1e-3;
      // e2: Lev-marq algorithm iteration stopping criterion #2: |delta_incr| < e2*(x_norm+e2)
      //optimization_params["e1"] = 1e-6;
      //optimization_params["e2"] = 1e-6;

      graphslam::TResultInfoSpaLevMarq  levmarq_info;

      // Execute the optimization
      optimize_graph_spa_levmarq(
          graph,
          levmarq_info,
          NULL,  // List of nodes to optimize. NULL -> all but the root node.
          optimization_params,
          NULL); // functor feedback
          //&optimization_feedback);

      MRPT_END
    }

    /**
     * GraphSlamEngine_t::visualizeGraph
     *
     * Called internally for updating the vizualization scene for the graph
     * building procedure
     */
    inline void visualizeGraph(const GRAPH_t& gr);
    /**
     * GRAPH_t getter function - return reference to own graph
     * Handy function for visualization, printing purposes
     */
    const GRAPH_t& getGraph() const { return m_graph; } 

  private:
    // Private function definitions
    //////////////////////////////////////////////////////////////

    //static void optimization_feedback(
        //const GRAPH_t& graph,
        //const size_t iter,
        //const size_t max_iter,
        //const double cur_sq_error )
    //{
      //m_log_sq_err_evolution.push_back(std::log(cur_sq_error));
      //if ((iter % 100)==0)
        //cout << "Progress: " << iter << " / " << max_iter << ", total sq err = " << cur_sq_error << endl;
    //}


    /**
     * General initialization method to call from the different Ctors
     */
    void initGraphSlamEngine();
    /**
     * Initialize (clean up and create new files) the output directory
     * Also provides cmd line arguements for the user to choose the desired
     * action.
     * \sa GraphSlamEngine_t::initResultsFile
     */
    void initOutputDir();
    /**
     * Method to automate the creation of the output result files
     * Open and write an introductory message using the provided fname
     */
    void initResultsFile(const string& fname);
    /**
     * GraphSlamEngine_t::getICPEdge
     *
     * Align the laser scans of the given poses and compute a constraint
     * between them. Return the goodness of the ICP operation and let the user
     * decide if they want to keep it as an edge or not
     */
    inline double getICPEdge(const TNodeID& from, const TNodeID& to, constraint_t *rel_edge );
    /**
     * assignTextMessageParameters
     *
     * Assign the next available offset_y and text_index for the textMessage under
     * construction. Then increment the respective current counters
     */
    inline void assignTextMessageParameters(double* offset_y, int* text_index) {
      *offset_y = m_curr_offset_y;
      m_curr_offset_y += kOffsetYStep;

      *text_index = m_curr_text_index;
      m_curr_text_index += kIndexTextStep;
    }

    // VARIABLES
    //////////////////////////////////////////////////////////////

    // the graph object to be built and optimized
    GRAPH_t m_graph;

    /**
     * Problem parameters.
     * Most are imported from a .ini config file
     * \sa GraphSlamEngine_t::readConfigFile
     */
    string  m_config_fname;

    string  m_rawlog_fname;
    string  m_output_dir_fname;
    bool    m_user_decides_about_output_dir;
    bool    m_do_debug;
    string  m_debug_fname;
    string  m_save_graph_fname;

    bool    m_do_pose_graph_only;
    string  m_optimizer;

    string  m_loop_closing_alg;
    double  m_loop_closing_min_nodeid_diff;

    string  m_decider_alg;
    double  m_distance_threshold;
    double  m_angle_threshold;

    bool    m_has_read_config;

    // use lookup table to hold the optimizer codes
    map<string, int> optimizer_codes; 

    /** 
     * FileStreams
     * variable that keeps track of the out fstreams so that they can be closed 
     * (if still open) in the class Dtor.
     */
    fstreams m_out_streams;

    // visualization objects
    CDisplayWindow3D* m_win;

    TParametersDouble m_optimized_graph_viz_params;
    bool m_visualize_optimized_graph;
    bool m_visualize_odometry_poses;

    /**
     * textMessage Parameters 
     *
     */
    string m_font_name;
    int m_font_size;

    // textMessage vertical text position
    const double kOffsetYStep;
    double m_curr_offset_y;
    double m_offset_y_graph;
    double m_offset_y_odometry;
    double m_offset_y_timestamp;

    // textMessage index
    const int kIndexTextStep;
    int m_curr_text_index;
    int m_text_index_graph;
    int m_text_index_odometry;
    int m_text_index_timestamp;

    // instance to keep track of all the edges + visualization related
    // functions
    EdgeCounter_t m_edge_counter;

    // std::maps to store information about the graph(s)
    map<const GRAPH_t*, string> graph_to_name;
    map<const GRAPH_t*, TParametersDouble*> graph_to_viz_params;

    // odometry visualization
    vector<pose_t*> m_odometry_poses;

    bool m_is3D;
    TNodeID m_nodeID_max;

    // ICP configuration
    float m_ICP_goodness_thres;
    int m_prev_nodes_for_ICP; // add ICP constraints with m_prev_nodes_for_ICP nodes back
    map<TNodeID, CObservation2DRangeScanPtr> m_nodes_to_laser_scans;
    CICP m_ICP;

    // graph optimization

    // Container to handle the propagation of the square root error of the problem
    vector<double> m_log_sq_err_evolution;

    pose_t m_curr_estimated_pose;

};

// pseudo-split the definition and implementation of template
#include "GraphSlamEngine.cpp"

#endif /* end of include guard: GRAPHSLAMENGINE_H */
