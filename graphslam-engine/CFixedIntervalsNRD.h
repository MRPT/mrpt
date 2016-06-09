#ifndef CFIXEDINTERVALSNRD_H
#define CFIXEDINTERVALSNRD_H



#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/obs/CRawlog.h>

#include "CNodeRegistrationDecider.h"

#include <iostream>

// TODO - change these
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::math;

using namespace std;


/**
 * Fixed intervals odometry edge insertion
 * Determine whether to insert a new pose in the graph given the distance and
 * angle thresholds
 *
 * Current Decider is a minimal, simple implementation of the
 * CNodeRegistrationDecider_t interface which can be used for 2D datasets
 */

template<class GRAPH_t>
class CFixedIntervalsNRD_t: public CNodeRegistrationDecider_t<GRAPH_t> {
	public:
		// Public functions
		//////////////////////////////////////////////////////////////

		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length, 
						constraint_t::state_length> InfMat;

		CFixedIntervalsNRD_t(GRAPH_t* graph);
		CFixedIntervalsNRD_t();
		/**
		 * Initialization function to be called from the various constructors
		 */
		void initCFixedIntervalsNRD_t();
		~CFixedIntervalsNRD_t();

		/**
		 * Initialize the graph to be used for the node registration procedure
		 */
		void getGraphPtr(GRAPH_t* graph);

		/**
		 * Make use of the CActionCollection to update the odometry estimation from
		 * the last inserted pose
		 */
		virtual bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ); 

    struct TParams: public mrpt::utils::CLoadableOptions {
    	public:
    		TParams();
    		~TParams();

    		void loadFromConfigFile(
    				const mrpt::utils::CConfigFileBase &source,
    				const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				// max values for new node registration
				double registration_max_distance;
				double registration_max_angle;
    };

		// Public members
		// ////////////////////////////
    TParams params;

	private:
		// Private functions
		//////////////////////////////////////////////////////////////
		/**
		 * If estimated position surpasses the registration max values since the
		 * previous registered node, register a new node in the graph.
		 *
		 * Returns new on successful registration.
		 */
		bool checkRegistrationCondition();
		bool registerNewNode();
		
		// Private members
		//////////////////////////////////////////////////////////////

		GRAPH_t* m_graph;
		bool m_initialized_graph;
		// store the last registered node - not his pose since it will most likely
		// change during calls to the graph-optimization procedure /
		// dijkstra_node_estimation
		TNodeID m_prev_registered_node;

		constraint_t	m_since_prev_node_PDF;
	  InfMat m_init_path_uncertainty;

		pose_t m_curr_estimated_pose;
};

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::CFixedIntervalsNRD_t(GRAPH_t* graph):
	m_graph(graph)
{
	this->initCFixedIntervalsNRD_t();
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::CFixedIntervalsNRD_t() {
	this->initCFixedIntervalsNRD_t();
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::initCFixedIntervalsNRD_t() {
	m_initialized_graph = false;
	//
	// Tracking the PDF of the current position of the robot with regards to the
	// PREVIOUS registered node
	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	InfMat m_init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = m_init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();
	
	std::cout << "CFixedIntervalsNRD: Initialized class object" << std::endl;
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::~CFixedIntervalsNRD_t() {
}

// Member function implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {

	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	if (observation.present()) { // FORMAT #2
		// TODO - implement this
	}
	else { // FORMAT #1
		// TODO - add input validation for the CActionRobotMovement2D -
		// GetRuntimeClass?
		mrpt::obs::CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
		mrpt::poses::CPosePDFPtr increment = robot_move->poseChange;
		pose_t increment_pose = increment->getMeanVal();
		InfMat increment_inf_mat;
		increment->getInformationMatrix(increment_inf_mat);

		// update the relative PDF of the path since the LAST node was inserted
		constraint_t incremental_constraint(increment_pose, increment_inf_mat);
		m_since_prev_node_PDF += incremental_constraint;

	}
	m_curr_estimated_pose = m_graph->nodes[m_prev_registered_node] + 
		m_since_prev_node_PDF.getMeanVal();

	bool registered = this->checkRegistrationCondition();
	return registered;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::checkRegistrationCondition() {
	bool registered = false;

	pose_t last_pose_inserted = m_graph->nodes[m_prev_registered_node];

	// odometry criterion
	if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose) 
				> params.registration_max_distance) ||
			(fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi())) 
			 > params.registration_max_angle ) ) {

		// register the new node
		registered = this->registerNewNode();
	}

	return registered;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::registerNewNode() {
	bool registered = true; // By default it should be able to register the node
	mrpt::utils::TNodeID from = m_prev_registered_node;
	mrpt::utils::TNodeID to = ++m_prev_registered_node;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	// reset the relative PDF
	m_since_prev_node_PDF.cov_inv = m_init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	return registered;
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::getGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_prev_registered_node = m_graph->nodeCount()-1;

	m_initialized_graph = true;
	std::cout << "CFixedIntervalsNRD: Initialized the graph successfully" << std::endl;
}

// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::TParams::TParams() {
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::TParams::~TParams() {
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	double max_angle_deg = RAD2DEG(registration_max_angle);

	out.printf("------------------[ Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n", registration_max_distance);
	out.printf("Max angle for registration    = %.2f deg\n", max_angle_deg);
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			60 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

}

#endif /* end of include guard: CFIXEDINTERVALSNRD_H */
