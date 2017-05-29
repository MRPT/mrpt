#ifndef CRANGESCANEDGEREGISTRATIONDECIDER_H
#define CRANGESCANEDGEREGISTRATIONDECIDER_H

#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/misc/CRangeScanOps.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>
#include <mrpt/graphslam/misc/TNodeProps.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/utils/mrpt_stdint.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Edge Registration Decider Interface from which RangeScanner-based
 * ERDs can inherit from.
 *
 * Holds common vars for the underlying classes and provides methods for
 * accessing and modifying them. Also takes care of common visualization needed
 * in derived classes
 *
 * \note Since the decider inherits from the CRangeScanOps
 * class, it parses the configuration parameters of the latter as well from the
 * "ICP" section. Refer to the CRangeScanOps documentation for
 * its list of configuration parameters
 */
template<class GRAPH_T>
class CRangeScanEdgeRegistrationDecider :
	public virtual CEdgeRegistrationDecider<GRAPH_T>,
	public virtual CRangeScanOps<GRAPH_T>
	{
	public:
		typedef mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>
			parent_t;
		/**\brief Typedef for accessing methods of the
		 * RangeScanRegistrationDecider_t parent class.
		 */
		typedef mrpt::graphslam::deciders::CRangeScanOps<GRAPH_T>
			range_ops_t;
		typedef std::map<
			mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr> nodes_to_scans2D_t;
		typedef std::map<
			mrpt::utils::TNodeID,
			mrpt::obs::CObservation3DRangeScanPtr> nodes_to_scans3D_t;
		typedef typename GRAPH_T::constraint_t constraint_t;
		typedef typename GRAPH_T::constraint_t::type_value pose_t;
		typedef typename GRAPH_T::global_pose_t global_pose_t;
		typedef mrpt::graphslam::detail::TNodeProps<GRAPH_T> node_props_t;

		CRangeScanEdgeRegistrationDecider();
		virtual ~CRangeScanEdgeRegistrationDecider();

	protected:
		/**\name Helper structs */
		/**\{*/
		/**
	 	* \brief Struct for passing additional parameters to the getICPEdge call
	 	*
	 	* Handy for overriding the search to the \a GRAPH_T::nodes map or the
	 	* search for the node's LaserScan
	 	*/
		struct TGetICPEdgeAdParams {
			typedef TGetICPEdgeAdParams self_t;

			node_props_t from_params; /**< Ad. params for the from_node */
			node_props_t to_params; /**< Ad. params for the to_node */
			pose_t init_estim; /**< Initial ICP estimation */

			void getAsString(std::string* str) const {
				using namespace std;
				using namespace mrpt;
				ASSERT_(str);
				str->clear();
				*str  += format("from_params: %s", from_params.getAsString().c_str());
				*str += format("to_params: %s", to_params.getAsString().c_str());
				*str += format("init_estim: %s\n", init_estim.asString().c_str());
			}
			std::string getAsString() const {
				std::string str;
				this->getAsString(&str);
				return str;
			}
			friend std::ostream& operator<<(std::ostream& o, const self_t& params) {
				o << params.getAsString() << endl;
				return o;
			}
		};

		/**\brief Get the ICP Edge between the provided nodes.
	 	 *
	 	 * Handy for not having to manually fetch the laser scans, as the method
	 	 * takes care of this.
	 	 *
	 	 * \param[out] icp_info Struct that will be filled with the results of the
	 	 * ICP operation
	 	 *
	 	 * \param[in] ad_params Pointer to additional parameters in the getICPEdge call
	 	 *
	 	 * \return True if operation was successful, false otherwise (e.g. if the
	 	 * either of the nodes' CObservation2DRangeScan object does not contain
	 	 * valid data.
	 	 */
		virtual bool getICPEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				constraint_t* rel_edge,
				mrpt::slam::CICP::TReturnInfo* icp_info=NULL,
				const TGetICPEdgeAdParams* ad_params=NULL
				);
		/**\brief Fill the TNodeProps instance using the parameters from the map
		 *
		 * \param[in] nodeID ID of node corresponding to the TNodeProps struct that
		 * is to be filled
		 * \param[in] group_params Map of TNodeID to corresponding TNodeProps
		 * instance.
		 * \param[out] node_props Pointer to the TNodeProps struct to be filled.
		 *
		 * 
		 * \return True if operation was successful, false otherwise.
		 */
		virtual bool fillNodePropsFromGroupParams(
				const mrpt::utils::TNodeID& nodeID,
				const std::map<mrpt::utils::TNodeID, node_props_t>& group_params,
					node_props_t* node_props);
		/**\brief Fill the pose and LaserScan for the given nodeID.
		 * Pose and LaserScan are either fetched from the TNodeProps struct, if it
		 * contains valid data, otherwise from the corresponding class vars
		 *
		 * \return True if operation was successful and pose, scan contain valid
		 * data.
		 */
		virtual bool getPropsOfNodeID(
				const mrpt::utils::TNodeID& nodeID,
				global_pose_t* pose,
				mrpt::obs::CObservation2DRangeScanPtr& scan,
				const node_props_t* node_props=NULL) const;

		/**\name Manipulation of observations manipulation.
		 *
		 * Methods for manipulating the underlying storage of the
		 * observation that the specific decider implementations.
		 * \note Defining a generic way of dealing with the underlying
		 * measurements, allows for flexible storage as well as possible
		 * modification to the method behavior.
		 */
		/**\{*/
		/** \brief Fetch the latest observation that the current instance
		 	* received (most probably during a call to the updateState method.
		 	*/
		// TODO - Implement these
		/**\}*/

		virtual void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred);

		/**\name LaserScan visualization variables */
		/**\{*/
		virtual void initializeVisuals();
		virtual void updateVisuals();
		virtual void initLaserScansVisualization();
		virtual void updateLaserScansVisualization();
		/**\brief Fetch a list of nodes with regards to the given nodeID for
		 	* which to try and add scan matching edges
		 	*
		 	* \sa addScanMatchingEdges
		 	*/
		virtual void fetchNodeIDsForScanMatching(
				const mrpt::utils::TNodeID& curr_nodeID,
				std::set<mrpt::utils::TNodeID>* nodes_set);
		/**\brief Addd ICP constraints from X previous nodeIDs up to the given
		 * nodeID.
		 *
		 * X is set by the user in the .ini configuration file (see
		 * prev_nodes_for_ICP)
		 *
		 * \sa fetchNodeIDsForScanMatching
		 */
		virtual void addScanMatchingEdges(const mrpt::utils::TNodeID& curr_nodeID);
		/**\brief togle the LaserScans visualization on and off
		*/
		virtual void toggleLaserScansVisualization();

		/**\}*/

		virtual void loadParams(const std::string& source_fname);
		virtual void printParams() const;
		/**brief Compare the suggested ICP edge against the initial node
		 * difference.
		 *
		 * If this difference is significantly larger than the rest of of the
		 * recorded mahalanobis distances, reject the suggested ICP edge.
		 *
		 * \return True if suggested ICP edge is accepted \note Method updates the
		 * Mahalanobis Distance TSlidingWindow which
		 * keep track of the recorded mahalanobis distance values.
		 * \sa getICPEdge
		 */
		virtual bool mahalanobisDistanceOdometryToICPEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge);


		/**\brief Map for keeping track of the observation recorded at each graph
		 * position
		 */
		/**\{*/
		nodes_to_scans2D_t  m_nodes_to_laser_scans2D;
		nodes_to_scans3D_t  m_nodes_to_laser_scans3D;
		/**\}*/
 		/**\brief Keep track of the total number of registered nodes since the last
 		 * time class method was called
 		 */
		size_t m_last_total_num_nodes;

		/**\brief ICP-related variables */
		/**\{*/
		/**\brief CICP instance that finds the correspondences */
		mrpt::slam::CICP m_icp;
 		/**\brief How many nodes back to check ICP against?
 		 	*
 		 	*/
		int m_prev_nodes_for_ICP;

		/**\brief Keep track of the mahalanobis distance between the initial pose
		 * difference and the suggested new edge for the pairs of checked
		 * nodes.
		 */
		TSlidingWindow m_mahal_distance_ICP_odom_win;
		/**\brief Keep track of ICP Goodness values for ICP between nearby nodes and
		* adapt the Goodness threshold based on the median of the recorded Goodness
		* values.
		*/
		TSlidingWindow m_goodness_threshold_win;
		/**\}*/

		/**\name LaserScans visualization parameters */
		/**\{*/
		/** Keystroke to be used by the user to toggle the LaserScans from the
		 * CDisplayWindow
		 */
		/**\brief Keep the last laser scan for visualization purposes */
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;
		/**\brief Keep the last 3D laser scan for visualization purposes */
		mrpt::obs::CObservation3DRangeScanPtr m_last_laser_scan3D;
		std::string m_keystroke_laser_scans;
		mrpt::utils::TColor m_laser_scans_color;
		bool m_visualize_laser_scans;
		std::string m_planar_laser_scan_obj_name;
		/**\}*/

		/**\brief Factor used for accepting an ICP Constraint as valid.
			*
			*/
		double m_consec_icp_constraint_factor;

	};

} } } // end of namespaces

#include "CRangeScanEdgeRegistrationDecider_impl.h"

#endif /* end of include guard: CRANGESCANEDGEREGISTRATIONDECIDER_H */
