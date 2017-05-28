#ifndef CRANGESCANEDGEREGISTRATIONDECIDER_H
#define CRANGESCANEDGEREGISTRATIONDECIDER_H

#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/misc/CRangeScanOps.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Edge Registration Decider Interface from which RangeScanner-based
 * ERDs can inherit from.
 *
 * Holds common vars for the underlying classesand provides methods for
 * accessing and modifying them.
 *
 * \note Since the decider inherits from the CRangeScanOps
 * class, it parses the configuration parameters of the latter as well from the
 * "ICP" section. Refer to the CRangeScanOps documentation for
 * its list of configuration parameters
 */
template<class GRAPH_T>
class CRangeScanEdgeRegistrationDecider :
	public virtual CEdgeRegistrationDecider<GRAPH_T>,
	public CRangeScanOps<GRAPH_T>
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

		CRangeScanEdgeRegistrationDecider();
		~CRangeScanEdgeRegistrationDecider();

	protected:
		/**\name Relevant-observations manipulation 
		 * Methods for manipulating the underlying storage of the
		 * observation that the specific decider implementations.
		 * \note Defining a generic way of dealing with the underlying
		 * measurements, allows for flexible storage as well as possible
		 * modification to the method behavior.
		 */
		/**\{ */
		/**\brief Fetch the latest observation that the current instance
		 * received (most probably during a call to the updateState method.
		 */
		// TODO - Implement these
		/**\} */

		virtual void loadParams(const std::string& source_fname);
		virtual void printParams() const;
		/**\brief Map for keeping track of the observation recorded at each graph
		 * position
		 */
		nodes_to_scans2D_t  m_nodes_to_laser_scans2D;
 		/**\brief Keep track of the total number of registered nodes since the last
 		 * time class method was called
 		 */
		size_t m_last_total_num_nodes;

	private:

};

} } } // end of namespaces

#include "CRangeScanEdgeRegistrationDecider_impl.h"

#endif /* end of include guard: CRANGESCANEDGEREGISTRATIONDECIDER_H */
