/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */
#ifndef CREGISTRATIONDECIDEROROPTIMIZER_H
#define CREGISTRATIONDECIDEROROPTIMIZER_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/graphslam/misc/CWindowManager.h>

#include <string>
#include <map>


namespace mrpt { namespace graphslam {

/**\brief Interface for implementing node/edge registration deciders or
 * optimizer classes.
 *
 * Specific interfaces - for implementing node/edge deciders / optimizers -
 * can inherit from CRegistrationDeciderOrOptimizer so that they can make use
 * of the generic methods defined in the latter.
 *
 * \note \b Convention: For the already implemented deciders/optimizers the
 * following naming convention has been used:
 * - NRD: Node Registration Decider class
 * - ERD: Edge Registration Decider class
 * - GSO: GraphSlam Optimizer class
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CRegistrationDeciderOrOptimizer : public mrpt::utils::COutputLogger {
	public:

		CRegistrationDeciderOrOptimizer():
			m_win_manager(NULL) {
		}
		/**\brief Generic method for fetching the incremental action-observations (or
		 * observation-only) measurements
		 *
		 * \return True if operation was successful. Criteria for Success depend on
		 * the decider/optimizer implementing this method
		 */
		virtual bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;

		/**\brief Fetch the graph on which the decider/optimizer will work on. */
		virtual void setGraphPtr(GRAPH_t* graph) {}
		/**\brief Fetch a CWindowManager pointer.
		 *
		 * CWindowManager instance should contain a CDisplayWindow3D* and,
		 * optionally, a CWindowObserver pointer so that interaction with the
		 * window is possible
		 */
		virtual void setWindowManagerPtr(mrpt::graphslam::CWindowManager* win_manager) {
			ASSERT_(win_manager);

			m_win_manager = win_manager;
		}
		/**\brief Fetch a mrpt::synch::CCriticalSection for locking the GRAPH_t resource.
		 *
		 * Handy for realising multithreading in the derived classes.
		 *
		 * \warning Beware that prior to the decider/optimizer public method call,
		 * the CCriticalSection will already be locked from CGraphSlamEngine_t
		 * instance, but this isn't effective in multithreaded implementations
		 * where the decider/optimizer itself has to lock the function at which the extra
		 * thread runs.
		 */
		virtual void setCriticalSectionPtr(
				mrpt::synch::CCriticalSection* graph_section) { }
		/**\brief Initialize visual objects in CDisplayWindow (e.g. \em add an
		 * object to scene).
		 *
		 * \exception std::exception If the method is called without having first
		 * provided a CDisplayWindow3D* to the class instance
		 *
		 * \sa setWindowManagerPtr, updateVisuals
		 */
		virtual void initializeVisuals() {
			ASSERT_(m_win_manager);
		}
		/**\brief Update the relevant visual features in CDisplayWindow.
		 *
		 *\exception std::exception If the method is called without having first
		 * provided a CDisplayWindow3D* to the class instance
		 *
		 * \sa setWindowManagerPtr, initializeVisuals
		 */
		virtual void updateVisuals() {
			ASSERT_(m_win_manager);
		
		}
		/**\brief Get a list of the window events that happened since the last call.
		 *
		 * Method in derived classes is automatically called from the
		 * CGraphSlamEngine_t instance. After that, decider/optimizer should just
		 * fetch the parameters that it is interested in.
		 */
		virtual void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred) {
			ASSERT_(m_win_manager);
		}
		/**\brief Load the necessary for the decider/optimizer configuration parameters.
		 */
		virtual void loadParams(const std::string& source_fname) {}
		/**\brief Print the problem parameters - relevant to the decider/optimizer to the
		 * screen in a unified/compact way.
		 */
		virtual void printParams() const {}
		/**\brief Fill the provided string with a detailed report of the
		 * decider/optimizer state.
		 *
		 * Report should include (part of) the following:
		 * - Timing of important methods
		 * - Properties fo class at the current time
		 * - Logging of commands until current time
		 */
		virtual void getDescriptiveReport(std::string* report_str) const {}

	protected:
		/**\brief Check if the current decider/optimizer is inappropriate for the given
		 * dataset / provided observations
		 */
		virtual void checkIfInvalidDataset(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) {}


		mrpt::graphslam::CWindowManager* m_win_manager;

};

} } // end of namespaces

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_H */
