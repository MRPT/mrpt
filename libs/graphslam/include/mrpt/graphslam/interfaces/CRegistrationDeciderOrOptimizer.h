/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/graphslam/misc/CWindowManager.h>

#include <string>
#include <map>

namespace mrpt::graphslam
{
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
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CRegistrationDeciderOrOptimizer : public mrpt::system::COutputLogger
{
   public:
	CRegistrationDeciderOrOptimizer() = default;
	~CRegistrationDeciderOrOptimizer() override = default;
	/**\brief Generic method for fetching the incremental action-observations
	 * (or
	 * observation-only) measurements
	 *
	 * \return True if operation was successful. Criteria for Success depend on
	 * the decider/optimizer implementing this method
	 */
	virtual bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) = 0;
	/**\brief Fetch a CWindowManager pointer.
	 *
	 * CWindowManager instance should contain a CDisplayWindow3D* and,
	 * optionally, a CWindowObserver pointer so that interaction with the
	 * window is possible
	 */
	virtual void setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager);

	/**\brief Fetch a std::mutex for locking the GRAPH_T
	 * resource.
	 *
	 * Handy for realising multithreading in the derived classes.
	 *
	 * \warning Beware that prior to the decider/optimizer public method call,
	 * the CCriticalSection will already be locked from CGraphSlamEngine_t
	 * instance, but this isn't effective in multithreaded implementations where
	 * the decider/optimizer itself has to lock the function at which the extra
	 * thread runs.
	 */
	virtual void setCriticalSectionPtr(std::mutex* graph_section);
	/**\brief Initialize visual objects in CDisplayWindow (e.g. \em add an
	 * object to scene).
	 *
	 * \exception std::exception If the method is called without having first
	 * provided a CDisplayWindow3D* to the class instance
	 *
	 * \sa setWindowManagerPtr, updateVisuals
	 */
	virtual void initializeVisuals();
	/**\brief Update the relevant visual features in CDisplayWindow.
	 *
	 *\exception std::exception If the method is called without having first
	 * provided a CDisplayWindow3D* to the class instance
	 *
	 * \sa setWindowManagerPtr, initializeVisuals
	 */
	virtual void updateVisuals();
	/**\brief Get a list of the window events that happened since the last call.
	 *
	 * Method in derived classes is automatically called from the
	 * CGraphSlamEngine_t instance. After that, decider/optimizer should just
	 * fetch the parameters that it is interested in.
	 */
	virtual void notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred);
	/**\brief Load the necessary for the decider/optimizer configuration
	 * parameters.
	 */
	virtual void loadParams(const std::string& source_fname);
	/**\brief Print the problem parameters - relevant to the decider/optimizer
	 * to the
	 * screen in a unified/compact way.
	 */
	virtual void printParams() const;
	/**\brief Fill the provided string with a detailed report of the
	 * decider/optimizer state.
	 *
	 * Report should include (part of) the following:
	 * - Timing of important methods
	 * - Properties fo class at the current time
	 * - Logging of commands until current time
	 */
	virtual void getDescriptiveReport(std::string* report_str) const;

	/**\brief Fetch the graph on which the decider/optimizer will work on.
	 *
	 */
	virtual void setGraphPtr(GRAPH_T* graph);

	/**\brief Initialize the COutputLogger, CTimeLogger instances given the
	 * name of the decider/optimizer at hand
	 */
	virtual void initializeLoggers(const std::string& name);
	virtual void setClassName(const std::string& name);
	bool isMultiRobotSlamClass();

	std::string getClassName() const { return m_class_name; };

   protected:
	/**\brief Handy function for making all the visuals assertions in a
	 * compact manner
	 */
	virtual void assertVisualsVars();
	/**\brief Pointer to the graph that is under construction */
	GRAPH_T* m_graph = nullptr;
	std::mutex* m_graph_section = nullptr;

	/** \name Visuals-related variables methods
	 */
	/**\{*/
	/**\brief Pointer to the CWindowManager object used to store
	 * visuals-related instances
	 */
	mrpt::graphslam::CWindowManager* m_win_manager = nullptr;
	/**\brief Window to use */
	mrpt::gui::CDisplayWindow3D* m_win = nullptr;
	/**\brief CWindowObserver object for monitoring various visual-oriented
	 * events.*/
	mrpt::graphslam::CWindowObserver* m_win_observer = nullptr;
	bool m_initialized_visuals = false;
	/**\}*/

	/**\brief Time logger instance */
	mrpt::system::CTimeLogger m_time_logger;
	/**\brief Name of the class instance */
	std::string m_class_name = "CRegistrationDeciderOrOptimizer";
	/**\brief Boolean indicating if the current class can be used in
	 * multi-robot SLAM operations
	 */
	bool is_mr_slam_class = false;

	/**\brief Separator string to be used in debugging messages
	 */
	static const std::string header_sep;
	static const std::string report_sep;
};
}  // namespace mrpt::graphslam
#include "CRegistrationDeciderOrOptimizer_impl.h"
