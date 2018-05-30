/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CEMPTYNRD_H
#define CEMPTYNRD_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
/**\brief Empty Node Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyNRD
	: public mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>
{
	/**\brief Handy typedefs */
	/**\{*/
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;
	/**\}*/
   public:
	CEmptyNRD();
	~CEmptyNRD();

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation);
	global_pose_t getCurrentRobotPosEstimation() const;

   private:
	void registerNewNode();
};

//////////////////////////////////////////////////////////////////////////////

template <class GRAPH_T>
CEmptyNRD<GRAPH_T>::CEmptyNRD()
{
}
template <class GRAPH_T>
CEmptyNRD<GRAPH_T>::~CEmptyNRD()
{
}

template <class GRAPH_T>
bool CEmptyNRD<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	return false;
}

template <class GRAPH_T>
void CEmptyNRD<GRAPH_T>::registerNewNode()
{
}

template <class GRAPH_T>
typename GRAPH_T::global_pose_t
	CEmptyNRD<GRAPH_T>::getCurrentRobotPosEstimation() const
{
	return typename GRAPH_T::global_pose_t();
}
}  // end of namespaces

#endif /* end of include guard: CEMPTYNRD_H */


