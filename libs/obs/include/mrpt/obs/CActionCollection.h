/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt::obs
{
/** Declares a class for storing a collection of robot actions. It is used in
 * mrpt::obs::CRawlog,
 *    for logs storage and particle filter based simulations.
 *
 * \sa CAction, CRawlog
 * \ingroup mrpt_obs_grp
 */
class CActionCollection : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CActionCollection)

   protected:
	/** The robot "actionss" */
	std::deque<mrpt::containers::deepcopy_poly_ptr<CAction::Ptr>> m_actions;

   public:
	/** ctor */
	CActionCollection() = default;
	/** Constructor from a single action. */
	CActionCollection(CAction& a);

	/** You can use CActionCollection::begin to get a iterator to the first
	 * element.
	 */
	using iterator =
		std::deque<mrpt::containers::deepcopy_poly_ptr<CAction::Ptr>>::iterator;

	/** You can use CActionCollection::begin to get a iterator to the first
	 * element.
	 */
	using const_iterator = std::deque<
		mrpt::containers::deepcopy_poly_ptr<CAction::Ptr>>::const_iterator;

	/** Returns a iterator to the first action: this is an example of usage:
	 * \code
	 *   CActionCollection  acts;
	 *   ...
	 *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
	 *	  {
	 *      (*it)->... // (*it) is a "CAction::Ptr"
	 *   }
	 *
	 * \endcode
	 */
	const_iterator begin() const { return m_actions.begin(); }
	/** Returns a iterator to the first action: this is an example of usage:
	 * \code
	 *   CActionCollection  acts;
	 *   ...
	 *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
	 *	  {
	 *      (*it)->... // (*it) is a "CAction::Ptr"
	 *   }
	 *
	 * \endcode
	 */
	iterator begin() { return m_actions.begin(); }
	/** Returns a iterator pointing to the end of the list: this is an example
	 *of usage:
	 * \code
	 *   CActionCollection  acts;
	 *   ...
	 *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
	 *	  {
	 *      (*it)->... // (*it) is a "CAction::Ptr"
	 *   }
	 *
	 * \endcode
	 */
	const_iterator end() const { return m_actions.end(); }
	/** Returns a iterator pointing to the end of the list: this is an example
	 *of usage:
	 * \code
	 *   CActionCollection  acts;
	 *   ...
	 *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
	 *	  {
	 *      (*it)->... // (*it) is a "CAction::Ptr"
	 *   }
	 *
	 * \endcode
	 */
	iterator end() { return m_actions.end(); }
	/** Removes the given action in the list, and return an iterator to the next
	 * element (or this->end() if it was the last one).
	 */
	iterator erase(const iterator& it);

	/** Erase all actions from the list.
	 */
	void clear();

	/** Access the i'th action.DO NOT MODIFY the returned object, make a copy of
	 * ir with "CSerializable::duplicate" if desired.
	 *  First element is 0.
	 * \exception std::exception On index out of bounds.
	 */
	CAction::Ptr get(size_t index);
	const CAction& get(size_t index) const;

	/** Access to the i'th action of a given class, or a nullptr smart pointer
	  if there is no action of that class in the list.
	  *  Example:
	  * \code
		   CActionRobotMovement2D::Ptr obs =
	  acts->getActionByClass<CActionRobotMovement2D>();
	  * \endcode
	  * By default (ith=0), the first one is returned.
	  */
	template <typename T>
	typename T::Ptr getActionByClass(const size_t& ith = 0) const
	{
		MRPT_START
		size_t foundCount = 0;
		const mrpt::rtti::TRuntimeClassId* class_ID =
			&T::GetRuntimeClassIdStatic();
		for (const auto& it : *this)
			if (it->GetRuntimeClass()->derivedFrom(class_ID))
				if (foundCount++ == ith)
					return std::dynamic_pointer_cast<T>(it.get_ptr());
		return typename T::Ptr();  // Not found: return empty smart pointer
		MRPT_END
	}

	/** Add a new object to the list.
	 */
	void insert(CAction& action);

	/** Returns the actions count in the collection.
	 */
	size_t size();

	/** Returns the best pose increment estimator in the collection, based on
	 * the determinant of its pose change covariance matrix.
	 * \return The estimation, or nullptr if none is available.
	 */
	CActionRobotMovement2D::Ptr getBestMovementEstimation() const;

	/** Returns the pose increment estimator in the collection having the
	 * specified type.
	 * \return The estimation, or nullptr if none is available.
	 */
	CActionRobotMovement2D::Ptr getMovementEstimationByType(
		CActionRobotMovement2D::TEstimationMethod method);

	/** Look for the first 2D or 3D "odometry" found in this collection of
	 * actions, and return the "mean" increment of the robot according to it.
	 * \return true on success,false on no odometry found.
	 */
	bool getFirstMovementEstimationMean(
		mrpt::poses::CPose3D& out_pose_increment) const;

	/** Look for the first 2D or 3D "odometry" found in this collection of
	 * actions, and return the "mean" increment of the robot and its covariance
	 * according to it.
	 * \return true on success,false on no odometry found.
	 */
	bool getFirstMovementEstimation(
		mrpt::poses::CPose3DPDFGaussian& out_pose_increment) const;

	/** Remove an action from the list by its index.
	 * \exception std::exception On index out of bounds.
	 */
	void eraseByIndex(const size_t& index);

};  // End of class def.

}  // namespace mrpt::obs
