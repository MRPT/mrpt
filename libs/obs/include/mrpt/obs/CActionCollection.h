/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CActionCollection_H
#define CActionCollection_H

#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/poly_ptr_ptr.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt
{
	namespace obs
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CActionCollection, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a class for storing a collection of robot actions. It is used in mrpt::obs::CRawlog,
		 *    for logs storage and particle filter based simulations.
		 *
		 * \sa CAction, CRawlog
	 	 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP CActionCollection : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CActionCollection )

		protected:
			std::deque<mrpt::utils::poly_ptr_ptr<CActionPtr> >	m_actions;  //!< The robot "actionss"

		 public:
			CActionCollection(); //!< ctor
			CActionCollection( CAction &a ); //!< Constructor from a single action.

			/** You can use CActionCollection::begin to get a iterator to the first element.
			  */
			typedef std::deque<mrpt::utils::poly_ptr_ptr<CActionPtr> >::iterator		iterator;

			/** You can use CActionCollection::begin to get a iterator to the first element.
			  */
			typedef std::deque<mrpt::utils::poly_ptr_ptr<CActionPtr> >::const_iterator	const_iterator;

			/** Returns a iterator to the first action: this is an example of usage:
			  * \code
			  *   CActionCollection  acts;
			  *   ...
			  *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CActionPtr"
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
			  *      (*it)->... // (*it) is a "CActionPtr"
			  *   }
			  *
			  * \endcode
			  */
			iterator begin() { return m_actions.begin(); }

			/** Returns a iterator pointing to the end of the list: this is an example of usage:
			  * \code
			  *   CActionCollection  acts;
			  *   ...
			  *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CActionPtr"
			  *   }
			  *
			  * \endcode
			  */
			const_iterator end() const { return m_actions.end(); }

			/** Returns a iterator pointing to the end of the list: this is an example of usage:
			  * \code
			  *   CActionCollection  acts;
			  *   ...
			  *   for (CActionCollection::iterator it=acts.begin();it!=acts.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CActionPtr"
			  *   }
			  *
			  * \endcode
			  */
			iterator end() { return m_actions.end(); }


			/** Removes the given action in the list, and return an iterator to the next element (or this->end() if it was the last one).
			  */
			iterator erase( const iterator &it);

			/** Erase all actions from the list.
			  */
			void  clear();

			/** Access the i'th action.DO NOT MODIFY the returned object, make a copy of ir with "CSerializable::duplicate" if desired.
			  *  First element is 0.
			  * \exception std::exception On index out of bounds.
			  */
			CActionPtr get(size_t index);

			 /** Access to the i'th action of a given class, or a NULL smart pointer if there is no action of that class in the list.
			   *  Example:
			   * \code
					CActionRobotMovement2DPtr obs = acts->getActionByClass<CActionRobotMovement2D>();
			   * \endcode
			   * By default (ith=0), the first one is returned.
			   */
			 template <typename T>
			 typename T::SmartPtr getActionByClass( const size_t &ith = 0 ) const
			 {
				MRPT_START
				size_t  foundCount = 0;
				const mrpt::utils::TRuntimeClassId*	class_ID = T::classinfo;
				for (const_iterator it = begin();it!=end();++it)
					if ( (*it)->GetRuntimeClass()->derivedFrom( class_ID ) )
						if (foundCount++ == ith)
							return typename T::SmartPtr(it->get_ptr());
				return typename T::SmartPtr();	// Not found: return empty smart pointer
				MRPT_END
			 }


			/** Add a new object to the list.
			  */
			void  insert(CAction	&action);

			/** Returns the actions count in the collection.
			  */
			size_t  size();

			/** Returns the best pose increment estimator in the collection, based on the determinant of its pose change covariance matrix.
			  * \return The estimation, or NULL if none is available.
			  */
			CActionRobotMovement2DPtr  getBestMovementEstimation() const;

			/** Returns the pose increment estimator in the collection having the specified type.
			  * \return The estimation, or NULL if none is available.
			  */
			CActionRobotMovement2DPtr  getMovementEstimationByType( CActionRobotMovement2D::TEstimationMethod method);

			/** Look for the first 2D or 3D "odometry" found in this collection of actions, and return the "mean" increment of the robot according to it.
			  * \return true on success,false on no odometry found.
			  */
			bool getFirstMovementEstimationMean( mrpt::poses::CPose3D &out_pose_increment ) const;

			/** Look for the first 2D or 3D "odometry" found in this collection of actions, and return the "mean" increment of the robot and its covariance according to it.
			  * \return true on success,false on no odometry found.
			  */
			bool getFirstMovementEstimation( mrpt::poses::CPose3DPDFGaussian &out_pose_increment ) const;

			/** Remove an action from the list by its index.
			  * \exception std::exception On index out of bounds.
			  */
			void  eraseByIndex(const size_t & index);


		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CActionCollection, mrpt::utils::CSerializable, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
