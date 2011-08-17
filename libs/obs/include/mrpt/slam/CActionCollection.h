/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CActionCollection_H
#define CActionCollection_H

#include <mrpt/slam/CAction.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt
{
	namespace slam
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CActionCollection, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a class for storing a collection of robot actions. It is used in mrpt::slam::CRawlog,
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
			/** The actions:
			  */
			std::deque<CActionPtr>	m_actions;

		 public:
			 /** Constructor
			   */
			CActionCollection();

			/** Constructor from a single action.
			   */
			CActionCollection( CAction &a );

			/** Copy Constructor
			   */
			CActionCollection(const CActionCollection &o );

			/** Copy operator
			  */
			CActionCollection&  operator = (const CActionCollection &o );

			/** Destructor
			   */
			virtual ~CActionCollection();

			/** You can use CActionCollection::begin to get a iterator to the first element.
			  */
			typedef std::deque<CActionPtr>::iterator		iterator;

			/** You can use CActionCollection::begin to get a iterator to the first element.
			  */
			typedef std::deque<CActionPtr>::const_iterator	const_iterator;

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
							return typename T::SmartPtr(*it);
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
			bool getFirstMovementEstimationMean( CPose3D &out_pose_increment ) const;

			/** Look for the first 2D or 3D "odometry" found in this collection of actions, and return the "mean" increment of the robot and its covariance according to it.
			  * \return true on success,false on no odometry found.
			  */
			bool getFirstMovementEstimation( CPose3DPDFGaussian &out_pose_increment ) const;

			/** Remove an action from the list by its index.
			  * \exception std::exception On index out of bounds.
			  */
			void  eraseByIndex(const size_t & index);


		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
