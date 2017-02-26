/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CListOfClasses_H
#define  CListOfClasses_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/system/string_utils.h>
#include <set>

namespace mrpt
{
	namespace utils
	{
		/** A list (actually based on a std::set) of MRPT classes, capable of keeping any class registered by the mechanism of CSerializable classes.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CListOfClasses
		{
		private:
			typedef std::set<const mrpt::utils::TRuntimeClassId*>  TSet;
			
			TSet m_data;
			
		public:
			typedef TSet::iterator iterator;
			typedef TSet::const_iterator const_iterator;
			
			inline iterator begin() {return m_data.begin(); }
			inline const_iterator begin() const {return m_data.begin(); }
			
			inline iterator end() {return m_data.end(); }
			inline const_iterator end() const {return m_data.end(); }
		
			/** Insert a class in the list. Example of usage:
			  *   \code
			  *     myList.insert(CLASS_ID(CObservationImage));
			  *   \endcode
			  */
			inline void insert( const mrpt::utils::TRuntimeClassId* id ) { m_data.insert(id); }		

			/** Does the list contains this class? */
			inline bool contains( const mrpt::utils::TRuntimeClassId* id ) const { return m_data.find(id)!=m_data.end(); }

			/** Does the list contains a class derived from...? */
			bool containsDerivedFrom( const mrpt::utils::TRuntimeClassId* id ) const;
			
			/** Empty the list */
			inline void clear() { m_data.clear(); }
			
			/** Is the list empty? */
			inline bool empty() const  { return m_data.empty(); }
			
			/** Return a string representation of the list, for example: "CPose2D, CObservation, CPose3D".
			  */
			std::string toString() const;
			
			/** Return a string representation of the list, for example: "CPose2D, CObservation, CPose3D".
			  * \exception std::exception On unregistered class name found.
			  */
			void fromString(const std::string &s);
			
		}; // end of class

	} // End of namespace
} // end of namespace
#endif
