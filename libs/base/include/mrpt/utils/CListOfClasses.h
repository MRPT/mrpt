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
#ifndef  CListOfClasses_H
#define  CListOfClasses_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/system/string_utils.h>

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
