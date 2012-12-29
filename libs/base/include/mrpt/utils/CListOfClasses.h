/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
