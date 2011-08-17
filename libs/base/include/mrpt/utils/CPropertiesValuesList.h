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
#ifndef  CPropertiesValuesList_H
#define  CPropertiesValuesList_H

#include <mrpt/utils/CSerializable.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPropertiesValuesList, mrpt::utils::CSerializable )

		/** An arbitrary list of "annotations", or named attributes, each being an instance of any CSerializable object.
		 *  A multi-hypotheses version exists in CMHPropertiesValuesList.
		 * \sa CSerializable, CMHPropertiesValuesList, mrpt::utils::TParameters
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CPropertiesValuesList : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPropertiesValuesList )
		protected:
			struct BASE_IMPEXP  TPropertyValuePair
			{
				std::string			name;
				CSerializablePtr	value;
			};
			/** The properties list: a map between strings and objects
			  */
			std::vector<TPropertyValuePair>	m_properties;

		public:
			/** Default constructor
			  */
			CPropertiesValuesList();

			/** Copy constructor
			  */
			CPropertiesValuesList(const CPropertiesValuesList &o);

			/** Copy operator
			  */
			CPropertiesValuesList& operator = (const CPropertiesValuesList &o);

			/** Destructor
			  */
			virtual ~CPropertiesValuesList();

			/** Clears the list.
			  */
			void  clear();

			/** Returns the value of the property (case insensitive), or NULL if it does not exist.
			  */
			CSerializablePtr get(const std::string &propertyName) const;

			/** Sets/change the value of the property (case insensitive), making a copy of the object (or setting it to NULL if it is the passed value)
			  */
			void  set(const std::string &propertyName,const CSerializablePtr &obj);

			/** Returns the number of properties in the list
			  */
			size_t  size() const;

			/** Returns the name of all properties in the list
			  */
			std::vector<std::string>  getPropertyNames() const;

		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
