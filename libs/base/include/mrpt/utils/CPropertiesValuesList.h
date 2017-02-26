/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CPropertiesValuesList_H
#define  CPropertiesValuesList_H

#include <mrpt/utils/CSerializable.h>

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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPropertiesValuesList, mrpt::utils::CSerializable )


	} // End of namespace
} // End of namespace

#endif
