/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
