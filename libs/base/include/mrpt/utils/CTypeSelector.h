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
#ifndef  CTypeSelector_H
#define  CTypeSelector_H

#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
namespace utils
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CTypeSelector, mrpt::utils::CSerializable )

	/** This class represents a std::string derived class which is also CSerializable
	 * \sa CSerializable
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CTypeSelector : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CTypeSelector )
	protected:
		/** The possibilities
		  */
		std::vector<std::string>	possibleTypes;

		/** The selected one:
		  */
		unsigned int				selection;

	public:
		/** Default constructor.
		  * \param posibilitiesList The list of options, as a comma-separated-string, for example: "type 1,other type 2,type_3"
		  * \param defaultType Default type value
		  */
		CTypeSelector(std::string posibilitiesList = "", std::string defaultType = "");

		/** Destructor
		  */
		virtual ~CTypeSelector();

		/** Returns the set of posibilities in the "type" represented by this class.
		  */
		void  getTypePosibilities( std::vector<std::string> &outPosibilities)const;

		/** Gets the currently selected type, from the set of posibilities.
		  * \sa setType,getTypePosibilities
		  * \exception std::exception If currently there is not a valid selection.
		  */
		std::string  getType() const;

		/** Fast check for a given type, returns true if the selection is exactly the specified type name.
		  */
		bool 	isType(const char *type) const;

		/** Fast check for a given type, returns true if the selection is exactly the specified type name.
		  */
		bool 	isType(const std::string &type) const;

		/** Sets the currently selected type.
		  * \sa getType,getTypePosibilities
		  * \exception std::exception On trying to select a type not in the list of posible values.
		  */
		void  setType(const std::string &type);

		/** Returns the index of a given type within the list of all possible types, or -1 if the given string is not a valid type.
		  */
		int checkTypeIndex(const std::string &type) const;

	}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
