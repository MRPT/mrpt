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
