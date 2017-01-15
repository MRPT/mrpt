/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CTypeSelector, mrpt::utils::CSerializable )

	} // End of namespace
} // End of namespace
#endif
