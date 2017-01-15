/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CTypeSelector.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CTypeSelector, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CTypeSelector::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		std::vector<std::string>::const_iterator	it;
		uint32_t							n = (uint32_t)possibleTypes.size();

		out << n;

		for (it=possibleTypes.begin();it<possibleTypes.end();++it)
			out << *it;

		// Selection:
		out << (uint32_t)selection;
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CTypeSelector::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;

			in >> n;

			possibleTypes.clear();

			for (i=0;i<n;i++)
			{
				std::string		aux;
				in >> aux;
				possibleTypes.push_back( aux );
			}

			// Selection:
			in >> i; selection = i;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CTypeSelector::CTypeSelector(	std::string		posibilitiesList,
								std::string		defaultType )
{
	// Init:
	possibleTypes.clear();

	// Extract tokens from the string:
	mrpt::system::tokenize(posibilitiesList,",",possibleTypes);

	// Select default type:
	setType(defaultType);
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CTypeSelector::~CTypeSelector()
{
}

/*---------------------------------------------------------------
					getType
 ---------------------------------------------------------------*/
std::string  CTypeSelector::getType()const
{
	if (selection>=possibleTypes.size())
		THROW_EXCEPTION("current selection index is out of bounds!");

	return possibleTypes[selection];
}

/*---------------------------------------------------------------
						setType
 ---------------------------------------------------------------*/
void  CTypeSelector::setType(const std::string  &type)
{
	size_t		i,n = possibleTypes.size();

	for (i=0;i<n;i++)
	{
		if (!os::_strcmpi(type.c_str(),possibleTypes[i].c_str()))
		{
			// Select:
			selection = (unsigned int)i;
			return;
		}
	}

	// Not in the list:
	THROW_EXCEPTION_CUSTOM_MSG1("Type '%s' is not one of the posibilities", type.c_str());
}

/*---------------------------------------------------------------
					getTypePosibilities
 ---------------------------------------------------------------*/
void  CTypeSelector::getTypePosibilities( std::vector<std::string> &outPosibilities)const
{
	outPosibilities = possibleTypes;
}

/*---------------------------------------------------------------
					isType
 ---------------------------------------------------------------*/
bool 	CTypeSelector::isType(const char *type) const
{
	return !strcmp( possibleTypes[selection].c_str(),type);
}

/*---------------------------------------------------------------
					isType
 ---------------------------------------------------------------*/
bool 	CTypeSelector::isType(const std::string &type) const
{
	return !strcmp( possibleTypes[selection].c_str(),type.c_str());
}

/*---------------------------------------------------------------
					checkTypeIndex
 ---------------------------------------------------------------*/
int CTypeSelector::checkTypeIndex(const std::string &type) const
{
	size_t	n = possibleTypes.size();
	for (size_t i=0;i<n;i++)
		if (!os::_strcmpi(type.c_str(),possibleTypes[i].c_str()))
			return int(i);	// Found:
	return -1; // Not found
}

