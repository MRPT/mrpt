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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/CTypeSelector.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/string_utils.h>

#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CTypeSelector, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CTypeSelector::writeToStream(CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		std::vector<std::string>::const_iterator	it;
		uint32_t							n = (uint32_t)possibleTypes.size();

		out << n;

		for (it=possibleTypes.begin();it<possibleTypes.end();it++)
			out << *it;

		// Selection:
		out << (uint32_t)selection;
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CTypeSelector::readFromStream(CStream &in, int version)
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
	vector<string> tokens;
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

