/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CListOfClasses.h>

using namespace mrpt;
using namespace mrpt::utils;


bool CListOfClasses::containsDerivedFrom( const mrpt::utils::TRuntimeClassId* id ) const
{ 
	for (const_iterator it=begin();it!=end();++it)
		if ( (*it)->derivedFrom(id) )
			return true;
	return false;
}

std::string CListOfClasses::toString() const
{
	std::string s;
	for (const_iterator it=begin();it!=end();++it)
	{
		if (it!=begin()) s+=", ";
		s+=std::string( (*it)->className );
	}
	return s;			
}

void CListOfClasses::fromString(const std::string &s)
{
	MRPT_TRY_START
	
	this->clear();
	std::vector<std::string> lstClasses;
	mrpt::system::tokenize(s," ,",lstClasses);
	
	for (size_t i=0;i<lstClasses.size();i++)
	{
		const mrpt::utils::TRuntimeClassId* id = mrpt::utils::findRegisteredClass(lstClasses[i]);
		ASSERTMSG_(id!=NULL, format("Unknown class name: %s",lstClasses[i].c_str()))
		this->insert(id);
	}

	MRPT_TRY_END
}

