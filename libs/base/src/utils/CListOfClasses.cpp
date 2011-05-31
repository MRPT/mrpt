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
#include <mrpt/base.h>  // Precompiled headers

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

