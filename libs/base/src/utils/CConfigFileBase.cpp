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


#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/system/string_utils.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::system;

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, double value)
{
	writeString(section, name, format("%e",value));
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, float value)
{
	writeString(section, name, format("%e",value) );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, int value)
{
	writeString(section, name, format("%i",value) );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, unsigned int value)
{
	writeString(section, name, format("%u",value) );
}

#if MRPT_WORD_SIZE>32
/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, size_t value)
{
	writeString(section, name, format("%u",static_cast<unsigned int>(value)) );
}
#endif


/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::string &value)
{
	writeString(section, name, value );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<int> &value)
{
	string str;
	for (std::vector<int>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%i ", *it);
	writeString(section, name, str  );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<unsigned int> &value)
{
	string str;
	for (std::vector<unsigned int>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%u ", *it);
	writeString(section, name, str  );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<float> &value)
{
	string str;
	for (std::vector<float>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%.9e ", *it);
	writeString(section, name, str  );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<double> &value)
{
	string str;
	for (std::vector<double>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%.16e ", *it);
	writeString(section, name, str  );
}

/*---------------------------------------------------------------
					write
 ---------------------------------------------------------------*/
void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<bool> &value)
{
	string str;
	for (std::vector<bool>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%c ", *it ? '1':'0');
	writeString(section, name, str  );
}

/*---------------------------------------------------------------
					read_double
 ---------------------------------------------------------------*/
double  CConfigFileBase::read_double(const std::string &section, const std::string &name, double defaultValue, bool failIfNotFound ) const
{
	return atof( readString(section,name,format("%.16e",defaultValue),failIfNotFound).c_str());
}

/*---------------------------------------------------------------
					read_float
 ---------------------------------------------------------------*/
float  CConfigFileBase::read_float(const std::string &section, const std::string &name, float defaultValue, bool failIfNotFound ) const
{
	return (float)atof( readString(section,name,format("%.10e",defaultValue),failIfNotFound).c_str());
}

/*---------------------------------------------------------------
					read_int
 ---------------------------------------------------------------*/
int  CConfigFileBase::read_int(const std::string &section, const std::string &name, int defaultValue, bool failIfNotFound ) const
{
	return atoi( readString(section,name,format("%i",defaultValue),failIfNotFound).c_str());
}


/*---------------------------------------------------------------
					read_uint64_t
 ---------------------------------------------------------------*/
uint64_t CConfigFileBase::read_uint64_t(const std::string &section, const std::string &name, uint64_t defaultValue, bool failIfNotFound ) const
{
	string s = readString(section,name,format("%lu",(long unsigned int)defaultValue),failIfNotFound);
	return mrpt::system::os::_strtoull(s.c_str(),NULL, 0);
}

/*---------------------------------------------------------------
					read_bool
 ---------------------------------------------------------------*/
bool  CConfigFileBase::read_bool(const std::string &section, const std::string &name, bool defaultValue, bool failIfNotFound ) const
{
	const string s = mrpt::system::lowerCase(trim(readString(section,name,string(defaultValue ? "1":"0"),failIfNotFound)));
	if (s=="true") return true;
	if (s=="false") return false;
	if (s=="yes") return true;
	if (s=="no") return false;
	return (0 != atoi(s.c_str()));
}

/*---------------------------------------------------------------
					read_string
 ---------------------------------------------------------------*/
std::string  CConfigFileBase::read_string(const std::string &section, const std::string &name, const std::string &defaultValue, bool failIfNotFound ) const
{
	return mrpt::system::trim(readString(section, name, defaultValue,failIfNotFound ));
}

/*---------------------------------------------------------------
					read_string_first_word
 ---------------------------------------------------------------*/
std::string  CConfigFileBase::read_string_first_word(const std::string &section, const std::string &name, const std::string &defaultValue, bool failIfNotFound ) const
{
	string s = readString(section, name, defaultValue,failIfNotFound );
	vector_string  auxStrs;
	mrpt::system::tokenize(s,"[], \t", auxStrs);
	if (auxStrs.empty())
	{
		if (failIfNotFound)
		{
			THROW_EXCEPTION( format("Value '%s' seems to be present in section '%s' but, are all whitespaces??",
				name.c_str(),
				section.c_str()  ) );
		}
		else return "";
	}
	else return auxStrs[0];
}

/** Checks if a given section exists (name is case insensitive) */
bool CConfigFileBase::sectionExists( const std::string &section_name) const
{
	vector_string sects;
	getAllSections(sects);
	for (vector_string::iterator s=sects.begin();s!=sects.end();++s)
		if (!mrpt::system::os::_strcmpi(section_name.c_str(), s->c_str() ))
			return true;
	return false;
}

