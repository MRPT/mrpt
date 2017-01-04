/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::system;

CConfigFileBase::~CConfigFileBase()
{
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, double value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format(((std::abs(value)>1e-4 && std::abs(value)<1e4 ) || value==.0)? "%f" : "%e",value), name_padding_width,value_padding_width,comment);
}
void  CConfigFileBase::write(const std::string &section, const std::string &name, float value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format(((std::abs(value)>1e-4f && std::abs(value)<1e4f ) || value==.0f) ? "%f" : "%e",value), name_padding_width,value_padding_width,comment);
}

/** Write a generic string with optional padding and a comment field ("// ...") at the end of the line. */
void  CConfigFileBase::writeString(const std::string &section,const std::string &name, const std::string &str, const int name_padding_width, const int value_padding_width, const std::string &comment)
{
	if (name_padding_width<1 && value_padding_width<1 && comment.empty())
		this->writeString(section,name,str);  // Default (old) behavior.

	std::string name_pad;
	if (name_padding_width>=1)
	     name_pad = mrpt::format("%*s",-name_padding_width, name.c_str());  // negative width: printf right padding
	else name_pad = name;

	std::string value_pad;
	if (value_padding_width>=1)
	     value_pad = mrpt::format(" %*s",-value_padding_width, str.c_str());   // negative width: printf right padding
	else value_pad = str;

	if (!comment.empty())
	{
		value_pad += std::string(" // ");
		value_pad += comment;
	}

	this->writeString(section,name_pad,value_pad);
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

