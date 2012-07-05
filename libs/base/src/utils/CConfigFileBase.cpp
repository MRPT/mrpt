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


#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/system/string_utils.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::system;

void  CConfigFileBase::write(const std::string &section, const std::string &name, double value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format("%e",value), name_padding_width,value_padding_width,comment);
}
void  CConfigFileBase::write(const std::string &section, const std::string &name, float value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format("%e",value), name_padding_width,value_padding_width,comment);
}
void  CConfigFileBase::write(const std::string &section, const std::string &name, int value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format("%i",value), name_padding_width,value_padding_width,comment);
}
void  CConfigFileBase::write(const std::string &section, const std::string &name, unsigned int value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format("%u",value), name_padding_width,value_padding_width,comment);
}
#if MRPT_WORD_SIZE>32
void  CConfigFileBase::write(const std::string &section, const std::string &name, size_t value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, format("%u",static_cast<unsigned int>(value)) , name_padding_width,value_padding_width,comment);
}
#endif

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::string &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	writeString(section, name, value , name_padding_width,value_padding_width,comment);
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<int> &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	string str;
	for (std::vector<int>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%i ", *it);
	writeString(section, name, str , name_padding_width,value_padding_width,comment);
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<unsigned int> &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	string str;
	for (std::vector<unsigned int>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%u ", *it);
	writeString(section, name, str , name_padding_width,value_padding_width,comment);
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<float> &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	string str;
	for (std::vector<float>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%.9e ", *it);
	writeString(section, name, str , name_padding_width,value_padding_width,comment);
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<double> &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	string str;
	for (std::vector<double>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%.16e ", *it);
	writeString(section, name, str , name_padding_width,value_padding_width,comment);
}

void  CConfigFileBase::write(const std::string &section, const std::string &name, const std::vector<bool> &value, const int name_padding_width, const int value_padding_width, const std::string &comment )
{
	string str;
	for (std::vector<bool>::const_iterator it=value.begin();it!=value.end();++it)
		str+=format("%c ", *it ? '1':'0');
	writeString(section, name, str , name_padding_width,value_padding_width,comment);
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
	else value_pad = name;

	if (!comment.empty())
	{
		value_pad += std::string("// ");
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

