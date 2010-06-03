/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
	return mrpt::utils::trim(readString(section, name, defaultValue,failIfNotFound ));
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

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<uint32_t>	&defaultValue,
	std::vector<uint32_t>		&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<uint32_t>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atoi(itSrc->c_str());
	}
}

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<int32_t>		&defaultValue,
	std::vector<int32_t>			&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<int32_t>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atoi(itSrc->c_str());
	}
}

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<uint64_t>	&defaultValue,
	std::vector<uint64_t>		&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<uint64_t>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atoi(itSrc->c_str());
	}
}

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<int64_t>		&defaultValue,
	std::vector<int64_t>			&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<int64_t>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atoi(itSrc->c_str());
	}
}



/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<float>		&defaultValue,
	std::vector<float>			&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<float>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = (float)atof(itSrc->c_str());
	}
}

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<double>			&defaultValue,
	std::vector<double>					&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<double>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atof(itSrc->c_str());
	}
}

/*---------------------------------------------------------------
					read_vector
 ---------------------------------------------------------------*/
void   CConfigFileBase::read_vector(
	const std::string				&section,
	const std::string				&name,
	const std::vector<bool>		&defaultValue,
	std::vector<bool>			&outValues,
	bool failIfNotFound ) const
{
	std::string aux ( readString(section, name, "",failIfNotFound ) );

	// Parse the text into a vector:
	std::vector<std::string>	tokens;
	mrpt::system::tokenize( aux,"[], \t",tokens);

	if (tokens.size()==0)
	{
		outValues = defaultValue;
	}
	else
	{
		// Parse to numeric type:
		outValues.resize( tokens.size() );
		std::vector<bool>::iterator itDest;
		std::vector<std::string>::iterator	itSrc;
		for (itSrc=tokens.begin(), itDest=outValues.begin();itSrc!=tokens.end();itSrc++,itDest++)
			*itDest = atoi(itSrc->c_str()) != 0;
	}
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



/** Reads a configuration parameter as a matrix written in a matlab-like format - for example: "[2 3 4 ; 7 8 9]"
 *  This template method can be instantiated for matrices of the types: int, long, unsinged int, unsigned long, float, double, long double
 * \exception std::exception If the key name is not found and "failIfNotFound" is true. Otherwise the "defaultValue" is returned.
 */
template <class T>
void CConfigFileBase::read_matrix(
	const std::string			&section,
	const std::string			&name,
	mrpt::math::CMatrixTemplate<T>	&outMatrix,
	const mrpt::math::CMatrixTemplate<T>	&defaultMatrix,
	bool failIfNotFound ) const
{
	std::string aux = readString(section, name, "",failIfNotFound );

	if (aux.empty())
		outMatrix = defaultMatrix;
	else
	{
		// Parse the text into a vector:
		if (!outMatrix.fromMatlabStringFormat(aux))
			THROW_EXCEPTION_CUSTOM_MSG1("Error parsing matrix: '%s'",aux.c_str())
	}
}

// Template instantiations:
// int, long, unsinged int, unsigned long, float, double, long double
template void BASE_IMPEXP CConfigFileBase::read_matrix<int>( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<int> &, const mrpt::math::CMatrixTemplate<int>&, bool ) const;
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<unsigned int> &, const mrpt::math::CMatrixTemplate<unsigned int>&, bool ) const;
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<long> &, const mrpt::math::CMatrixTemplate<long>&, bool ) const;
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<unsigned long> &, const mrpt::math::CMatrixTemplate<unsigned long>&, bool ) const;
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<float> &, const mrpt::math::CMatrixTemplate<float>&, bool ) const;
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<double> &, const mrpt::math::CMatrixTemplate<double>&, bool ) const;
#ifdef HAVE_LONG_DOUBLE
template void BASE_IMPEXP CConfigFileBase::read_matrix( const std::string &, const std::string &, mrpt::math::CMatrixTemplate<long double> &, const mrpt::math::CMatrixTemplate<long double>&, bool ) const;
#endif

