/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/os.h>
#include "simpleini/SimpleIni.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::utils::simpleini;
using namespace std;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFile::CConfigFile( const std::string &fileName )
{
    MRPT_START

	m_file = fileName;
	m_modified = false;
    m_ini = (void*) new CSimpleIniA();
    static_cast<CSimpleIniA*>(m_ini.get())->LoadFile(fileName.c_str());


    MRPT_END
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFile::CConfigFile()
{
    MRPT_START

	m_file = "";
	m_modified = false;
    m_ini = (void*) new CSimpleIniA();

    MRPT_END
}

/*---------------------------------------------------------------
					setFileName
 ---------------------------------------------------------------*/
void CConfigFile::setFileName(const std::string &fil_path)
{
    MRPT_START

	m_file = fil_path;
	m_modified = false;

    static_cast<CSimpleIniA*>(m_ini.get())->LoadFile(fil_path.c_str());
    MRPT_END
}

/*---------------------------------------------------------------
					writeNow
 ---------------------------------------------------------------*/
void CConfigFile::writeNow()
{
    MRPT_START
	if (m_modified && !m_file.empty())
	{
	    static_cast<CSimpleIniA*>(m_ini.get())->SaveFile( m_file.c_str() );
	    m_modified = false;
	}
    MRPT_END
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CConfigFile::~CConfigFile()
{
    writeNow();
    delete static_cast<CSimpleIniA*>(m_ini.get());
}


/*---------------------------------------------------------------
					writeString
 ---------------------------------------------------------------*/
void  CConfigFile::writeString(const std::string &section,const std::string &name, const std::string &str)
{
    MRPT_START

	m_modified = true;

    if (0 > static_cast<CSimpleIniA*>(m_ini.get())->SetValue( section.c_str(),name.c_str(),str.c_str(), NULL ))
        THROW_EXCEPTION("Error changing value in INI-style file!");

    MRPT_END

}

/*---------------------------------------------------------------
					readString
 ---------------------------------------------------------------*/
std::string  CConfigFile::readString(
    const std::string &section,
    const std::string &name,
    const std::string &defaultStr,
    bool failIfNotFound ) const
{
    MRPT_START
    const char *defVal = failIfNotFound ? NULL :defaultStr.c_str();

    const char *aux = static_cast<const CSimpleIniA*>(m_ini.get())->GetValue(
        section.c_str(),
        name.c_str(),
        defVal,
        NULL );     // The memory is managed by the SimpleIni object

    if (failIfNotFound && !aux )
    {
        string tmpStr( format("Value '%s' not found in section '%s' of file '%s' and failIfNotFound=true.",
			name.c_str(),
			section.c_str(),
			m_file.c_str() ) );
        THROW_EXCEPTION(tmpStr);
    }

	// Remove possible comments: "//"
	std::string ret = aux;
	size_t  pos;
	if ((pos=ret.find("//"))!=string::npos && pos>0 && isspace(ret[pos-1]))
		ret = ret.substr(0,pos);
	return ret;

    MRPT_END
}

/*---------------------------------------------------------------
					 getAllSections
 ---------------------------------------------------------------*/
void CConfigFile::getAllSections( vector_string	&sections ) const
{
	CSimpleIniA::TNamesDepend	names;
	static_cast<const CSimpleIniA*>(m_ini.get())->GetAllSections(names);

	CSimpleIniA::TNamesDepend::iterator		n;
	vector_string::iterator		s;
	sections.resize(names.size());
	for (n=names.begin(),s=sections.begin(); n!=names.end();++n,++s)
		*s = n->pItem;
}


/*---------------------------------------------------------------
					  getAllKeys
 ---------------------------------------------------------------*/
void CConfigFile::getAllKeys( const string &section, vector_string	&keys ) const
{
	CSimpleIniA::TNamesDepend	names;
	static_cast<const CSimpleIniA*>(m_ini.get())->GetAllKeys(section.c_str(), names);

	CSimpleIniA::TNamesDepend::iterator		n;
	vector_string::iterator		s;
	keys.resize(names.size());
	for ( n = names.begin(), s = keys.begin(); n!=names.end();++n,++s)
		*s = n->pItem;
}
