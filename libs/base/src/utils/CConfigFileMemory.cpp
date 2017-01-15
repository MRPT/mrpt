/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers



#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/system/os.h>

#include "simpleini/SimpleIni.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::utils::simpleini;
using namespace std;


#define THE_INI  static_cast<CSimpleIniA*>(m_ini.get())

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFileMemory::CConfigFileMemory( const utils::CStringList &stringList )
{
	// Create the object:
    m_ini = (void*) new CSimpleIniA();

    // Load the strings:
    std::string aux;
    stringList.getText(aux);
    THE_INI->Load( aux.c_str(), aux.size() );
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFileMemory::CConfigFileMemory( const std::string &str )
{
	// Create the object:
    m_ini = (void*) new CSimpleIniA();

    // Load the strings:
    THE_INI->Load( str.c_str(), str.size() );
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFileMemory::CConfigFileMemory()
{
	// Create the empty object:
    m_ini = (void*) new CSimpleIniA();
}

/** Copy constructor */
CConfigFileMemory::CConfigFileMemory(const CConfigFileMemory& o)
{
	// Create the empty object:
    m_ini = (void*) new CSimpleIniA();
	(*this) = o;
}

/** Copy operator */
CConfigFileMemory& CConfigFileMemory::operator = (const CConfigFileMemory& o)
{
	std::string  str;
	static_cast<const CSimpleIniA*>(o.m_ini.get())->Save(str);
	THE_INI->Load(str.c_str(),str.size());
	return *this;
}


/*---------------------------------------------------------------
					setContent
 ---------------------------------------------------------------*/
void CConfigFileMemory::setContent(  const utils::CStringList &stringList  )
{
    // Load the strings:
    std::string aux;
    stringList.getText(aux);
    THE_INI->Load( aux.c_str(), aux.size() );
}

/*---------------------------------------------------------------
					setContent
 ---------------------------------------------------------------*/
void CConfigFileMemory::setContent(  const std::string &str )
{
    THE_INI->Load( str.c_str(), str.size() );
}

/*---------------------------------------------------------------
					getContent
 ---------------------------------------------------------------*/
void CConfigFileMemory::getContent( std::string &str ) const
{
	((CSimpleIniA*)(m_ini.get()))->Save( str, false );
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CConfigFileMemory::~CConfigFileMemory(  )
{
    delete THE_INI;
}

/*---------------------------------------------------------------
					writeString
 ---------------------------------------------------------------*/
void  CConfigFileMemory::writeString(const std::string &section,const std::string &name, const std::string &str)
{
    MRPT_START

	SI_Error ret = THE_INI->SetValue( section.c_str(),name.c_str(),str.c_str(), NULL );
    if (ret<0)
        THROW_EXCEPTION("Error changing value in INI-style file!");

    MRPT_END
}

/*---------------------------------------------------------------
					readString
 ---------------------------------------------------------------*/
std::string  CConfigFileMemory::readString(
    const std::string &section,
    const std::string &name,
    const std::string &defaultStr,
    bool failIfNotFound) const
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
        string tmpStr( format("Value '%s' not found in section '%s' of memory configuration string list and failIfNotFound=true.",name.c_str(),section.c_str() ) );
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
					readString
 ---------------------------------------------------------------*/
void CConfigFileMemory::getAllSections( vector_string	&sections ) const
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
void CConfigFileMemory::getAllKeys( const string &section, vector_string	&keys ) const
{
	CSimpleIniA::TNamesDepend	names;
	static_cast<const CSimpleIniA*>(m_ini.get())->GetAllKeys(section.c_str(), names);

	CSimpleIniA::TNamesDepend::iterator		n;
	vector_string::iterator		s;
	keys.resize(names.size());
	for ( n = names.begin(), s = keys.begin(); n!=names.end();++n,++s)
		*s = n->pItem;
}
