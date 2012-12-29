/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef  CConfigFileMemory_H
#define  CConfigFileMemory_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/safe_pointers.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This class implements a config file-like interface over a memory-stored string list.
	 * \ingroup mrpt_base_grp
	  */
	class BASE_IMPEXP CConfigFileMemory : public CConfigFileBase
	{
	private:
		/** The IniFile object
		  */
		void_ptr_noncopy		m_ini;

	protected:
		/** A virtual method to write a generic string.
		  */
		void  writeString(const std::string &section,const std::string &name, const std::string &str);

		/** A virtual method to read a generic string.
		  */
		std::string  readString(
            const std::string &section,
            const std::string &name,
            const std::string &defaultStr,
            bool failIfNotFound = false) const;

	public:
		/** Constructor and initialize from a list of strings */
		CConfigFileMemory( const utils::CStringList &stringList );

		/** Constructor and initialize from string with the whole "config file"  */
		CConfigFileMemory( const std::string &str );

		/** Empty constructor. Upon construction, call any of the "setContent" method. */
		CConfigFileMemory();

		/** Copy constructor */
		CConfigFileMemory(const CConfigFileMemory& o);

		/** Copy operator */
		CConfigFileMemory& operator = (const CConfigFileMemory& o);

		/** Changes the contents of the virtual "config file" */
		void setContent(  const utils::CStringList &stringList  );

		/** Changes the contents of the virtual "config file" */
		void setContent(  const std::string &str );

		/** Return the currnet contents of the virtual "config file" */
		void getContent(  std::string &str ) const;

		/** Return the currnet contents of the virtual "config file" */
		inline std::string getContent() const {  std::string s; getContent(s); return s; }

		/** Destructor
		 */
		virtual ~CConfigFileMemory( );

		/** Returns a list with all the section names.
		  */
		virtual void getAllSections( vector_string	&sections ) const;

		/** Returs a list with all the keys into a section.
		  */
		virtual void getAllKeys( const std::string section, vector_string	&keys ) const;

	}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
