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
