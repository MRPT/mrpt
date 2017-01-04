/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CConfigFileMemory_H
#define  CConfigFileMemory_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/safe_pointers.h>

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
		void_ptr_noncopy		m_ini; //!< The IniFile object

	protected:
		/** A virtual method to write a generic string. */
		void  writeString(const std::string &section,const std::string &name, const std::string &str) MRPT_OVERRIDE;

		/** A virtual method to read a generic string.
		  */
		std::string  readString(
            const std::string &section,
            const std::string &name,
            const std::string &defaultStr,
			bool failIfNotFound = false) const MRPT_OVERRIDE;

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

		/** Returns a list with all the section names */
		void getAllSections( vector_string	&sections ) const MRPT_OVERRIDE;

		/** Returs a list with all the keys into a section */
		void getAllKeys( const std::string &section, vector_string	&keys ) const MRPT_OVERRIDE;

	}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
