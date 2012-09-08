/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef  CConfigFile_H
#define  CConfigFile_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/safe_pointers.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This class allows loading and storing values and vectors of different types from ".ini" files easily.
	  *  The contents of the file will be modified by "write" operations in memory, and will be saved back
	  *   to the file at the destructor, and only if at least one write operation has been applied.
	 * \ingroup mrpt_base_grp
	  */
	class BASE_IMPEXP CConfigFile : public CConfigFileBase
	{
	private:
		/** The name of the file
		  */
		std::string		m_file;

		/** The interface to the file:
		  */
		void_ptr_noncopy m_ini;

		/** If modified since load.
		  */
		bool			m_modified;

	protected:
		/** A virtual method to write a generic string.
		  */
		void  writeString(const std::string &section,const std::string &name, const std::string &str);

		/** A virtual method to read a generic string.
         * \exception std::exception If the key name is not found and "failIfNotFound" is true. Otherwise the "defaultValue" is returned.
		 */
		std::string  readString(
            const std::string &section,
            const std::string &name,
            const std::string &defaultStr,
            bool failIfNotFound = false) const;

	public:
		/** Constructor that opens a configuration file. */
		CConfigFile( const std::string &fileName );

		/** Constructor, does not open any file. You should call "setFileName" before reading or writting or otherwise nothing will be read and write operations will be eventually lost.
		  * However, it's perfectly right to use this object without an associated file, in which case it will behave as an "in-memory" file.
		  */
		CConfigFile();

        /** Associate this object with the given file, so future read/write operations will be applied to that file (it's synchronized at destruction).
          */
		void setFileName(const std::string &fil_path);

        /** Dumps the changes to the physical configuration file now, not waiting until destruction. */
		void writeNow();

        /** Returns the file currently open by this object.
          */
		std::string getAssociatedFile() const { return m_file; }

		/** Destructor
		  */
        virtual ~CConfigFile();

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
